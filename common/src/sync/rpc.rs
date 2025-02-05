use core::{marker::PhantomData, sync::atomic::Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as M, channel::Channel};
use maitake_sync::WaitMap;
use portable_atomic::AtomicUsize;

use super::channel::MpscChannel;

/// Define the messages to use with a [`Procedure`] instance.
///
/// # Usage
/// ```rust
/// rpc_message!{
///     // Define the procedure
///     Msg: Request => Response,
/// //  ^    ^          ^
/// //  |    |          | This is the 'generic' request type
/// //  |    | The request type is what is received, and holds the responder handle
/// //  | This type marks the protocol, by relating the request and response types
///
///     // Define individual messages
///     async GetTime {}            -> Instant,
///     async SetTarget { t: f32 }  -> bool,
///     async GetTarget {}          -> f32,
/// }
///
/// /// Define the procedure globally
/// static PROC: Procedure<Msg> = Procedure::new();
///
/// ```
#[macro_export]
macro_rules! rpc_message {
    (
        $def_name:ident : $req_name:ident -> $res_name:ident $(,)?

        $(
            $(#[$attr:meta])*
            async $variant:ident {
                $(
                    $(#[$inner:meta])*
                    $named:ident : $req_type:ty
                ),*
            } -> $res_type:ty $(,)?
        ),
    +) => {

        pub enum $res_name {
            $(
                $variant($res_type),
            )+
        }

        pub enum $req_name<'a> {
            $(
                $variant($crate::sync::rpc::Handle<'a, $def_name, $variant>, $variant),
            )+
        }

        pub struct $def_name;

        impl <'a> $crate::sync::rpc::Protocol <'a> for $def_name {
            type Res = $res_name;
            type Req = $req_name<'a>;
        }

        $(
            #[allow(unused)]
            $(#[$attr])*
            pub struct $variant {
                $(
                    $(#[$inner])*
                    pub $named : $req_type
                ),*
            }
            impl <'a> $crate::sync::rpc::Call<'a, $def_name> for $variant {
                type Res = $res_type;

                fn make_request(self, proc: &'a $crate::sync::rpc::Procedure<'a, $def_name>, idx: usize) -> $req_name<'a> {
                    $req_name::$variant($crate::sync::rpc::Handle::new(proc, idx), self)
                }

                fn make_response(res: Self::Res) -> $res_name {
                    $res_name::$variant(res)
                }

                fn get_response(res: $res_name) -> Option<Self::Res> {
                    match res {
                        $res_name::$variant(ack) => Some(ack),
                        #[allow(unreachable_patterns)]
                        _ => None
                    }
                }
            }
        )+
    };
}

pub trait Protocol<'a> {
    type Req: 'a;
    type Res;
}

pub trait Call<'a, P: Protocol<'a>>: Sized {
    type Res;
    fn make_request(self, proc: &'a Procedure<'a, P>, idx: usize) -> P::Req;
    fn make_response(res: Self::Res) -> P::Res;
    fn get_response(res: P::Res) -> Option<Self::Res>;
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("The incorrect response type was sent for this request")]
    IncorrectResponse,
    #[error("The handle used to respond to this request was dropped before making a response")]
    HandleDropped,
    #[error("The RPC channel is closed, and no further requests can be made ")]
    RpcClosed,
}

pub struct Procedure<'a, R: Protocol<'a>> {
    count: portable_atomic::AtomicUsize,
    ch: MpscChannel<R::Req, 2>,
    map: maitake_sync::wait_map::WaitMap<usize, Option<R::Res>>,
}

impl <'a, P: Protocol<'a>> Procedure<'a, P> {

    pub const fn new() -> Self {
        Self {
            count: AtomicUsize::new(0),
            ch: MpscChannel::new(),
            map: WaitMap::new(),
        }
    }

    /// Make a request to the procedure with a specific `Call` implementor.
    /// This ensures the correct return type is received for the request.
    pub async fn request<C: Call<'a, P>>(&'a self, req: C) -> Result<C::Res, Error> {

        let idx = self.new_index();

        // Insert outselves in the queue for the ID
        let waiter = self.map.wait(idx);
        futures::pin_mut!(waiter);

        waiter.as_mut().subscribe().await.ok()
            .ok_or(Error::RpcClosed)?;

        // Do a lazy send, such that the request is only generated
        // once there is a free slot in the channel.
        self.ch.send_lazy(|| req.make_request(self, idx)).await;

        // Wait for the response
        let response = waiter.await.ok()
            .ok_or(Error::RpcClosed)?
            .ok_or(Error::HandleDropped)?;

        // Extract the correct response type
        C::get_response(response)
            .ok_or(Error::IncorrectResponse)
    }

    /// Make a request to the procedure
    async fn request_inner(&'a self, req: P::Req, idx: usize) -> Result<P::Res, Error> {

        // Insert outselves in the queue for the ID
        let waiter = self.map.wait(idx);
        futures::pin_mut!(waiter);

        waiter.as_mut().subscribe().await.ok()
            .ok_or(Error::RpcClosed)?;

        // Send the request associated with the ID
        self.ch.send(req).await;

        // Wait for the response
        waiter.await.ok()
            .ok_or(Error::RpcClosed)?
            .ok_or(Error::HandleDropped)
    }

    /// Receive a procedure request
    pub async fn get_request(&self) -> P::Req {
        self.ch.receive().await
    }

    fn new_index(&self) -> usize {
        self.count.fetch_add(1, Ordering::Relaxed)
    }
}

/// Handle to produce a response to a specific request
pub struct Handle<'a, P: Protocol<'a>, C: Call<'a, P>> {
    idx: usize,
    proc: &'a Procedure<'a, P>,
    _p: PhantomData<C>,
}

impl <'a, R: Protocol<'a>, C: Call<'a, R>> Handle<'a, R, C> {

    /// Construc a new responder handle.
    ///
    /// # Note
    ///
    /// Thisis not meant to be used directly, and is mostly
    /// public so that the `rpc_message` macro can use it,
    /// though manual implementation is possible as well.
    pub fn new(proc: &'a Procedure<'a, R>, idx: usize) -> Self {
        Self { idx, proc, _p: PhantomData }
    }

    /// Respond to the request, consuming this handle
    /// and waking the requestee.
    pub fn respond(self, res: C::Res) {
        self.proc.map.wake(&self.idx, Some(C::make_response(res)));
    }

    /// Notify the requestor that the request is closed.
    /// This will also happen automatically if the handle
    /// is dropped without using [`Handle::respond`].
    pub fn close(&self) {
        self.proc.map.wake(&self.idx, None);
    }
}

impl <'a, P: Protocol<'a>, C: Call<'a, P>> Drop for Handle<'a, P, C> {
    fn drop(&mut self) {
        self.close();
    }
}

// ---- playground ----

#[cfg(test)]
mod tests{

    use futures::task::SpawnExt;

    use super::*;

    rpc_message!{
        Msg: Request -> Response

        /// # Make a request to set the value.
        async SetValue {val: f32} -> bool,

        /// # Make a request to get the value.
        async GetValue {} -> f32,

        /// # Make a request to exit the test
        async Exit {} -> (),
    }

    #[test]
    fn test() {
        static PROCEDURE: Procedure<Msg> = Procedure::new();
        let mut spawner = futures_executor::LocalPool::new();

        spawner.spawner().spawn(async{
            let mut value = 5.0;

            let mut delayed: Option<Handle<'_, Msg, GetValue>> = None;

            loop {

                if let Some(handle) = delayed.take() {
                    handle.respond(value);
                }

                match PROCEDURE.get_request().await {
                    Request::SetValue(handle, req) => {
                        if req.val > 0.0 {
                            value = req.val;
                            handle.respond(true);
                        } else {
                            handle.respond(false);
                        }
                    },
                    Request::GetValue(handle, ..) => {
                        delayed = Some(handle);
                    },
                    Request::Exit(handle, ..) => {
                        handle.respond(());
                        break;
                    }
                }
            }
        }).unwrap();

        spawner.spawner().spawn(async { requester().await.unwrap() }).unwrap();

        async fn requester() -> Result<(), Error> {

            assert_eq!(PROCEDURE.request(GetValue{}).await?, 5.0);

            assert_eq!(PROCEDURE.request(SetValue{val: -1.0}).await?, false);
            assert_eq!(PROCEDURE.request(GetValue{}).await?, 5.0);

            assert_eq!(PROCEDURE.request(SetValue{val: 2.5}).await?, true);
            assert_eq!(PROCEDURE.request(GetValue{}).await?, 2.5);

            assert_eq!(PROCEDURE.request(Exit{}).await?, ());

            Ok(())
        }

        spawner.run();
    }
}
