use core::{marker::PhantomData, sync::atomic::Ordering};

use embassy_futures::join::join;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as M, channel::Channel};
use embassy_time::Instant;
use maitake_sync::WaitMap;
use portable_atomic::AtomicUsize;

macro_rules! rpc_message {
    (
        REQ = $req_name:ident $(,)?
        RES = $res_name:ident $(,)?
        $(
            $(#[$attr:meta])*
            fn $variant:ident $(($req_type:ty))? => $res_type:ty $(,)?
        ),
    +) => {
        pub enum $req_name<'a, 'b> {
            $(
                $(#[$attr])*
                $variant(Handle<'a, 'b, $req_name<'a, 'b>, $res_name, requests::$variant>),
            )+
        }

        pub enum $res_name {
            $(
                $variant($res_type),
            )+
        }

        pub mod requests {

            use $crate::sync::procedure::*;

            $(
                #[allow(unused)]
                pub struct $variant $((pub $req_type))?;
                impl <'a, 'b> Rpc<'b, $req_name<'a, 'b>, $res_name> for $variant
                {
                    type Res = $res_type;

                    fn as_request(self, idx: usize, rpc: &'b Procedure<$req_name<'a, 'b>, $res_name>) -> $req_name<'a, 'b> {
                        $req_name::$variant(Handle::new(idx, rpc, self))
                    }

                    fn make_response(res: Self::Res) -> $res_name {
                        $res_name::$variant(res)
                    }

                    fn get_response(res: $res_name) -> Option<Self::Res> {
                        match res {
                            $res_name::$variant(ack) => Some(ack),
                            _ => None
                        }
                    }
                }
            )+
        }
    };
}



pub trait Rpc<'b, REQ, RES>: Sized {
    type Res;
    fn as_request(self, idx: usize, rpc: &'b Procedure<REQ, RES>) -> REQ;
    fn make_response(res: Self::Res) -> RES;
    fn get_response(res: RES) -> Option<Self::Res>;
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

pub struct Procedure<REQ, RES> {
    count: portable_atomic::AtomicUsize,
    chn: embassy_sync::channel::Channel<M, REQ, 2>,
    map: maitake_sync::wait_map::WaitMap<usize, Option<RES>>,
}

impl <REQ, RES> Procedure<REQ, RES> {

    pub const fn new() -> Self {
        Self {
            count: AtomicUsize::new(0),
            chn: Channel::new(),
            map: WaitMap::new(),
        }
    }

    pub async fn request<'b, R: Rpc<'b, REQ, RES>>(&'b self, req: R) -> Result<R::Res, Error> {

        // Obtain a unique ID for this request
        let idx = self.count.fetch_add(1, Ordering::Relaxed);

        // Construct the full request context
        let req = req.as_request(idx, self);

        // Use join to first register the idx, and then send the request
        let (wait_result, _) = join(
            self.map.wait(idx),
            self.chn.send(req)
        ).await;

        // Wait for the response
        let response = wait_result.ok()
            .ok_or(Error::RpcClosed)?
            .ok_or(Error::HandleDropped)?;

        // extract the correct response type
        R::get_response(response)
            .ok_or(Error::IncorrectResponse)
    }

    pub async fn get_request(&self) -> REQ {
        self.chn.receive().await
    }
}

/// Handle to produce a response to a specific request
pub struct Handle<'a, 'b, REQ, RES, R: Rpc<'b, REQ, RES>> {
    idx: usize,
    map: &'b Procedure<REQ, RES>,
    _p: PhantomData<&'a REQ>,
    request: R,
}

impl <'a, 'b, REQ, RES, R: Rpc<'b, REQ, RES>> Handle<'a, 'b, REQ, RES, R> {

    fn new(idx: usize, map: &'b Procedure<REQ, RES>, request: R) -> Self {
        Self {
            idx, map, request, _p: PhantomData
        }
    }

    /// Respond to the request, consuming this handle
    pub fn respond(self, res: R::Res) {
        self.map.map.wake(&self.idx, Some(R::make_response(res)));
    }

    /// Cancel this request, notifying the requestee
    pub fn cancel(&self) {
        self.map.map.wake(&self.idx, None);
    }
}

impl <'a, 'b, REQ, RES, R: Rpc<'b, REQ, RES>> Drop for Handle <'a, 'b, REQ, RES, R> {
    fn drop(&mut self) {
        self.cancel();
    }
}





// ----- Example usage ------

rpc_message!{
    REQ = Request,
    RES = Response,
    fn GetTime => Instant,
    fn SetTarget(f32) => ACK,
    fn GetTarget => f32,
}

#[derive(PartialEq)]
pub enum ACK {
    Ack,
    Deny,
}

static PROC: Procedure<Request, Response> = Procedure::new();

async fn handler() {

    let mut altitude = 0.0;

    loop {
        match PROC.get_request().await {
            Request::GetTime(responder) => {
                responder.respond(Instant::now());
            },
            Request::SetTarget(responder) => {
                if responder.request.0 > 2.0 {
                    altitude = responder.request.0;
                    responder.respond(ACK::Ack);
                } else {
                    altitude = responder.request.0;
                    responder.respond(ACK::Deny)
                }
            },
            Request::GetTarget(responder) => {
                responder.respond(altitude);
            },
        }
    }
}


#[cfg(test)]
mod tests {

    use futures::task::SpawnExt;

    use super::*;

    #[futures_test::test]
    async fn test() {
        static PROCEDURE: Procedure<Request, Response> = Procedure::new();

        let spawner = futures_test::task::noop_spawner_mut();



        spawner.spawn( async {

            let target = PROCEDURE.request(requests::GetTarget).await.unwrap();

            assert_eq!(target,1. );

            if target > 100. {
                let res = PROCEDURE.request(requests::SetTarget(10.)).await.unwrap();

                if res == ACK::Ack {
                    let target = PROCEDURE.request(requests::GetTarget).await.unwrap();
                    assert_eq!(target,10. );
                }
            }
        });




        spawner.spawn(async {

            let mut target = 1.0;

            loop {
                match PROCEDURE.get_request().await {
                    Request::GetTime(_) => unimplemented!(),
                    Request::SetTarget(handle) => {
                        if (-100. .. 100.).contains(&handle.request.0) {
                            target = handle.request.0;
                            handle.respond(ACK::Ack);
                        } else {
                            handle.respond(ACK::Deny);
                        }
                    },
                    Request::GetTarget(handle) => {
                        handle.respond(target);
                    },
                }
            }
        });
    }
}
