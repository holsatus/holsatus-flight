use core::sync::atomic::Ordering;
use maitake_sync::{blocking::DefaultMutex, wait_map::WaitMap};
use mutex::{ConstInit, ScopedRawMutex};
use portable_atomic::AtomicUsize;

use super::channel::Channel;

pub struct Procedure<Req, Res, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    index: AtomicUsize,
    chan: Channel<(Req, Option<usize>), N, M>,
    map: WaitMap<usize, Option<Res>, M>,
}

impl<M: ScopedRawMutex + ConstInit, Req, Res, const N: usize> Default
    for Procedure<Req, Res, N, M>
{
    fn default() -> Self {
        Procedure::new()
    }
}

impl<M: ScopedRawMutex, Req, Res, const N: usize> Procedure<Req, Res, N, M> {
    pub const fn new() -> Self
    where
        M: ConstInit,
    {
        Self {
            chan: Channel::new(),
            index: AtomicUsize::new(0),
            map: WaitMap::new_with_raw_mutex(M::INIT),
        }
    }

    /// Receive a request
    pub async fn receive_request(&self) -> (Req, Handle<'_, Res>) {
        let (req, idx) = self.chan.receive().await;
        (
            req,
            Handle {
                map: &self.map,
                idx,
            },
        )
    }

    /// Make a request and await the response.
    ///
    /// If a response is not required, use [`send`] instead.
    pub async fn request(&self, req: Req) -> Option<Res> {
        let idx = self.new_index();

        // Insert outselves in the queue for the ID
        let waiter = self.map.wait(idx);
        let mut waiter = core::pin::pin!(waiter);

        // Subscribing so we are registered prior to sending the request
        waiter.as_mut().subscribe().await.ok()?;

        // Make the request and send it over the channel
        self.chan.send((req, Some(idx))).await;

        // Wait for the response
        waiter.await.ok()?
    }

    /// Send a request without waiting for a response
    ///
    /// If a response is required, use [`request`] instead.
    pub async fn send(&self, req: Req) {
        self.chan.send((req, None)).await;
    }

    fn new_index(&self) -> usize {
        self.index.fetch_add(1, Ordering::Relaxed)
    }
}

/// Trait to do some type erasure of the underlying [`WaitMap`] for the [`Handle`]
trait DynWaitMap<Res> {
    fn wake(&self, key: usize, res: Option<Res>);
}

impl<Res, M: ScopedRawMutex> DynWaitMap<Res> for WaitMap<usize, Option<Res>, M> {
    fn wake(&self, key: usize, res: Option<Res>) {
        _ = self.wake(&key, res)
    }
}

/// Handle to produce a response to a specific request
pub struct Handle<'a, Res> {
    map: &'a dyn DynWaitMap<Res>,
    idx: Option<usize>,
}

impl<'a, Res> Handle<'a, Res> {
    /// Returns true if the sender of this request expects a response.
    ///
    /// This may be checked if the response is expensive to compute.
    pub fn expects_response(&self) -> bool {
        self.idx.is_some()
    }

    /// Respond to the request, consuming this handle
    /// and waking the requestee.
    pub fn respond(self, res: Res) {
        if let Some(index) = self.idx {
            self.map.wake(index, Some(res));
        }
    }

    /// Notify the requestor that the request is closed.
    fn close(&self) {
        if let Some(index) = self.idx {
            self.map.wake(index, None);
        }
    }
}

impl<Res> Drop for Handle<'_, Res> {
    fn drop(&mut self) {
        self.close();
    }
}

// ---- playground ----

#[cfg(all(test, not(feature = "defmt")))]
mod tests {

    use futures::task::SpawnExt;

    use super::*;

    #[derive(Debug)]
    enum Command {
        Task(bool),
        Exit,
    }

    #[derive(Debug, PartialEq)]
    enum Response {
        Accepted,
        Rejected,
    }

    #[test]
    fn test() {
        static PROCEDURE: Procedure<Command, Response, 10> = Procedure::new();
        let mut spawner = futures_executor::LocalPool::new();

        spawner
            .spawner()
            .spawn(async {
                loop {
                    let (req, handle) = PROCEDURE.receive_request().await;

                    match req {
                        Command::Task(foo) => {
                            handle.respond(match foo {
                                true => Response::Accepted,
                                false => Response::Rejected,
                            });
                        }
                        Command::Exit => {
                            handle.respond(Response::Accepted);
                            break;
                        }
                    }
                }
            })
            .unwrap();

        spawner
            .spawner()
            .spawn(async { requester().await.unwrap() })
            .unwrap();

        async fn requester() -> Option<()> {
            let response = PROCEDURE.request(Command::Task(true)).await?;
            assert_eq!(response, Response::Accepted);

            let response = PROCEDURE.request(Command::Task(false)).await?;
            assert_eq!(response, Response::Rejected);

            let response = PROCEDURE.request(Command::Exit).await?;
            assert_eq!(response, Response::Accepted);

            Some(())
        }

        spawner.run();
    }
}
