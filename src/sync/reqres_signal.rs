//! A synchronization primitive which enables request-response style communication.
//! One task, makes a request, and awaits a response. Another task can receive the
//! request, evaluate it and form a response, and respond to the request.
//! The requester is simply awaiting the request the entire time.

use core::cell::Cell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Poll, Waker};

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::blocking_mutex::Mutex;

use defmt::Format;
use portable_atomic::AtomicU64;

pub struct ReqResSignal<M, T, R>
where
    M: RawMutex,
{
    state: Mutex<M, Cell<State<T, R>>>,
    msg_id: AtomicU64,
    res_taken: AtomicBool,
}

enum State<T, R> {
    Empty,
    RequestAndRequesterWaiting(u64, T, Waker),
    RequesterWaiting(u64, Waker),
    Response(u64, R),
    ResponderWaiting(Waker),
    ResponseAndResponderWaiting(u64, R, Waker),
}

pub struct Requester<'a, M, T, R>
where
    M: RawMutex,
{
    channel: &'a ReqResSignal<M, T, R>,
}

pub struct Responder<'a, M, T, R>
where
    M: RawMutex,
{
    channel: &'a ReqResSignal<M, T, R>,
}

/// This type is a handle that allows for responding to a specific request.
/// Responding to the request consumes this handle and notifies the requster
/// that a response is ready. If this handle is dropped without responding,
/// the requester is also awoken, but without 
pub struct ResponderHandle<'a, M, T, R>
where
    M: RawMutex,
{
    channel: &'a ReqResSignal<M, T, R>,
    msg_id: u64,
}

impl<'a, M, T, R> Drop for Responder<'a, M, T, R>
where
    M: RawMutex,
{
    fn drop(&mut self) {
        self.channel.res_taken.store(false, Ordering::Relaxed);
    }
}

impl<'a, M, T, R> Drop for ResponderHandle<'a, M, T, R>
where
    M: RawMutex,
{
    fn drop(&mut self) {
        self.channel.state.lock(|state| {

            match state.replace(State::Empty) {
                // If requester is waiting and there was no
                // response before drop, wake up the requester.
                State::RequestAndRequesterWaiting(_, _, w) => {
                    w.wake()
                },
                State::RequesterWaiting(_, w) => {
                    w.wake()
                },

                // Otherwise leave state as is
                prev_state => {
                    state.set(prev_state)
                },
            }
        });
    }
}

impl<M: RawMutex, T, R> ReqResSignal<M, T, R> {
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(Cell::new(State::Empty)),
            msg_id: AtomicU64::new(0),
            res_taken: AtomicBool::new(false),
        }
    }

    pub fn requester(&self) -> Requester<M, T, R> {
        Requester { channel: self }
    }

    pub fn responder(&self) -> Option<Responder<M, T, R>> {
        if !self.res_taken.load(Ordering::Relaxed) {
            self.res_taken.store(true, Ordering::Relaxed);
            Some(Responder { channel: self })
        } else {
            None
        }
    }
}

#[derive(Format, Debug)]
pub enum ReqResError {
    ResponderDropped,
    RequesterDropped,
    ChannelOccupied,
}

impl<'a, M: RawMutex, T, R> Requester<'a, M, T, R> {

    /// Submit a request and wait for the response. This is fallible because the responder might
    /// have dropped the future or the ResponderHandle, meaning no response will ever happen for
    /// the given request. 
    /// 
    /// ## Note
    /// It is technically possible, if multiple tasks are trying to make requests in a near busy-loop, that
    /// some tasks can starve some others. The solution is to ensure there are *at least* a few ticks of async
    /// delay in the near busy-loops making the requests, using e.g. `Timer` or `Ticker` from `embassy`.
    pub async fn request(&mut self, req: T) -> Result<R, ReqResError> {
        let mut opt_req = Some(req);

        // Increase the message id and store it in a local variable
        let req_id = self.channel.msg_id.fetch_add(1, Ordering::Relaxed);
        poll_fn(|cx|{
            self.channel.state.lock(|state| {
                let old_state = state.replace(State::Empty);
                match old_state {
                    State::Empty => {
                        // This state is indicative that the responder dropped, either
                        // by dropping the ResponderHandle, or by dropping the future.

                        // If we have not yet submitted the request, we submit it with our waker
                        if let Some(req) = opt_req.take() {
                            state.set(State::RequestAndRequesterWaiting(req_id, req, cx.waker().clone()));
                            Poll::Pending

                        // If have submitted our response and the state is None, this indicates that
                        // the responder dropped the future or ResponderHandle. We return an error.
                        } else {
                            Poll::Ready(Err(ReqResError::ResponderDropped))
                        }
                    }
                    State::RequestAndRequesterWaiting(id, req, w) => {
                        state.set(State::RequestAndRequesterWaiting(id, req, w));
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    },
                    State::RequesterWaiting(id, w) => {
                        state.set(State::RequesterWaiting(id, w));
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    },
                    State::Response(id, res) => {

                        // If we have delivered the request, this is our response
                        if opt_req.is_none() && id == req_id {
                            state.set(State::Empty);
                            Poll::Ready(Ok(res))

                        // If we have not delivered the request, we must wait for the reponse to be taken
                        } else {
                            state.set(State::Response(id, res));
                            cx.waker().wake_by_ref();
                            Poll::Pending
                        }

                    },
                    State::ResponderWaiting(w) => {

                        // If we have not yet sent the request, we send it and wait for the response
                        if let Some(req) = opt_req.take() {
                            state.set(State::RequestAndRequesterWaiting(req_id, req, cx.waker().clone()));
                            w.wake();
                            Poll::Pending

                        // If we already sent the request, the responder must have dropped
                        } else {
                            state.set(State::ResponderWaiting(w));
                            return Poll::Ready(Err(ReqResError::ResponderDropped))
                        }
                    },
                    State::ResponseAndResponderWaiting(id,  res, w) => {

                        // If we have delivered the request, this is our response
                        if opt_req.is_none() && id == req_id {
                            state.set(State::ResponderWaiting(w));
                            Poll::Ready(Ok(res))

                        // If we have not delivered the request, we must wait for the reponse to be taken
                        } else {
                            state.set(State::ResponseAndResponderWaiting(id, res, w));
                            cx.waker().wake_by_ref();
                            Poll::Pending
                        }                        
                    },
                }
            })
        }).await
    }
}

impl<'a, M: RawMutex, T, R> Responder<'a, M, T, R> {

    /// Checks if a request is available in the channel without altering its state.
    pub fn request_available(&self) -> bool {
        self.channel.state.lock(|state| {
            let old_state = state.replace(State::Empty);
            let request_available = match old_state {
                State::RequestAndRequesterWaiting(_, _, _) => true,
                _ => false,
            };
            state.set(old_state);
            request_available
        })
    }

    pub async fn get_request(&self) -> Result<(T, ResponderHandle<M, T, R>), ReqResError> {
        poll_fn(|cx| {
            self.channel.state.lock(|state| {
                let old_state = state.replace(State::Empty);
                match old_state {

                    // If the channel is empty, we register the responder as waiting.
                    State::Empty => {
                        state.set(State::ResponderWaiting(cx.waker().clone()));
                        Poll::Pending
                    }

                    // If a request is available and the requester is waiting we return
                    // the ResponderHandle and change the state to just requester waiting.
                    State::RequestAndRequesterWaiting(id, req, w) => {
                        let handle = ResponderHandle {
                            channel: self.channel,
                            msg_id: id,
                        };
                        state.set(State::RequesterWaiting(id, w));
                        Poll::Ready(Ok((req, handle)))
                    }

                    // Being in this state indicates that the requester is already waiting
                    // for a response. We force that original requester to drop. This prevents
                    // a dead-lock, and the requester can now send a new/retry request.
                    State::RequesterWaiting(_id, w) => {
                        state.set(State::ResponderWaiting(cx.waker().clone()));
                        w.wake();
                        Poll::Pending
                    }

                    // If responder was previously waiting but dropped the future,
                    // it mght be in this state.
                    State::ResponderWaiting(_w) => {
                        state.set(State::ResponderWaiting(cx.waker().clone()));
                        Poll::Pending
                    }

                    // This state represents that there is a previous response in the channel,
                    // which has not yet been received by the requester. We simply register
                    // the current responder as waiting along with the original response.
                    State::Response(id, res) => {
                        state.set(State::ResponseAndResponderWaiting(id, res, cx.waker().clone()));
                        Poll::Pending
                    }

                    // If responder was previously waiting but dropped the future,
                    // we will be in this state. Re-register the current responder as waiting.
                    State::ResponseAndResponderWaiting(id, res, _w) => {
                        state.set(State::ResponseAndResponderWaiting(id, res, cx.waker().clone()));
                        Poll::Pending
                    },
                }
            })
        })
        .await
    }

    pub fn try_get_request(&self) -> Option<(T, ResponderHandle<M, T, R>)> {
        self.channel.state.lock(|state| {
            let old_state = state.replace(State::Empty);
            match old_state {
                State::RequestAndRequesterWaiting(id, req, w) => {
                    let handle = ResponderHandle {
                        channel: self.channel,
                        msg_id: id,
                    };
                    state.set(State::RequesterWaiting(id, w));
                    Some((req, handle))
                }

                // Otherwise, leave the state as it was
                state_otherwise => {
                    state.set(state_otherwise);
                    None
                }
            }
        })
    }
}

impl<'a, M: RawMutex, T, R> ResponderHandle<'a, M, T, R> {
    pub fn respond(self, res: R) -> Result<(), ReqResError> {
        self.channel.state.lock(|state| {
            let old_state = state.replace(State::Empty);
            match old_state {
                State::Empty => {
                    Err(ReqResError::RequesterDropped)
                }

                // If the original request was dropped and a new one is available,
                // we are in this state. We essentially just ignore the current
                // response and set the state back to what it just was. 
                State::RequestAndRequesterWaiting(req, id, w) => {
                    state.set(State::RequestAndRequesterWaiting(req, id, w));
                    Err(ReqResError::RequesterDropped)
                }

                // This is the only "happy path" state. If the ID of our respose
                // matches the ID of the request, we set the state to be the response.
                // Otherwise assume the requester has dropped and leave state unchanged.
                State::RequesterWaiting(id, w) => {
                    if self.msg_id == id {
                        state.set(State::Response(id, res));
                        w.wake();
                        Ok(())
                    } else {
                        state.set(State::RequesterWaiting(id, w));
                        Err(ReqResError::RequesterDropped)
                    }
                }

                // If the the responder registered itself as waiting, after receiving
                // this handle, we are in this state. We simply ignore the current
                // response and set the state back to what it just was.
                State::ResponderWaiting(w) => {
                    w.wake();
                    Err(ReqResError::ResponderDropped)
                }

                // These states should in theory be unreachable,since there can only
                // be one responder instance, meaning a response cannot exist, while
                // we are holding a ResponderHandle. But we handle them as errors just in case.
                State::Response(_res, _id) => {
                    defmt::debug_assert!(false, "This state should never occur. It is a bug.");
                    Err(ReqResError::ChannelOccupied)
                }
                State::ResponseAndResponderWaiting(_res, _id, w) => {
                    defmt::debug_assert!(false, "This state should never occur. It is a bug.");
                    w.wake();
                    Err(ReqResError::ChannelOccupied)
                },
            }
        })
    }
}
