use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex}, waitqueue::MultiWakerRegistration};
use heapless::Deque;
use portable_atomic::AtomicBool;

use super::reqres::{self, ImmediateError, ReqResError, ReqResSignal};

struct Request {

}

struct Response {
    
}

const NEW_REQRES: ReqResSignal<CriticalSectionRawMutex, Request, Response> = ReqResSignal::new();
static MULTI_REQRES: [ReqResSignal<CriticalSectionRawMutex, Request, Response>; 4] = [NEW_REQRES; 4];

struct QueuedReqRes<const N: usize, M: RawMutex, T, R> {
    multi_waker: MultiWakerRegistration<N>,
    signal: ReqResSignal<M, T, R>,
}

impl <const N: usize, M: RawMutex, T, R> QueuedReqRes<N, M, T, R> {

    pub async fn make_request(&self, req: T) -> Result<R, ReqResError> {
        let mut req = Some(req);

        for signal in &self.signal {
            match signal.requester().request_immediate(req.take().unwrap()).await {
                Err(ImmediateError::SignalOccupied(returned)) => req = Some(returned),
                Err(ImmediateError::Other(err)) => return Err(err),
                Ok(ok_result) => return Ok(ok_result),
            }
        }

        Err(ReqResError::InvalidState)
    }
}
