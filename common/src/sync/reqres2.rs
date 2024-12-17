use core::{cell::{Cell, RefCell}, marker::PhantomData, task::Waker};

use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

struct ResponseHandle<RES> {
    res: Mutex<CriticalSectionRawMutex, Cell<Option<RES>>>,
    waker: Mutex<CriticalSectionRawMutex, Cell<Option<Waker>>>,
}

impl<RES> ResponseHandle<RES> {
    fn new() -> Self {
        Self {
            res: Mutex::new(Cell::new(None)),
            waker: Mutex::new(Cell::new(None))
        }
    }

    fn respond(&self, res: RES) {
        self.res.lock(|cell|{
            cell.set(Some(res));
        })
    }
}
struct Request<REQ, RES> {
    request: REQ,
    response: ResponseHandle<RES>
}

impl <REQ, RES> From<REQ> for Request<REQ, RES> {
    fn from(request: REQ) -> Self {
        Request {
            request,
            response: ResponseHandle::new()
        }
    }
}

macro_rules! req_res {
    (
        $(#[$meta:meta])*
        NAME = $msg_name:ident
        $( $req_name:ident : $req:ty => $res:ty $(,)?)+
    ) => {
        $(#[$meta])*
        enum $msg_name {
            $(
                $req_name(Request<$req, $res>),
            )+
        }
    };
}

req_res! {
    #[allow(unused)]
    NAME = Message
    GetCounter: () => u32,
    GetSquared: u8 => u16,
}

struct RequestState<M> {
    message: Option<M>,
    req_waker: Option<Waker>,
    res_waker: Option<Waker>,
}

struct ReqResChannel<M> {
    inner: Mutex<CriticalSectionRawMutex, RefCell<RequestState<M>>>
}

impl ReqResChannel<Message> {
    async fn req_get_counter<'a>(&'a self, message: ()) -> RequesterHandle<'a, (), u32> {
        self.inner.lock(|inner| {
            let mut state = inner.borrow_mut();
            state.message = Some(Message::GetCounter(message.into()));
            
            if let Some(waker) = state.res_waker.take() {
                waker.wake()
            }

            RequesterHandle {
                ch: self,
                _p: PhantomData
            }
        })
    }
}

struct RequesterHandle<'a, REQ, RES> {
    ch: &'a ReqResChannel<Message>,
    _p: PhantomData<(REQ, RES)>
}

async fn _main2() {

    let req: Request<(), u32> = ().into();

    let request = Message::GetCounter(req);



    match request {
        Message::GetCounter(mut handle) => {
            handle.response.respond(0u32)
        },

        Message::GetSquared(mut handle) => {
            handle.response.respond(handle.request as u16 * handle.request as u16)
        },
    }
}

