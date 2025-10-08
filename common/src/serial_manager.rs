use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, pipe::{Pipe, Reader, Writer}};
use embedded_io_async::Read;
use maitake_sync::MutexGuard;
use mutex::{raw_impls::cs::CriticalSectionRawMutex as CSR, BlockingMutex};
use static_cell::StaticCell;

const MAX_SERIAL: usize = 6;


struct Serial {
    reader: SerialReader,
    writer: SerialWriter,
}

struct SerialReader {
    pipe: MutexGuard<'static, &'static Pipe<CriticalSectionRawMutex, 64>, CSR> 
}

struct SerialWriter {
    pipe: MutexGuard<'static, &'static Pipe<CriticalSectionRawMutex, 64>, CSR> 
}

struct SerialReaderGuard {
    pipe: &'static Mutex<CriticalSectionRawMutex, &'static Pipe<CriticalSectionRawMutex, 64>> 
}

struct WriterHandle {
    pipe: &'static Mutex<CriticalSectionRawMutex, &'static Pipe<CriticalSectionRawMutex, 64>> 
}

struct IoReader {
    pipe: &'static Pipe<CriticalSectionRawMutex, 64> 
}

struct IoWriter {
    pipe: &'static Pipe<CriticalSectionRawMutex, 64> 
}

const SERIAL_MANAGER: SerialManager = SerialManager::new();

struct SerialManager {
    inner: BlockingMutex<CSR, ManagerState>
}

struct ManagerState {
    serial: heapless::Vec<&'static Pipe<CriticalSectionRawMutex, 64>, MAX_SERIAL>
}

impl SerialManager {

    pub const fn new() -> SerialManager {
        SerialManager { inner: BlockingMutex::new(ManagerState{
            serial: heapless::Vec::new()
        }) }
    }

    pub fn new_reader() -> (IoReader, WriterHandle) {
        use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as M;

        const N: usize = 64;
        static PIPE: Pipe<M, N> = Pipe::new();

        static WRITER: StaticCell<Mutex<M, &'static Pipe<M, N>>> = StaticCell::new();
        let writer = &*WRITER.init(Mutex::new(&PIPE));

        let writer_handle = WriterHandle {
            pipe: writer,
        };

        let io_reader = IoReader {
            pipe: &PIPE,
        };

        (io_reader, writer_handle)
    }


    pub fn register_serial(&self, serial: &'static Pipe<CriticalSectionRawMutex, 64>) -> bool {
        self.inner.with_lock(|state| {

            // First, ensure we are not already registered
            for each in state.serial.iter() {
                if core::ptr::addr_of!(*each) == core::ptr::addr_of!(serial) {
                    return false;
                }
            }

            state.serial.push(serial).is_ok()
        })
    }
}

struct PortId(u8);
