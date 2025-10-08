use core::{future::pending, sync::atomic::Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embedded_io_async::{Read, Write};
use portable_atomic::AtomicU8;

use super::serial_dummy::*;
use crate::sync::channel::Channel;

pub const NUM_INSTANCES: usize = 2;

enum Message {
    UseSerialRx(Option<DummySerialRx>),
    UseSerialTx(Option<DummySerialTx>),
}

pub struct InstanceHandle {
    channel: &'static Channel<Message, 1>,
}

pub struct Instance {
    id: u8,
    sequence: u8,
    serial_rx: Option<DummySerialRx>,
    serial_tx: Option<DummySerialTx>,
    channel: &'static Channel<Message, 1>,
}

impl Instance {
    pub fn new(id: u8) -> Instance {
        static CHANNEL: Channel<Message, 1> = Channel::new();

        Instance {
            id,
            sequence: 0,
            serial_rx: None,
            serial_tx: None,
            channel: &CHANNEL,
        }
    }

    pub fn handle(&self) -> InstanceHandle {
        InstanceHandle {
            channel: self.channel,
        }
    }
}

pub fn initialize_instance(spawner: Spawner, id: u8) -> InstanceHandle {
    let instance = Instance::new(id);
    let handle = instance.handle();

    spawner.must_spawn(instance_receiver(instance));

    handle
}

use core::future::Future;

trait OrPending {
    type Fut: core::future::Future;
    fn unwrap_or_never(self) -> impl Future<Output = <Self::Fut as Future>::Output>;
}

impl<T: core::future::Future> OrPending for Option<T> {
    type Fut = T;

    async fn unwrap_or_never(self) -> T::Output {
        match self {
            Some(fut) => fut.await,
            None => pending::<T::Output>().await,
        }
    }
}

#[embassy_executor::task(pool_size = NUM_INSTANCES)]
pub async fn instance_receiver(mut instance: Instance) -> ! {
    trace!("Starting MavLink instance");

    let mut buffer = [0u8; 255];
    let mut write_buf = [0u8; 255];

    loop {
        match select(
            instance.channel.receive(),
            instance
                .serial_rx
                .as_mut()
                .map(|rx| rx.read(buffer.as_mut()))
                .unwrap_or_never(),
        )
        .await
        {
            Either::First(msg) => match msg {
                Message::UseSerialRx(rx) => instance.serial_rx = rx,
                Message::UseSerialTx(tx) => instance.serial_tx = tx,
            },
            Either::Second(Ok(n)) => {
                let bytes = &buffer[..n];
            }
            Either::Second(Err(error)) => {}
        }
    }
}

#[embassy_executor::task(pool_size = NUM_INSTANCES)]
pub async fn instance_handler(mut instance: Instance) -> ! {
    trace!("Starting MavLink instance");

    let mut buffer = [0u8; 255];

    loop {
        match select(
            instance.channel.receive(),
            instance
                .serial_rx
                .as_mut()
                .map(|rx| rx.read(buffer.as_mut()))
                .unwrap_or_never(),
        )
        .await
        {
            Either::First(msg) => match msg {
                Message::UseSerialRx(rx) => instance.serial_rx = rx,
                Message::UseSerialTx(tx) => instance.serial_tx = tx,
            },
            Either::Second(Ok(n)) => {
                let bytes = &buffer[..n];
            }
            Either::Second(Err(error)) => {}
        }
    }
}
