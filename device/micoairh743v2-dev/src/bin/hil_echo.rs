#![no_std]
#![no_main]

use embassy_stm32::usart::{Config as UartConfig, Uart};
use micoairh743v2::resources::{self, UartLogIrqs, UartLogResources};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn uart_echo_task(r: UartLogResources) {
    let (mut tx, mut rx) = Uart::new(
        r.usart,
        r.rx,
        r.tx,
        r.dma_rx,
        r.dma,
        UartLogIrqs,
        UartConfig::default(),
    )
    .unwrap()
    .split();
    let mut buf = [0u8; 32];
    loop {
        rx.read(&mut buf).await.ok();
        tx.write(&buf).await.ok();
    }
}

#[embassy_executor::main]
async fn main(thread_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let r = resources::split(p);
    thread_spawner.spawn(uart_echo_task(r.uart_log).unwrap());
}
