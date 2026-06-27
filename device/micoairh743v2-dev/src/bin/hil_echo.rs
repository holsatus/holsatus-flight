#![no_std]
#![no_main]

use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals};
use micoairh743v2::resources::{self, UartLogResources};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn uart_echo_task(r: UartLogResources) {
    bind_interrupts!(struct UartIrqs {
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        DMA2_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH7>;
        USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
    });
    let (mut tx, mut rx) = Uart::new(
        r.usart,
        r.rx,
        r.tx,
        r.dma_rx,
        r.dma,
        UartIrqs,
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
