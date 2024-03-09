use embassy_executor::Spawner;
use embassy_rp::{
    gpio::AnyPin,
    peripherals::UART0,
    uart::{Async, Uart},
};

use crate::config::Configuration;

// Import task modules

#[embassy_executor::task]
pub async fn main_core1(
    spawner: Spawner,
    led_pin: AnyPin,
    uart_mavlink: Uart<'static, UART0, Async>,
    config: &'static Configuration,
) {
    defmt::info!("[MAIN_CORE1]: Starting main task");

    // Start the mavlink server (RX and TX)
    crate::mavlink::mavlink_uart_companion_server(spawner, uart_mavlink, led_pin, config);

    // Start the task which sets the arming blocker flag
    spawner.must_spawn(crate::t_arm_checker::arm_checker());

    // Start the status printer task
    spawner.must_spawn(crate::t_status_printer::status_printer());

    defmt::info!("[MAIN_CORE0]: All tasks started");
}
