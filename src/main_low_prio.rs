use embassy_executor::Spawner;

use embassy_rp::peripherals::USB;
#[cfg(feature = "mavlink")]
use embassy_rp::{
    gpio::AnyPin,
    peripherals::UART0,
    uart::{Async, Uart},
};


#[embassy_executor::task]
pub async fn main_low_prio(
    spawner: Spawner,
    #[cfg(feature = "mavlink")]
    led_pin: AnyPin,
    #[cfg(feature = "mavlink")]
    uart_mavlink: Uart<'static, UART0, Async>,

    #[cfg(feature = "shell")] // and rp2040
    usb: USB,
) {
    defmt::info!("[MAIN_CORE1]: Starting main task");

    // Start the mavlink server (RX and TX)
    #[cfg(feature = "mavlink")]
    crate::mavlink::mavlink_uart_companion_server(spawner, uart_mavlink, led_pin).await;

    // Start the task which sets the arming blocker flag
    spawner.must_spawn(crate::t_arm_checker::arm_checker());

    // Start the status printer task
    spawner.must_spawn(crate::t_status_printer::status_printer());

    // Start calibration routine tasks
    spawner.must_spawn(crate::t_gyr_calibration::gyr_calibration());
    spawner.must_spawn(crate::t_acc_calibration::acc_calibration());

    spawner.must_spawn(crate::t_flight_detector::flight_detector());

    #[cfg(feature = "shell")]
    spawner.must_spawn(crate::shell::holsatus_shell(usb));

    defmt::info!("[MAIN_CORE0]: All tasks started");
}
