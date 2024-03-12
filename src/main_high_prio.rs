use crate::drivers::rp2040::dshot_pio::DshotPio;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C0;

use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, UartRx};

use embassy_rp::peripherals::PIO0;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

type I2cInnerAsyncType = I2c<'static, I2C0, i2c::Async>;
static I2C_ASYNC: StaticCell<Mutex<ThreadModeRawMutex, I2cInnerAsyncType>> = StaticCell::new();
pub type I2cAsyncType = I2cDevice<'static, ThreadModeRawMutex, I2cInnerAsyncType>;

#[embassy_executor::task]
pub async fn main_high_prio(
    i2c_async: I2c<'static, I2C0, i2c::Async>,
    uart_rx_sbus: UartRx<'static, UART1, uart::Async>,
    driver_dshot_pio: DshotPio<'static, 4, PIO0>,
) {
    defmt::info!("[MAIN_CORE0]: Starting main task");

    let spawner =  embassy_executor::Spawner::for_current_executor().await;

    let i2c_async_mutex = I2C_ASYNC.init(Mutex::new(i2c_async));

    // Start SBUS reader task
    spawner.must_spawn(crate::t_sbus_reader::sbus_reader(
        uart_rx_sbus,
    ));

    // Setup IMU governor task, responsible for selecting the active IMU
    spawner.must_spawn(crate::t_imu_governor::imu_governor());

    // Setup MAG governor task, responsible for selecting the active MAG
    spawner.must_spawn(crate::t_mag_governor::mag_governor());

    // Setup IMU + MAG (icm20948) driver task
    spawner.must_spawn(crate::t_icm20948_driver::icm_20948_driver(
        I2cDevice::new(i2c_async_mutex),
        0,
        0,
    ));

    // Start the estimator, control loop and motor mixing
    spawner.must_spawn(crate::t_attitude_estimator::attitude_estimator());
    spawner.must_spawn(crate::t_attitude_controller::attitude_controller());
    spawner.must_spawn(crate::t_motor_mixing::motor_mixing());

    spawner.must_spawn(crate::t_commander::commander());

    // Start the motor governor task
    spawner.must_spawn(crate::t_motor_governor::motor_governor(
        driver_dshot_pio,
    ));

    // Start the transmitter governor task
    spawner.must_spawn(crate::t_rc_mapper::rc_mapper());

    defmt::info!("[MAIN_CORE0]: All tasks started");
}
