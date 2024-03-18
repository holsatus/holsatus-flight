#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use panic_probe as _;
use embassy_futures as _;
pub use holsatus_flight::board::bsp;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {

    // Initialize all peripherals
    let peripherals = bsp::init_peripherals();
    
    // Two second delay (to prevent CsSpinLock deadlock on startup)
    defmt::info!("Starting in 2 seconds");
    Timer::after_secs(2).await;


    let i2c = holsatus_flight::I2C_ASYNC.init(Mutex::new(peripherals.async_i2c));

    spawner.must_spawn(holsatus_flight::t_librarian::librarian(peripherals.cfg_flash));
    spawner.must_spawn(holsatus_flight::t_blackbox_logging::black_box());

    // Start SBUS reader task
    spawner.must_spawn(holsatus_flight::t_sbus_reader::sbus_reader(peripherals.uart_sbus));

    // Setup IMU governor task, responsible for selecting the active IMU
    spawner.must_spawn(holsatus_flight::t_imu_governor::imu_governor());

    // Setup MAG governor task, responsible for selecting the active MAG
    spawner.must_spawn(holsatus_flight::t_mag_governor::mag_governor());

    // Setup IMU + MAG (icm20948) driver task
    spawner.must_spawn(holsatus_flight::t_icm20948_driver::icm_20948_driver(I2cDevice::new(i2c), 0, 0));

    // Start the attitude estimator, control loop and motor mixing
    spawner.must_spawn(holsatus_flight::t_attitude_estimator::attitude_estimator());
    spawner.must_spawn(holsatus_flight::t_attitude_controller::attitude_controller());
    spawner.must_spawn(holsatus_flight::t_motor_mixing::motor_mixing());

    spawner.must_spawn(holsatus_flight::t_commander::commander());

    // Start the motor governor task
    spawner.must_spawn(holsatus_flight::t_motor_governor::motor_governor(peripherals.motor_driver));

    // Start the transmitter governor task
    spawner.must_spawn(holsatus_flight::t_rc_mapper::rc_mapper());

    // Start the mavlink server (RX and TX)
    #[cfg(feature = "mavlink")]
    spawner.must_spawn(holsatus_flight::mavlink::mavlink_server(peripherals.uart_mav, embassy_rp::gpio::Pin::degrade(p.PIN_25)));

    // Start the task which sets the arming blocker flag
    spawner.must_spawn(holsatus_flight::t_arm_checker::arm_checker());

    // Start the status printer task
    // spawner.must_spawn(crate::t_status_printer::status_printer());

    // Start calibration routine tasks
    spawner.must_spawn(holsatus_flight::t_gyr_calibration::gyr_calibration());
    spawner.must_spawn(holsatus_flight::t_acc_calibration::acc_calibration());

    spawner.must_spawn(holsatus_flight::t_flight_detector::flight_detector());

    #[cfg(feature = "shell")]
    spawner.must_spawn(holsatus_flight::shell::holsatus_shell(peripherals.usb));


}
