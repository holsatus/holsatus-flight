use crate::common::types::{ArmedState, DisarmReason, MotorState};
use crate::drivers::rp2040::dshot_pio::DshotPioTrait; // TODO Get rid of this trait!
use embassy_futures::select::{select, Either};
use embassy_time::{with_timeout, Timer};

pub const TASK_ID: &str = "[MOTOR GOVERNOR]";

bitflags::bitflags! {
    /// This bitflag represents the possible reasons why the vehicle cannot be armed.
    /// The bitflag is supposed to be `0x0000` when the vehicle is ready to be armed,
    /// which can be checked with the `is_empty()` method.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ArmBlocker: u16 {

        /// **Bit 0** - The gyroscope is not calibrated.
        const NO_GYR_CALIB  = 1 << 0;

        /// **Bit 1** - The accelerometer is not calibrated.
        const NO_ACC_CALIB  = 1 << 1;

        /// **Bit 2** - The vehicle is currently undergoing sensor calibration.
        const GYR_CALIBIN   = 1 << 2;

        /// **Bit 3** - The vehicle is currently undergoing sensor calibration.
        const ACC_CALIBIN   = 1 << 3;

        /// **Bit 4** - No gyroscope data available.
        const NO_GYR_DATA   = 1 << 4;

        /// **Bit 5** - No accelerometer data available.
        const NO_ACC_DATA   = 1 << 5;

        /// **Bit 6** - The vehicle is at a higher throttle than allowed.
        const HIGH_THROTTLE_CMD = 1 << 6;

        /// **Bit 7** - The vehicle is at a higher attitude than allowed.
        const HIGH_ATTITUDE_CMD = 1 << 7;

        /// **Bit 8** - The vehicle attitude is at an abnormally high angle.
        const HIGH_ATTITUDE = 1 << 8;

        /// **Bit 9** - The vehicle was armed too soon after boot.
        const BOOT_GRACE    = 1 << 9;

        /// **Bit 10** - The system load it too high (low loop frequency)
        const SYSTEM_LOAD   = 1 << 10;

        /// **Bit 11** - The receiver is in failsafe mode.
        const RX_FAILSAFE   = 1 << 11;

        /// **Bit 12** - The flight controller has established a USB connection.
        const USB_CONNECTED = 1 << 12;

        /// **Bit 13** - The `CMD_DISARM` flag must be raised once before arming can continue.
        const MUST_DISARM    = 1 << 13;

        /// **Bit 14** - The vehicle is commanded to be disarmed.
        /// This is the only bitflag which can be directly set by,
        /// the user, and which can disarm the vehicle at any time.
        const CMD_DISARM    = 1 << 14;
    }
}

use crate::messaging as msg;

/// Task to govern the arming, disarming and setting the speed of the motors.
/// Arming takes 3.5 seconds: 3.0 s to arm, 0.5 s to set direction
#[embassy_executor::task]
pub async fn motor_governor(
    mut driver_dshot: crate::bsp::MotorDriverPeripheral,
) -> ! {

    // Input messages
    let mut rcv_motor_speed = msg::MOTOR_SPEEDS.receiver().unwrap();
    let mut rcv_arming_prevention = msg::ARM_BLOCKER.receiver().unwrap();
    let mut rcv_arm_vehicle = msg::CMD_ARM_VEHICLE.receiver().unwrap();

    // Output messages
    let snd_motor_state = msg::MOTOR_STATE.sender();

    // Send initial disarmed state
    snd_motor_state.send(MotorState::Disarmed(DisarmReason::NotInitialized));

    #[allow(unused_labels)]
    'infinite: loop {
        // Wait for arming prevention flag to be completely empty
        rcv_arm_vehicle.changed_and(|arm|arm == &true).await;

        // Notify that motors are arming
        snd_motor_state.send(MotorState::Arming);

        // Send minimum throttle for a few seconds to arm the ESCs
        defmt::info!("{}: Initializing motors", TASK_ID);
        Timer::after_millis(500).await;
        for _i in 0..50 {
            driver_dshot.throttle_minimum();
            Timer::after_millis(50).await;
        }

        // Set motor directions for the four motors
        defmt::info!("{}: Setting motor directions", TASK_ID);
        for _i in 0..10 {
            driver_dshot.reverse(msg::CFG_REVERSE_MOTOR.spin_get().await);
            Timer::after_millis(50).await;
        }

        // After arming, ensure (again) no arming prevention flags are set
        if !rcv_arming_prevention.get().await.is_empty() {
            defmt::warn!("{}: Disarming motors -> arming prevention", TASK_ID);
            snd_motor_state.send(MotorState::Disarmed(DisarmReason::Fault));
            continue 'infinite;
        }

        let dshot_timeout = msg::CFG_DSHOT_TIMEOUT.spin_get().await;

        defmt::info!("{}: Motors are armed and active", TASK_ID);
        'armed: loop {
            match with_timeout(
                dshot_timeout,
                select(
                    rcv_arm_vehicle.changed_and(|arm|arm == &false),
                    rcv_motor_speed.changed(),
                )
            ).await {

                // Motors are commanded to disarm
                Ok(Either::First(_)) => {
                    driver_dshot.throttle_minimum();
                    snd_motor_state.send(MotorState::Disarmed(DisarmReason::Commanded));
                    defmt::warn!("{}: Disarming motors -> commanded", TASK_ID);
                    break 'armed;
                },

                // Motors are set to idle (armed, not spinning)
                Ok(Either::Second([0,0,0,0])) => {
                    driver_dshot.throttle_minimum();
                    snd_motor_state.send(MotorState::Armed(ArmedState::Idle));
                },

                // Motor speed message received correctly
                Ok(Either::Second(speeds)) => {
                    driver_dshot.throttle_clamp(speeds);
                    snd_motor_state.send(MotorState::Armed(ArmedState::Running(speeds)));
                },

                // Automatic disarm due to message timeout
                Err(_) => {
                    driver_dshot.throttle_minimum();
                    snd_motor_state.send(MotorState::Disarmed(DisarmReason::Timeout));
                    defmt:: warn!("{}: Disarming motors -> timeout", TASK_ID);
                    break 'armed;
                },
            }
        }
    }
}
