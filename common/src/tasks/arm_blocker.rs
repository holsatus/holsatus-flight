use core::f32::consts::FRAC_1_SQRT_2;

use crate::{multi_receiver, signals as s, tasks::calibrator::CalibratorState, types::status::ArmingBlocker, NUM_IMU};
use embassy_futures::select::select;
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::Vector3;

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "arm_blocker";
    info!("{}: Task started", ID);

    // Input channels
    let mut rcv_attitude_euler = s::AHRS_ATTITUDE.receiver().unwrap();
    let mut rcv_rc_status = s::RC_STATUS.receiver().unwrap();
    let mut rcv_arm_motors = s::CMD_ARM_MOTORS.receiver().unwrap();
    let mut rcv_calibrator_state = s::CALIBRATOR_STATE.receiver().unwrap();
    let mut rcv_gyr_cal = multi_receiver!(s::CFG_MULTI_GYR_CAL, NUM_IMU);
    let mut _rcv_acc_cal = multi_receiver!(s::CFG_MULTI_ACC_CAL, NUM_IMU);
    let mut rcv_rc_controls = s::RC_ANALOG_RATE.receiver().unwrap();
    let mut _rcv_usb_connected = s::USB_CONNECTED.receiver().unwrap();
    // let mut rcv_loop_health = s::LOOP_HEALTH.receiver().unwrap();
    // let mut rcv_landed_state = s::LANDED_STATE.receiver().unwrap();

    // Output channels
    let snd_arm_blocker = s::ARMING_BLOCKER.sender();

    // Initialize the arm blocker flag as all high
    let mut local_arm_blocker_flag = ArmingBlocker::all() & 
        !(ArmingBlocker::SYSTEM_LOAD | ArmingBlocker::NO_ACC_DATA | ArmingBlocker::NO_GYR_DATA | ArmingBlocker::RESERVED | ArmingBlocker::NO_ACC_CALIB | ArmingBlocker::HIGH_ATTITUDE | ArmingBlocker::USB_CONNECTED);

    // Set flag to the initial value
    snd_arm_blocker.send(local_arm_blocker_flag);

    // Theoretically this task could run only whenever an arming command is received,
    // but for state visibility and debugging, we run it at a low fixed frequency.
    let mut ticker = Ticker::every(Duration::from_hz(1));

    '_infinite: loop {
        // Wait for the vehicle to be on the ground for these checks. Saves computation
        // rcv_landed_state
        //     .get_and(|x| *x == LandedState::OnGround)
        //     .await;

        // Await arming command (or ticker, not explicitly needed)
        _ = select(rcv_arm_motors.changed(), ticker.next()).await;
        local_arm_blocker_flag = snd_arm_blocker.try_get().unwrap_or(local_arm_blocker_flag);

        // Check if the drone is in a high attitude ~ atan(sqrt(tan(pitch)^2 + tan(roll)^2))
        if let Some(attitude) = rcv_attitude_euler.try_changed() {
            let high_attitude = Vector3::from(attitude).xy().norm() > FRAC_1_SQRT_2;
            local_arm_blocker_flag.set(ArmingBlocker::HIGH_ATTITUDE, high_attitude);
        }
        
        // Check if the attitude commands are too high
        if let Some(controls) = rcv_rc_controls.try_changed() {
            let controls_vec = Vector3::from(controls.roll_pitch_yaw());
            local_arm_blocker_flag.set(ArmingBlocker::HIGH_THROTTLE_CMD, controls.throttle() > 0.1);
            local_arm_blocker_flag.set(ArmingBlocker::HIGH_ATTITUDE_CMD, controls_vec.norm() > 0.1);
        }

        // Check that transmitter is not in failsafe
        if let Some(rc_status) = rcv_rc_status.try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::RX_FAILSAFE, rc_status.failsafe);
        }

        // Check if the active accelerometer is calibrated
        // if let Some(acc_cal) = rcv_acc_cal[0].try_changed() {
        //     local_arm_blocker_flag.set(ArmingBlocker::NO_ACC_CALIB, !acc_cal.has_bias());
        // }

        // Check if the active gyroscpe is calibrated
        if let Some(gyr_cal) = rcv_gyr_cal[0].try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::NO_GYR_CALIB, !gyr_cal.has_bias());
        }

        /*

        // Check if the loop frequency health is too low
        if let Some(loop_health) = rcv_loop_health.try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::SYSTEM_LOAD, loop_health < 0.98);
        }
        
        // Check if the active IMU is currently assigned to a sensor
        if let Some(imu_redundancy) = rcv_imu_active_id.try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::NO_GYR_DATA, imu_redundancy.is_none());
            local_arm_blocker_flag.set(ArmingBlocker::NO_ACC_DATA, imu_redundancy.is_none());
        }

        */


        // Check if a sensor is currently calibrating
        if let Some(calibrating) = rcv_calibrator_state.try_changed() {
            let is_calibrating = matches!(calibrating, CalibratorState::Calibrating(_));
            local_arm_blocker_flag.set(ArmingBlocker::CALIBRATING, is_calibrating);
        }

        // // Check if the USB is connected
        // if let Some(usb_connected) = rcv_usb_connected.try_changed() {
        //     local_arm_blocker_flag.set(ArmingBlocker::USB_CONNECTED, usb_connected);
        // }

        // Check if the boot grace period has passed
        let boot_grace = Instant::MIN.elapsed() < Duration::from_secs(5);
        local_arm_blocker_flag.set(ArmingBlocker::BOOT_GRACE, boot_grace);

        // #[cfg(feature = "defmt")]
        // info!("{:?}", defmt::Debug2Format(&local_arm_blocker_flag));
        // #[cfg(not(feature = "defmt"))]
        // info!("{:?}", local_arm_blocker_flag);
        // info!("motors {:?}", s::MOTORS_STATE.try_get());

        // Transmit changes to the arm blocker flag
        snd_arm_blocker.send(local_arm_blocker_flag);
    }
}
