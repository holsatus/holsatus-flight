use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};

use crate::{messaging as msg, t_motor_governor::ArmBlocker};

const INV_SQRT_2: f32 = 0.7071067811865475;

#[embassy_executor::task]
pub async fn arm_checker() -> ! {
    // Input channels
    let mut rcv_attitude_euler = msg::ATTITUDE_EULER.receiver().unwrap();
    let mut rcv_attitude_setpoint = msg::ATTITUDE_SETPOINT.receiver().unwrap();
    let mut rcv_throttle_setpoint = msg::THROTTLE_SETPOINT.receiver().unwrap();
    let mut rcv_sbus_data = msg::SBUS_DATA.receiver().unwrap();
    let mut rcv_arming_cmd = msg::RC_ARMING_CMD.receiver().unwrap();
    let mut rcv_loop_health = msg::LOOP_HEALTH.receiver().unwrap();
    let mut rcv_imu_active_id = msg::IMU_ACTIVE_ID.receiver().unwrap();
    let mut rcv_gyr_calibrating = msg::GYR_CALIBRATING.receiver().unwrap();
    let mut rcv_gyr_calibrated = msg::GYR_CALIBRATED.receiver().unwrap();
    let mut rcv_acc_calibrating = msg::ACC_CALIBRATING.receiver().unwrap();
    let mut rcv_acc_calibrated = msg::ACC_CALIBRATED.receiver().unwrap();

    // Output channels
    let snd_arm_blocker = msg::ARM_BLOCKER.sender();

    // Initialize the arm blocker flag as all high
    // (except for the calibration flags, which are not implemented yet)
    let mut local_arm_blocker_flag = ArmBlocker::all();
    local_arm_blocker_flag.set(ArmBlocker::ACC_CALIBIN, false);

    // Set flag to the initial value
    snd_arm_blocker.send(local_arm_blocker_flag);

    let mut ticker = Ticker::every(Duration::from_hz(10));

    '_infinite: loop {

        // Await arming command (or ticker, not explicitly needed)
        let res = select(rcv_arming_cmd.changed(), ticker.next()).await;
        local_arm_blocker_flag = defmt::unwrap!(snd_arm_blocker.try_get());
        
        // Handle case where arming command is received
        if let Either::First(arm) = res {
            let should_arm = (local_arm_blocker_flag & !ArmBlocker::CMD_DISARM).is_empty() && arm;
            local_arm_blocker_flag.set(ArmBlocker::CMD_DISARM, !should_arm);
        }

        // Check if the drone is in a high attitude ~ atan(sqrt(tan(pitch)^2 + tan(roll)^2))
        if let Some(attitude) = rcv_attitude_euler.try_changed() {
            let high_attitude = nalgebra::Vector2::new(attitude.x, attitude.y).norm() > INV_SQRT_2;
            local_arm_blocker_flag.set(ArmBlocker::HIGH_ATTITUDE, high_attitude);
        }

        // Check if the throttle command is too high
        if let Some(throttle) = rcv_throttle_setpoint.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::HIGH_THROTTLE_CMD, throttle > 0.1);
        }

        // Check if the attitude commands are too high
        use crate::common::types::AttitudeReference::{Angle, Rate};
        if let Some(Angle(setpoint) | Rate(setpoint)) = rcv_attitude_setpoint.try_changed() {
            let high_setpoint = setpoint.norm() > 0.1;
            local_arm_blocker_flag.set(ArmBlocker::HIGH_ATTITUDE_CMD, high_setpoint);
        }

        // Check that transmitter is not in failsafe
        if let Some(sbus_data) = rcv_sbus_data.try_changed() {
            let rx_failsafe = sbus_data.map_or(true, |p| p.failsafe);
            local_arm_blocker_flag.set(ArmBlocker::RX_FAILSAFE, rx_failsafe);
        }

        // Check if the loop frequency health is too low
        if let Some(loop_health) = rcv_loop_health.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::SYSTEM_LOAD, loop_health < 0.98);
        }

        // Check if the active IMU is currently assigned to a sensor
        if let Some(imu_redundancy) = rcv_imu_active_id.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::NO_GYR_DATA, imu_redundancy.is_none());
            local_arm_blocker_flag.set(ArmBlocker::NO_ACC_DATA, imu_redundancy.is_none());
        }

        // Check if the active gyroscpe is calibrated
        if let Some(gyr_calibrated) = rcv_gyr_calibrated.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::NO_GYR_CALIB, !gyr_calibrated);
        }

        // Check if the active accelerometer is calibrated
        if let Some(acc_calibrated) = rcv_acc_calibrated.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::NO_ACC_CALIB, !acc_calibrated);
        }

        // Check if the gyroscopes are currently calibrating
        if let Some(gyr_calibrating) = rcv_gyr_calibrating.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::GYR_CALIBIN, gyr_calibrating);
        }

        // Check if the accelerometers are currently calibrating
        if let Some(acc_calibrating) = rcv_acc_calibrating.try_changed() {
            local_arm_blocker_flag.set(ArmBlocker::ACC_CALIBIN, acc_calibrating);
        }

        // Check if the boot grace period has passed
        let boot_grace = Instant::MIN.elapsed() < Duration::from_secs(5);
        local_arm_blocker_flag.set(ArmBlocker::BOOT_GRACE, boot_grace);

        // Transmit changes to the arm blocker flag
        snd_arm_blocker.send(local_arm_blocker_flag);
    }
}