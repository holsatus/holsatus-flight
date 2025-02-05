use core::f32::consts::FRAC_1_SQRT_2;

use crate::{multi_receiver, rpc_message, signals as s, sync::rpc::Procedure, tasks::calibrator::CalibratorState, types::status::ArmingBlocker, NUM_IMU};
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::Vector3;

rpc_message! {
    ArmBlockerRpc: Request -> Response,
    async GetArmingBlocker{} -> ArmingBlocker,

    async SetConfig{cfg: ArmingBlockerConfig} -> bool,
    async GetConfig{} -> ArmingBlockerConfig,

    async SetMaxAttitudeCmd{max: f32} -> bool,
    async GetMaxAttitudeCmd{} -> f32,

    async SetMaxThrottleCmd{max: f32} -> bool,
    async GetMaxThrottleCmd{} -> f32,
}

pub static PROCEDURE: Procedure<ArmBlockerRpc> = Procedure::new();

#[derive(Debug, Clone)]
pub struct ArmingBlockerConfig {
    ignore_mask: ArmingBlocker,
    max_attitude: f32,
    max_throttle_cmd: f32,
    max_attitude_cmd: f32,
    boot_grace_secs: u8,
    system_load_limit: f32,
}

impl Default for ArmingBlockerConfig {
    fn default() -> Self {
        Self {
            ignore_mask: ArmingBlocker::empty(),
            max_attitude: FRAC_1_SQRT_2,
            max_throttle_cmd: 0.1,
            max_attitude_cmd: 0.1,
            boot_grace_secs: 5,
            system_load_limit: 0.98,
        }
    }
}

pub async fn get_arming_blocker() -> ArmingBlocker {
    PROCEDURE.request(GetArmingBlocker{}).await
        .expect("Failed to get arming blocker")
}


#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "arm_blocker";
    info!("{}: Task started", ID);

    // Input channels
    let mut rcv_attitude_euler =s::AHRS_ATTITUDE.receiver();
    let mut rcv_rc_status = s::RC_STATUS.receiver();
    let mut rcv_arm_motors = s::CMD_ARM_MOTORS.receiver();
    let mut rcv_calibrator_state = s::CALIBRATOR_STATE.receiver();
    let mut rcv_gyr_cal = multi_receiver!(s::CFG_MULTI_GYR_CAL, NUM_IMU);
    let mut rcv_acc_cal = multi_receiver!(s::CFG_MULTI_ACC_CAL, NUM_IMU);
    let mut rcv_rc_controls = s::RC_ANALOG_RATE.receiver();
    let mut rcv_usb_connected = s::USB_CONNECTED.receiver();
    // let mut rcv_loop_health = s::LOOP_HEALTH.receiver();
    // let mut rcv_landed_state = s::LANDED_STATE.receiver();

    // Output channels
    let mut snd_arm_blocker = s::ARMING_BLOCKER.sender();

    // Initialize the arm blocker flag as all high
    let mut local_arm_blocker_flag = ArmingBlocker::all();
    // Set flag to the initial value
    snd_arm_blocker.send(local_arm_blocker_flag);

    // Theoretically this task could run only whenever an arming command is received,
    // but for state visibility and debugging, we run it at a low fixed frequency.
    let mut ticker = Ticker::every(Duration::from_hz(1));
    let mut get_blocker_handle = None;

    let mut config = ArmingBlockerConfig::default();

    'infinite: loop {

        // Wait for procedure command (or ticker, not explicitly needed)
        match select(PROCEDURE.get_request(),  ticker.next(), ).await {
            // This thing runs when a request is received
            Either::First(request) => match request {

                Request::GetArmingBlocker(handle, ..) => {
                    get_blocker_handle = Some(handle);
                }

                Request::GetConfig(handle, ..) => {
                    handle.respond(config.clone());
                    continue 'infinite;
                },
                Request::SetConfig(handle, req) => {
                    config = req.cfg.clone();
                    handle.respond(true);
                    continue 'infinite;
                },

                Request::GetMaxAttitudeCmd(handle, ..) => {
                    handle.respond(config.max_attitude_cmd);
                    continue 'infinite;
                },
                Request::SetMaxAttitudeCmd(handle, req) => {
                    config.max_attitude_cmd = req.max;
                    handle.respond(true);
                    continue 'infinite;
                },

                Request::GetMaxThrottleCmd(handle, ..) => {
                    handle.respond(config.max_throttle_cmd);
                    continue 'infinite;
                },
                Request::SetMaxThrottleCmd(handle, req) => {
                    config.max_throttle_cmd = req.max;
                    handle.respond(true);
                    continue 'infinite;
                },


            },
            Either::Second(()) => ()
        }

        // Check if the drone is in a high attitude ~ atan(sqrt(tan(pitch)^2 + tan(roll)^2))
        if let Some(attitude) = rcv_attitude_euler.try_changed() {
            let high_attitude = Vector3::from(attitude).xy().norm() > config.max_attitude;
            local_arm_blocker_flag.set(ArmingBlocker::HIGH_ATTITUDE, high_attitude);
        }

        // Check if the attitude commands are too high
        if let Some(controls) = rcv_rc_controls.try_changed() {
            let controls_vec = Vector3::from(controls.roll_pitch_yaw());
            let high_throttle_cmd = controls.throttle() > config.max_throttle_cmd;
            let high_attitude_cmd = controls_vec.norm() > config.max_attitude_cmd;
            local_arm_blocker_flag.set(ArmingBlocker::HIGH_THROTTLE_CMD, high_throttle_cmd);
            local_arm_blocker_flag.set(ArmingBlocker::HIGH_ATTITUDE_CMD, high_attitude_cmd);
        }

        // Check that transmitter is not in failsafe
        if let Some(rc_status) = rcv_rc_status.try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::RX_FAILSAFE, rc_status.failsafe);
        }

        // Check if the active accelerometer is calibrated
        if let Some(acc_cal) = rcv_acc_cal[0].try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::NO_ACC_CALIB, !acc_cal.has_bias());
        }

        // Check if the active gyroscpe is calibrated
        if let Some(gyr_cal) = rcv_gyr_cal[0].try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::NO_GYR_CALIB, !gyr_cal.has_bias());
        }

        /*

        // Check if the loop frequency health is too low
        if let Some(loop_health) = rcv_loop_health.try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::SYSTEM_LOAD, loop_health < 0.98);
        }
        */

        // // Check if the active IMU is currently assigned to a sensor
        // if let Some(imu_redundancy) = rcv_imu_active_id.try_changed() {
        //     local_arm_blocker_flag.set(ArmingBlocker::NO_GYR_DATA, imu_redundancy.is_none());
        //     local_arm_blocker_flag.set(ArmingBlocker::NO_ACC_DATA, imu_redundancy.is_none());
        // }



        // Check if a sensor is currently calibrating
        if let Some(calibrating) = rcv_calibrator_state.try_changed() {
            let is_calibrating = matches!(calibrating, CalibratorState::Calibrating(_));
            local_arm_blocker_flag.set(ArmingBlocker::CALIBRATING, is_calibrating);
        }

        // Check if the USB is connected
        if let Some(usb_connected) = rcv_usb_connected.try_changed() {
            local_arm_blocker_flag.set(ArmingBlocker::USB_CONNECTED, usb_connected);
        }

        // Check if the boot grace period has passed
        let boot_grace = Instant::MIN.elapsed() < Duration::from_secs(config.boot_grace_secs as u64);
        local_arm_blocker_flag.set(ArmingBlocker::BOOT_GRACE, boot_grace);

        // Mask out the ignored flags
        let masked_flag = local_arm_blocker_flag.difference(config.ignore_mask);

        // If a request was made, respond to it
        if let Some(handle) = get_blocker_handle.take() {
            handle.respond(masked_flag);
        }

        // Transmit changes to the arm blocker flag
        snd_arm_blocker.send(masked_flag);
    }
}
