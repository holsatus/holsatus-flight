use core::array::from_fn;
use core::sync::atomic::Ordering;

use crate::filters::{Complementary, Lowpass, NthOrderLowpass, SlewRate};
use crate::filters::rate_pid::RatePid;
use crate::sync::watch::Watch;
use crate::{get_or_warn, multi_receiver, signals as sig, NUM_IMU};
use embassy_futures::select::{select3, Either3::{First, Second, Third}};

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "rate_loop";
    info!("{}: Task started", ID);

    // Task inputs
    let mut rcv_imu_data = multi_receiver!(sig::CAL_MULTI_IMU_DATA, NUM_IMU);
    let mut rcv_throttle_sp = sig::TRUE_THROTTLE_SP.receiver();
    let mut rcv_rate_sp = sig::TRUE_RATE_SP.receiver();
    let mut rcv_rate_int_en = sig::ATTITUDE_INT_EN.receiver();
    let mut rcv_cfg_rate_loop = sig::CFG_RATE_LOOP_PIDS.receiver();
    let mut rcv_cfg_vehicle_info = sig::CFG_VEHICLE_INFO.receiver();
    let mut rcv_cfg_control_freq = sig::CFG_CONTROL_FREQ.receiver();

    // Task outputs
    let mut snd_ctrl_motors = sig::CTRL_MOTORS.sender();
    let mut snd_rate_pid_terms = sig::RATE_PID_TERMS.sender();
    let mut snd_comp_fuse_gyr = sig::COMP_FUSE_GYR.sender();
    let mut snd_slew_rate_sp = sig::SLEW_RATE_SP.sender();
    let mut snd_ff_pred_gyr = sig::FF_PRED_GYR.sender();

    // Sampling time
    let mut ts: f32 = 1.0 / rcv_cfg_control_freq.get().await as f32;

    // Here we do not use default values, but instead wait for the first configuration
    let pids_cfg = get_or_warn!(rcv_cfg_rate_loop).await;
    let motor_mix = get_or_warn!(rcv_cfg_vehicle_info).await.motor_mix;

    // Define PID controllers
    let mut pid = [
        RatePid::new_from_cfg(pids_cfg.x, ts),
        RatePid::new_from_cfg(pids_cfg.y, ts),
        RatePid::new_from_cfg(pids_cfg.z, ts),
    ];

    // TODO The following values should not be hard-coded

    // Slew-rate limiter for the reference signal
    let mut slew_filt_sp = [
        SlewRate::new(300., ts),
        SlewRate::new(300., ts),
        SlewRate::new(300., ts),
    ];

    // Lowpass limiter for the slew-limited reference signal
    let mut lp_filt_sp = [
        NthOrderLowpass::<_, 2>::new(0.005, ts),
        NthOrderLowpass::<_, 2>::new(0.005, ts),
        NthOrderLowpass::<_, 2>::new(0.005, ts),
    ];

    // Lowpass filters will act as feed-forward prediction.
    // We designed the closed loop system to have a bandwidth of 25Hz
    let mut pred_model = [
        Lowpass::new(0.04, ts),
        Lowpass::new(0.04, ts),
        Lowpass::new(0.04, ts),
    ];

    // Complementary filter used to smoothen gyro data
    let mut comp = [
        Complementary::new(1. / 100., ts),
        Complementary::new(1. / 100., ts),
        Complementary::new(1. / 100., ts),
    ];

    // Disable all integral controllers initially
    pid.iter_mut().for_each(|pid| pid.enable_reset_integral(false));

    // We await for the initial setpoints to be available
    let mut rate_sp = get_or_warn!(rcv_rate_sp).await;
    let mut throttle_sp = get_or_warn!(rcv_throttle_sp).await;
    let mut filtered_sp = rate_sp.clone();
    let mut comp_fuse_gyr = [0.0; 3];
    let mut ff_pred_gyr = [0.0; 3];

    info!("{}: Entering main loop", ID);
    loop {

        // Ensure we are reading from the correct IMU
        let active = sig::ACTIVE_IMU.load(Ordering::Relaxed);

        match select3(
            rcv_imu_data[active].changed(),
            rcv_cfg_rate_loop.changed(),
            rcv_cfg_control_freq.changed(),
        )
        .await
        {
            // On receiving new IMU or MARG data
            First(imu_data) => {

                // Try to update rate and throttle setpoints
                rate_sp = rcv_rate_sp.try_get().unwrap_or(rate_sp);
                throttle_sp = rcv_throttle_sp.try_get().unwrap_or(throttle_sp);

                // Enable or disable integral controllers
                if let Some(enable) = rcv_rate_int_en.try_changed() {
                    debug!("{}: Integrators enabled: {}", ID, enable);
                    pid.iter_mut().for_each(|pid| pid.enable_reset_integral(enable));
                }

                // Apply prediction, filtering and control pipeline
                let pid_out: [f32; 3] = from_fn(|axis| {
                    // Make prediction of the gyroscope based on (previous) reference
                    ff_pred_gyr[axis] = pred_model[axis].update(filtered_sp[axis]);

                    // Apply slight lowpass filter to the reference signal
                    filtered_sp[axis] = slew_filt_sp[axis].update(rate_sp[axis]);

                    // Slew-rate limit the reference signal (avoids D-term clipping)
                    filtered_sp[axis] = lp_filt_sp[axis].update(filtered_sp[axis]);

                    // Fuse gyro and prediction using complementary filter
                    comp_fuse_gyr[axis] = comp[axis].update(imu_data.gyr[axis], ff_pred_gyr[axis]);

                    // Update PID controller
                    pid[axis].update(filtered_sp[axis], comp_fuse_gyr[axis], ff_pred_gyr[axis])
                });

                // Apply xyz and throttle to mixing function
                let ctrl_motors = motor_mix.mixing_fn(throttle_sp, pid_out, false);

                // Extract PID terms for logging
                let rate_pid_terms = from_fn(|axis| *pid[axis].get_terms());

                // Use critical section here to reduce number of interrupt enable/disable events
                critical_section::with(|_| {
                    // Only send control signals if the output is not overridden
                    if !sig::OUTPUT_OVERRIDE.load(Ordering::Relaxed) {
                        snd_ctrl_motors.send(ctrl_motors);
                    }
                    snd_rate_pid_terms.send(rate_pid_terms);
                    snd_comp_fuse_gyr.send(comp_fuse_gyr);
                    snd_ff_pred_gyr.send(ff_pred_gyr);
                    snd_slew_rate_sp.send(filtered_sp);
                });
            }

            // On receiving a new configuration
            Second(new_pids) => {
                info!("{}: New PID configuration received", ID);
                pid[0].set_config(new_pids.x);
                pid[1].set_config(new_pids.y);
                pid[2].set_config(new_pids.z);

                pid[0].reset_integral();
                pid[1].reset_integral();
                pid[2].reset_integral();
            }

            // On receiving a new control frequency
            Third(new_freq) => {
                ts = 1.0 / new_freq as f32;
                comp.iter_mut().for_each(|c| c.set_dt(ts));
                pid.iter_mut().for_each(|p| p.set_ts(ts));
                pred_model.iter_mut().for_each(|lp| lp.set_dt(ts));
                slew_filt_sp.iter_mut().for_each(|s| s.set_dt(ts));
            }
        }
    }
}
