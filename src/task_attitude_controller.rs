
use core::f32::consts::PI;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Instant;
use nalgebra::Vector3;
use pid_controller_rs::Pid;
use crate::channels;
use crate::config::definitions::ATTITUDE_LOOP_TIME_SECS;

use defmt::*;

#[derive(Clone, Debug, Copy, PartialEq)]
pub enum StabilizationMode {
    Inactive,
    Horizon(Vector3<f32>),
    Acro(Vector3<f32>)
}

impl Format for StabilizationMode {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt,"{}",Debug2Format(self))
    }
}

pub static FREQUENCY_SIG: PubSubChannel<CriticalSectionRawMutex,f32,1,1,1> = PubSubChannel::new();

static TASK_ID : &str = "[ATTITUDE_CONTROLLER]";

#[embassy_executor::task]
pub async fn attitude_controller(
    mut s_attitude_sense: channels::AttitudeSenseSub,
    mut s_attitude_int_enable : channels::AttitudeIntEnableSub,
    mut s_attitude_stab_mode : channels::AttitudeStabModeSub,
    p_attitude_actuate: channels::AttitudeActuatePub,
) {

    // Acquire stabilization mode
    let mut stabilization_mode = s_attitude_stab_mode.next_message_pure().await;

    // Setup controllers for pitch, roll and yaw, using a cascaded controller scheme.
    let mut pid_pitch_outer:Pid<_,_,_,_,_>  = Pid::new( 10., 0.1, 0., true, ATTITUDE_LOOP_TIME_SECS ).set_wrapping(-PI, PI);
    let mut pid_pitch_inner:Pid<_,_,_,_,_>  = Pid::new( 40., 1.0, 0.01, true, ATTITUDE_LOOP_TIME_SECS ).set_lp_filter(0.01);
    let mut pid_roll_outer: Pid<_,_,_,_,_>  = Pid::new( 10., 0.1, 0., true, ATTITUDE_LOOP_TIME_SECS ).set_wrapping(-PI, PI);
    let mut pid_roll_inner: Pid<_,_,_,_,_>  = Pid::new( 30., 1.0, 0.01, true, ATTITUDE_LOOP_TIME_SECS ).set_lp_filter(0.01);
    let mut pid_yaw_outer:  Pid<_,_,_,_,_>  = Pid::new( 8., 1e-3, 0., true, ATTITUDE_LOOP_TIME_SECS ).set_wrapping(-PI, PI);
    let mut pid_yaw_inner:  Pid<_,_,_,_,_>  = Pid::new( 60., 1.0, 0., true, ATTITUDE_LOOP_TIME_SECS ).set_lp_filter(0.01);

    let mut prev_time = Instant::now();
    let mut average_frequency = 0.0;
    let freq_publisher = FREQUENCY_SIG.immediate_publisher();

    let mut integral_enabled = true;

    info!("{}: Entering main loop",TASK_ID);
    loop {

        // Update reference signal (and stabilization mode) from channel
        channels::update_from_channel(&mut s_attitude_stab_mode, &mut stabilization_mode );

        // Enable/disable and reset integrators if signaled to do so
        if let Some(enable) = s_attitude_int_enable.try_next_message_pure() {
            if enable != integral_enabled {
                integral_enabled = enable;
                info!("{}: Setting integrals enabled -> {}",TASK_ID, enable);
                pid_pitch_outer.enable_reset_integral(enable);  pid_pitch_inner.enable_reset_integral(enable);
                pid_roll_outer.enable_reset_integral(enable);   pid_roll_inner.enable_reset_integral(enable);
                pid_yaw_outer.enable_reset_integral(enable);    pid_yaw_inner.enable_reset_integral(enable);
            }
        }

        // Wait for new measurements to arrive
        let (att_angle,att_rate) = s_attitude_sense.next_message_pure().await;

        // Generate actuation signal
        p_attitude_actuate.publish_immediate( match stabilization_mode {

            StabilizationMode::Horizon(reference) => {

                // Run outer part of cascaded control loop
                let outer_error = reference - att_angle;
                let inner_reference = Vector3::new(
                    pid_roll_outer.update( outer_error.x ),
                    pid_pitch_outer.update( outer_error.y ),
                    pid_yaw_outer.update( outer_error.z )
                );

                // Run inner part of cascaded control loop
                let inner_error = inner_reference - att_rate;
                Vector3::new(
                    pid_roll_inner.update( inner_error.x ),
                    pid_pitch_inner.update( inner_error.y ),
                    pid_yaw_inner.update( inner_error.z )
                )
            }

            StabilizationMode::Acro(reference) => {

                let error = reference - att_rate;
                Vector3::new(
                    pid_roll_inner.update( error.x ),
                    pid_pitch_inner.update( error.y ),
                    pid_yaw_inner.update( error.z )
                )
            }
            
            StabilizationMode::Inactive => {
                pid_pitch_outer.reset_integral();   pid_pitch_inner.reset_integral();
                pid_roll_outer.reset_integral();    pid_roll_inner.reset_integral();
                pid_yaw_outer.reset_integral();     pid_yaw_inner.reset_integral();
                Vector3::default()
            },
        });
         
        // Calculate loop time
        let time_now = Instant::now();
        let time_passed = time_now.duration_since(prev_time);
        average_frequency = 0.999*average_frequency + 0.001*(1e6 / time_passed.as_micros() as f64);
        prev_time = time_now;
        freq_publisher.publish_immediate(average_frequency as f32);
    }
}


