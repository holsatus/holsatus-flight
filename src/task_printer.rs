use defmt::*;
use embassy_time::{Ticker, Duration};
use crate::channels;

static TASK_ID : &str = "[PRINTER]";

#[embassy_executor::task]
pub async fn printer(
    mut sub_imu_reading: channels::ImuReadingSub,
    mut sub_attitude_sense: channels::AttitudeSenseSub,
    mut sub_attitude_actuate: channels::AttitudeActuateSub,
    mut sub_attitude_stab_mode: channels::AttitudeStabModeSub,
    mut sub_thrust_actuate: channels::ThrustActuateSub,
    mut sub_motor_state: channels::MotorStateSub,
    mut sub_flight_mode: channels::FlightModeSub,
) {

    let mut sub_con_frequency = crate::task_attitude_controller::FREQUENCY_SIG.subscriber().unwrap();
    let mut sub_icm_frequency = crate::drivers::task_icm20948_driver::FREQUENCY_SIG.subscriber().unwrap();

    let mut imu_reading:Option<channels::ImuReadingType> = sub_imu_reading.try_next_message_pure();
    let mut attitude_sense:Option<channels::AttitudeSenseType> = sub_attitude_sense.try_next_message_pure();
    let mut attitude_actuate:Option<channels::AttitudeActuateType> = sub_attitude_actuate.try_next_message_pure();
    let mut attitude_stab_mode:Option<channels::AttitudeStabModeType> = sub_attitude_stab_mode.try_next_message_pure();
    let mut thrust_actuate:Option<channels::ThrustActuateType> = sub_thrust_actuate.try_next_message_pure();
    let mut motor_state:Option<channels::MotorStateType> = sub_motor_state.try_next_message_pure();
    let mut flight_mode:Option<channels::FlightModeType> = sub_flight_mode.try_next_message_pure();
    let mut con_frequency:Option<f32> = None;
    let mut icm_frequency:Option<f32> = None;

    let mut ticker = Ticker::every(Duration::from_hz(1));
    info!("{}: Entering main loop",TASK_ID);
    loop {

        if let Some(value) = sub_imu_reading.try_next_message_pure() {imu_reading = Some(value);}
        if let Some(value) = sub_attitude_sense.try_next_message_pure() {attitude_sense = Some(value);}
        if let Some(value) = sub_attitude_actuate.try_next_message_pure() {attitude_actuate = Some(value);}
        if let Some(value) = sub_attitude_stab_mode.try_next_message_pure() {attitude_stab_mode = Some(value);}
        if let Some(value) = sub_thrust_actuate.try_next_message_pure() {thrust_actuate = Some(value);}
        if let Some(value) = sub_motor_state.try_next_message_pure() {motor_state = Some(value);}
        if let Some(value) = sub_flight_mode.try_next_message_pure() {flight_mode = Some(value);}
        if let Some(value) = sub_con_frequency.try_next_message_pure() { con_frequency = Some(value); }
        if let Some(value) = sub_icm_frequency.try_next_message_pure() { icm_frequency = Some(value); }

        debug!("\nImuData:{}\nAttitudeSens:{}\nAttitudeAct:{}\nAttitudeStab:{}\nThrustActuate:{}\nMotorState:{}\nFlightMode:{}\nConFrequency:{}\nIcmFrequency:{}",
            Debug2Format(&imu_reading),
            Debug2Format(&attitude_sense),
            Debug2Format(&attitude_actuate),
            Debug2Format(&attitude_stab_mode),
            Debug2Format(&thrust_actuate),
            motor_state,
            flight_mode,
            con_frequency,
            icm_frequency
        );

        ticker.next().await;

    }
}
