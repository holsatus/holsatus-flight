use core::array::from_fn;

use embassy_futures::select::{select, select_array, Either};
use embassy_time::{Duration, Ticker, Timer};

use crate::health::redundancy::{Mode, SensorEvaluator};
use crate::{get_ctrl_freq, multi_receiver, signals as s, NUM_IMU};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "imu_manager";
    info!("{}: Task started", ID);

    // Task inputs
    let mut rcv_multi_imu_data = multi_receiver!(s::CAL_MULTI_IMU_DATA, NUM_IMU);

    // Task outputs
    let mut snd_main_imu_data = s::CAL_IMU_DATA.sender();
    let mut snd_imu_modes = s::IMU_MODES.sender();

    // TODO - Make cfg to select the main IMU
    let mut _main_imu_idx: usize = 0;

    let hz = get_ctrl_freq!();

    // Create array of sensor evaluators
    let mut _acc_eval: [_; NUM_IMU] = from_fn(|_| SensorEvaluator::<10>::new());
    let mut gyr_eval: [_; NUM_IMU] = from_fn(|_| SensorEvaluator::<10>::new());

    // Tell the IMUs to start in active mode
    let mut imu_modes: [_; NUM_IMU] = [Mode::Active; NUM_IMU];
    snd_imu_modes.send(imu_modes);

    {
        // Wait for at least one IMU to start sending data
        debug!("{}: Waiting for IMUs to start sending data", ID);
        select_array(rcv_multi_imu_data.each_mut().map(|r| r.changed())).await;
        debug!("{}: IMUs started sending data", ID);
        let initial_counters = s::CAL_MULTI_IMU_DATA.each_ref().map(|ch| ch.get_msg_id());

        Timer::after_millis(100).await; // Wait a bit for other IMUs to start sending data
        let final_counters = s::CAL_MULTI_IMU_DATA.each_ref().map(|ch| ch.get_msg_id());

        imu_modes = from_fn(|idx| {
            let diff = final_counters[idx] - initial_counters[idx];
            let freq_ratio = diff as f32 / (hz as f32 / 10.0);
            match freq_ratio {
                r if r > 0.95 && r < 1.05 => Mode::Active,
                r if r > 0.8 => Mode::Idle(16),
                _ => Mode::Stopped,
            }
        });
        snd_imu_modes.send(imu_modes);
    }

    debug!("{}: IMU modes set: {:?}", ID, imu_modes);

    let mut ticker = Ticker::every(Duration::from_hz(hz as u64 / 5));
    let mut loop_counter: u8 = 0; // It is okay for this to wrap around

    info!("{}: Entering main loop", ID);
    loop {
        let futures = rcv_multi_imu_data.each_mut().map(|r| r.changed());

        match select(select_array(futures), ticker.next()).await {
            Either::First((imu_data, idx)) => {
                // acc_eval[idx].add_sample(imu_data.acc);
                // if acc_eval[idx].detect_stuck() {
                //     warn!("{}: Accelerometer {} is stalled", ID, idx);
                //     continue; // Short-circuit the loop
                // }

                // gyr_eval[idx].add_sample(imu_data.gyr);
                // if gyr_eval[idx].detect_stuck() {
                //     warn!("{}: Gyroscope {} is stuck", ID, idx);
                //     continue;
                // }

                // All checks passed, send the
                // data if it is the main IMU.

                if idx == _main_imu_idx {
                    snd_main_imu_data.send(imu_data);
                }
            }
            Either::Second(_) => {
                let idx = loop_counter as usize % NUM_IMU;
                if gyr_eval[idx].detect_stall() {
                    warn!("{}: IMU {} is stalled", ID, idx);
                    continue;
                }
            }
        }

        loop_counter = loop_counter.wrapping_add(1);
    }
}
