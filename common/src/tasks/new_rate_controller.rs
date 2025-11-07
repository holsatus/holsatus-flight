use embassy_futures::select::select;
use nalgebra::{Matrix4};

use crate::filters::rate_pid::RatePid;
use crate::filters::{Complementary, Lowpass, NthOrderLowpass, SlewRate};
use crate::sync::channel::{self, Channel};
use crate::sync::watch::{Receiver, Watch};
use crate::tasks::eskf::EskfEstimate;
use crate::types::measurements::Imu6DofData;
use crate::{ConstDefault, signals as sig};

pub mod params {
    use crate::tasks::param_storage::Table;

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {
        /// Roll-axis related parameters
        pub x: AxisParameters,
        /// Pitch-axis related parameters
        pub y: AxisParameters,
        /// Yaw-axis related parameters
        pub z: AxisParameters,
        /// Slewrate limiter for reference signal
        pub ref_slew: f32,
        /// Low-pass filter for reference signal
        pub ref_lp: f32,
    }

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct AxisParameters {
        /// Proportional gain
        pub kp: f32,
        /// Integral gain
        pub ki: f32,
        /// Derivative gain
        pub kd: f32,
        /// Configuration flags
        pub cfg: AxisFlags,
        /// Time-constant of D-term LP filter
        pub dtau: f32,
        /// Prediction model time-constant
        pub pred: f32,
        /// Complementary filter time constant
        pub comp: f32,
    }

    crate::const_default!(
        Params => {
            x: AxisParameters::const_default(),
            y: AxisParameters::const_default(),
            z: AxisParameters::const_default(),
            ref_slew: 500.0,
            ref_lp: 0.01,
        }
    );

    crate::const_default!(
        AxisParameters => {
            kp: 0.08,
            ki: 0.5,
            kd: 0.03,
            cfg: AxisFlags(0),
            dtau: 0.001,
            pred: 0.04,
            comp: 0.005,
        }
    );

    #[derive(Clone, Debug, mav_param::Node)]
    pub struct AxisFlags(pub u8);

    bitflags::bitflags! {
        impl AxisFlags: u8 {
            const D_TERM_LP = 1 << 0;
            const REF_SLEW = 1 << 1;
            const COMP_PRED = 1 << 2;
        }
    }

    pub static TABLE: Table<Params> = Table::new("rate", Params::const_default());
}


enum Message<'a> {
    RefreshParams,
    SourceSetpoint(&'a Watch<[f32; 3]>),
    SourceImuData(&'a Watch<Imu6DofData<f32>>),
}

static INBOX: Channel<Message<'static>, 1> = Channel::new();

#[embassy_executor::task]
pub async fn entry() -> ! {
    RateController::default().run().await
}

struct RateController<'a, const N: usize> {
    inbox: channel::Receiver<'a, Message<'a>, N>,
    recv_setpoint: Receiver<'a, [f32; 3]>,
    recv_imu_data: Receiver<'a, Imu6DofData<f32>>,
    recv_eskf_estimate: Receiver<'a, EskfEstimate>,
    recv_int_enable: Receiver<'a, bool>,
    recv_z_thrust_sp: Receiver<'a, f32>,
    axis: [ControlAxis; 3],
    motor_mixer: Matrix4<f32>,
}

impl Default for RateController<'static, 1> {
    fn default() -> Self {
        let dt = 0.001;
        let params = params::Params::DEFAULT;
        Self { 
            inbox: INBOX.receiver(), 
            recv_setpoint: sig::TRUE_RATE_SP.receiver(),
            recv_imu_data: sig::CAL_MULTI_IMU_DATA[0].receiver(), 
            recv_eskf_estimate: sig::ESKF_ESTIMATE.receiver(), 
            recv_int_enable: sig::ATTITUDE_INT_EN.receiver(), 
            recv_z_thrust_sp: sig::TRUE_Z_THRUST_SP.receiver(), 
            axis: [
                ControlAxis {
                    pid: RatePid::new(params.x.kp, params.x.ki, params.x.kd, params.x.dtau, dt),
                    sp_slew_rate: SlewRate::new(params.ref_slew, dt),
                    sp_lp_filter: NthOrderLowpass::new(params.ref_lp, dt),
                    precition_model: Lowpass::new(params.x.pred, dt),
                    complementary: Complementary::new(params.x.pred, dt),
                },
                    ControlAxis {
                    pid: RatePid::new(params.y.kp, params.y.ki, params.y.kd, params.y.dtau, dt),
                    sp_slew_rate: SlewRate::new(params.ref_slew, dt),
                    sp_lp_filter: NthOrderLowpass::new(params.ref_lp, dt),
                    precition_model: Lowpass::new(params.y.pred, dt),
                    complementary: Complementary::new(params.y.pred, dt),
                },
                ControlAxis {
                    pid: RatePid::new(params.z.kp, params.z.ki, params.z.kd, params.z.dtau, dt),
                    sp_slew_rate: SlewRate::new(params.ref_slew, dt),
                    sp_lp_filter: NthOrderLowpass::new(params.ref_lp, dt),
                    precition_model: Lowpass::new(params.z.pred, dt),
                    complementary: Complementary::new(params.z.pred, dt),
                },
            ], 
            motor_mixer: Default::default()
        }
    }
}

struct ControlAxis {
    pid: RatePid,
    sp_slew_rate: SlewRate<f32>,
    sp_lp_filter: NthOrderLowpass<f32, 2>,
    precition_model: Lowpass<f32>,
    complementary: Complementary<f32>,
}

impl <'a, const N: usize> RateController<'a, N> {
    async fn run(&mut self) -> ! {

        self.handle_message(Message::RefreshParams).await;

        loop {
            match select(self.inbox.receive(), self.recv_imu_data.changed()).await {
                embassy_futures::select::Either::First(message) => self.handle_message(message).await,
                embassy_futures::select::Either::Second(imu_data) => self.update_filters(imu_data),
            }
        }
    }

    async fn handle_message(&mut self, message: Message<'a>) {
        match message {
            Message::RefreshParams => {
                info!("[rate_controller] Refreshing parameters");
                let params = params::TABLE.read().await;
                // Update stuff using parameters
            },
            Message::SourceSetpoint(watch) => {
                info!("[rate_controller] Changing setpoint data source");
                self.recv_setpoint = watch.receiver();
                _ = self.recv_setpoint.try_changed();
            },
            Message::SourceImuData(watch) => {
                info!("[rate_controller] Changing IMU data source");
                self.recv_imu_data = watch.receiver();
                _ = self.recv_imu_data.try_changed();
            },
        }
    }

    fn update_filters(&mut self, imu_data: Imu6DofData<f32>) {

    }
}
