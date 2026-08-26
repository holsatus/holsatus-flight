use crate::tasks::{
    param_storage::Table,
    rc_binder::rates::{Actual, Rates},
};

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
    /// Attitude disturbance rejection leak time constant
    pub att_leak_tc: f32,
    /// Configurable flags for the attitude controller
    pub flags: CtrlFlags,
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
    pub flag: RateAxisFlags,
    /// Time-constant of D-term LP filter
    pub dtau: f32,
    /// Prediction model time-constant
    pub pred: f32,
    /// Complementary filter time constant
    pub comp: f32,
    /// Proportional attitude disturbance rejection gain
    pub qint: f32,
    /// Rates to use in manual rates "acro" mode
    pub rc: Rates,
}

#[derive(Clone, Debug, mav_param::Node)]
pub struct RateAxisFlags(pub u8);

bitflags::bitflags! {
    impl RateAxisFlags: u8 {
        const D_TERM_LP = 1 << 0;
        const REF_SLEW = 1 << 1;
        const COMP_PRED = 1 << 2;
    }
}

#[derive(Clone, Debug, mav_param::Node)]
pub struct CtrlFlags(pub u8);

bitflags::bitflags! {
    impl CtrlFlags: u8 {
        /// Use leaky-quaternion to make SticksRate better able to reject disturbances
        /// This strategy uses the raw stick references, and will feel more snappy.
        const LEAK_QUAT_FILT = 1 << 0;
        /// Use leaky-quaternion to make SticksRate better able to reject disturbances
        /// This strategy uses the predicted gyro rates, and will feel smoother.
        const LEAK_QUAT_PRED = 1 << 1;
    }
}

crate::const_default!(
    Params => {
        x: AxisParameters {
            kp: 0.06,
            ki: 0.5,
            kd: 0.041,
            flag: RateAxisFlags(0),
            dtau: 0.0010,
            pred: 0.035,
            comp: 0.005,
            qint: 15.0,
            rc: Rates::Actual(Actual {
                rate: 20.,
                expo: 0.5,
                cent: 5.0,
            }),
        },
        y: AxisParameters {
            kp: 0.06,
            ki: 0.5,
            kd: 0.041,
            flag: RateAxisFlags(0),
            dtau: 0.0010,
            pred: 0.035,
            comp: 0.005,
            qint: 15.0,
            rc: Rates::Actual(Actual {
                rate: 20.,
                expo: 0.5,
                cent: 5.0,
            }),
        },
        z: AxisParameters {
            kp: 0.15,
            ki: 0.5,
            kd: 0.05,
            flag: RateAxisFlags(0),
            dtau: 0.001,
            pred: 0.08,
            comp: 0.01,
            qint: 5.0,
            rc: Rates::Actual(Actual {
                rate: 10.,
                expo: 0.5,
                cent: 5.0,
            }),
        },
        ref_slew: 500.0,
        ref_lp: 0.002,
        att_leak_tc: 1.0,
        flags: CtrlFlags(0),
    }
);

pub static TABLE: Table<Params> = Table::default("att");
