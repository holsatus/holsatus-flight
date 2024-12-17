use serde::{Deserialize, Serialize};

pub mod analog;
pub mod digital;

const NUM_CHANNELS: usize = 16;
const NUM_DIGITAL_BINDS: usize = 3;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Binding {
    Analog(analog::Analog),
    Digital(digital::DigitalCfg),
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RcBindings(pub [Option<Binding>; NUM_CHANNELS]);

struct Digital<const N: usize> {
    bindings: [(u16, digital::RcEvent); N],
}

impl <const N: usize> From<Digital<N>> for Binding {
    fn from(value: Digital<N>) -> Self {
        let mut bindings = [None; NUM_DIGITAL_BINDS];
        for (binding, value_binding) in bindings.iter_mut().zip(value.bindings) {
            *binding = Some(value_binding);
        }
        Binding::Digital(digital::DigitalCfg(bindings))
    }
}

macro_rules! rc_bindings {
    ($($binding:expr),* $(,)?) => {{
        #[allow(unused)]
        use analog::*;
        use digital::*;
        let mut bindings = [None; NUM_CHANNELS];
        let mut _i = 0;
        $(
            let converted: Binding = $binding.into();
            bindings[_i] = Some(converted);
            _i += 1;
        )*
        RcBindings(bindings)
    }};
}

impl Default for RcBindings {
    fn default() -> Self {
        rc_bindings!(
            // Right stick L/R
            Analog { 
                axis: Axis::Roll,
                in_min: 174,
                in_max: 1811,
                deadband: 2,
                fullrange: true,
                reverse: false,
                rates: Actual {
                    rate: 20.,
                    expo: 0.3,
                    cent: 15.0
                }.into(),
            },

            // Right stick U/D
            Analog { 
                axis: Axis::Pitch,
                in_min: 174,
                in_max: 1811,
                deadband: 2,
                fullrange: true,
                reverse: true,
                rates: Actual {
                    rate: 20.,
                    expo: 0.3,
                    cent: 15.0
                }.into(),
            },

            // Left stick U/D
            Analog { 
                axis: Axis::Throttle,
                in_min: 174,
                in_max: 1811,
                deadband: 2,
                fullrange: false,
                reverse: false,
                rates: Linear::new(
                    0., // in min
                    1., // in max
                    0.01, // out min
                    0.6, // out max
                ).into(),
            },

            // Left stick L/R
            Analog { 
                axis: Axis::Yaw,
                in_min: 174,
                in_max: 1811,
                deadband: 2,
                fullrange: true,
                reverse: false,
                rates: Actual {
                    rate: 20.,
                    expo: 0.5,
                    cent: 10.0
                }.into(),
            },

            // Button A 
            Digital{bindings: [
                (1792, RcEvent::CalibrateGyr),
            ]},

            // Switch B
            Digital {bindings: [
                (191,  RcEvent::AngleMode),
                (997,  RcEvent::RateMode),
                (1792, RcEvent::RateMode),
            ]},

            // Switch C
            Digital {bindings: [
                (191,  RcEvent::DisarmMotors),
                (997,  RcEvent::ArmMotors),
                (1792, RcEvent::ArmMotors),
            ]},
            
            // Button D
            Digital{bindings: [
                (1792, RcEvent::CalibrateGyr),
            ]},

            // Switch E
            Digital {bindings: []},

            // Switch F 
            Digital {bindings: [
                (191,  RcEvent::StopMotorTest),
                (1792, RcEvent::StartMotorTest),
            ]},
        )
    }
}
