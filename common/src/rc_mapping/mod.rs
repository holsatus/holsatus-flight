use serde::{Deserialize, Serialize};

pub mod analog;
pub mod digital;

const NUM_CHANNELS: usize = 16;

// As of implementing, the analog binding takes up more memory than a single digital bind. So we miht
// as well try to cram in more digital binds, since they must share the same amount of storage.
const NUM_DIGITAL_BINDS: usize = size_of::<analog::AnalogBind>() / size_of::<Option<(u16, digital::RcEvent)>>();

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Binding {
    Analog(analog::AnalogBind),
    Digital(digital::DigitalBind),
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RcBindings(pub [Option<Binding>; NUM_CHANNELS]);

impl RcBindings {
    const fn const_new(binds: &[Binding]) -> Self {
        if binds.len() > NUM_CHANNELS {
            core::panic!("Cannot assign more bindings than what is allocated for");
        }

        let mut bindings = [None; NUM_CHANNELS];
        let mut index = 0;
        while index < binds.len() {
            bindings[index] = Some(binds[index]);
            index += 1
        }

        Self(bindings)
    }
}

impl Default for RcBindings {
    fn default() -> Self {
        RcBindings::const_default()
    }
}

impl RcBindings {
    fn const_default() -> Self {

        const POS_1: u16 = 191;
        const POS_2: u16 = 997;
        const POS_3: u16 = 1792;

        const A_MIN: u16 = 174;
        const A_MAX: u16 = 1811;

        RcBindings::const_new(&[

            // -----------------------------------------------
            // --- Analog bindings, primary control inputs ---
            // -----------------------------------------------

            // Right stick L/R
            Binding::Analog(analog::AnalogBind {
                axis: analog::Axis::Roll,
                in_min: A_MIN,
                in_max: A_MAX,
                deadband: 2,
                fullrange: true,
                reverse: false,
                rates: analog::Actual {
                    rate: 20.,
                    expo: 0.3,
                    cent: 15.0
                }.into()
            }),

            // Right stick U/D
            Binding::Analog(analog::AnalogBind {
                axis: analog::Axis::Pitch,
                in_min: A_MIN,
                in_max: A_MAX,
                deadband: 2,
                fullrange: true,
                reverse: true,
                rates: analog::Actual {
                    rate: 20.,
                    expo: 0.3,
                    cent: 15.0
                }.into(),
            }),

            // Left stick U/D
            Binding::Analog(analog::AnalogBind {
                axis: analog::Axis::Throt,
                in_min: A_MIN,
                in_max: A_MAX,
                deadband: 2,
                fullrange: false,
                reverse: false,
                rates: analog::Linear::new(
                    0., // in min
                    1., // in max
                    0.01, // out min
                    0.6, // out max
                ).into(),
            }),

            // Left stick L/R
            Binding::Analog(analog::AnalogBind {
                axis: analog::Axis::Yaw,
                in_min: A_MIN,
                in_max: A_MAX,
                deadband: 2,
                fullrange: true,
                reverse: false,
                rates: analog::Actual {
                    rate: 20.,
                    expo: 0.5,
                    cent: 10.0
                }.into(),
            }),

            // ------------------------------------------------------
            // --- Two-state buttons (POS_1 release, POS_3 press) ---
            // ------------------------------------------------------

            // Button A
            Binding::Digital(digital::DigitalBind::new(&[
                (POS_3, digital::RcEvent::CalibrateGyr),
            ])),

            // Button D
            Binding::Digital(digital::DigitalBind::new(&[
                (POS_3, digital::RcEvent::CalibrateGyr),
            ])),

            // --------------------------------------------------
            // --- Tri-state switches (POS_1 away from pilot) ---
            // --------------------------------------------------

            // Switch B
            Binding::Digital(digital::DigitalBind::new(&[
                (POS_1, digital::RcEvent::SetControlModeRate),
                (POS_2, digital::RcEvent::SetControlModeAngle),
                (POS_3, digital::RcEvent::SetControlModeVelocity),
            ])),

            // Switch C
            Binding::Digital(digital::DigitalBind::new(&[
                (POS_1, digital::RcEvent::DisarmMotors),
                (POS_2, digital::RcEvent::ArmMotors),
                (POS_3, digital::RcEvent::ArmMotors),
            ])),

            // It is more efficient to leave any remaining unbound channels out

            // Switch E
            // Binding::Digital(digital::DigitalBind::new(&[])),

            // Switch F
            // Binding::Digital(digital::DigitalBind::new(&[])),
        ])
    }
}
