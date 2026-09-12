use crate::params::ParamTable;

pub const NUM_CHANNELS: usize = 16;
pub const NUM_DIGITALS: usize = 4;

#[derive(mav_param::Tree, Clone)]
pub struct Parameters {
    #[param(rename = "ch")]
    pub(super) channel_binding: Bindings,
}

crate::const_default!(
    Parameters => {
        channel_binding: Bindings::const_default(),
    }
);

#[derive(mav_param::Node, Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Bindings(pub(super) [Binding; NUM_CHANNELS]);

crate::const_default!(
    Bindings => Bindings({
        const POS_1: u16 = 886;
        const POS_2: u16 = 1500;
        const POS_3: u16 = 2114;

        const A_MIN: u16 = 886;
        const A_MAX: u16 = 2114;

        bindings(&[
            Binding::Analog(analog::Binding {
                flags: analog::Flags::empty(),
                axis: analog::Axis::Roll,
                deadband: 2,
                min: A_MIN,
                max: A_MAX,
            }),
            Binding::Analog(analog::Binding {
                flags: analog::Flags::REVERSE,
                axis: analog::Axis::Pitch,
                deadband: 2,
                min: A_MIN,
                max: A_MAX,
            }),
            Binding::Analog(analog::Binding {
                flags: analog::Flags::HALFRANGE,
                axis: analog::Axis::Throttle,
                deadband: 2,
                min: A_MIN,
                max: A_MAX,
            }),
            Binding::Analog(analog::Binding {
                flags: analog::Flags::empty(),
                axis: analog::Axis::Yaw,
                deadband: 2,
                min: A_MIN,
                max: A_MAX,
            }),
            digital_binds(&[ // Button A
                (POS_3, digital::Event::DoAccCalibrate),
            ]),
            digital_binds(&[ // Switch B
                (POS_1, digital::Event::FlightMode1),
                (POS_2, digital::Event::FlightMode2),
                (POS_3, digital::Event::FlightMode3),
            ]),
            digital_binds(&[ // Switch C
                (POS_1, digital::Event::DisarmVehicle),
                (POS_2, digital::Event::ArmVehicle),
                (POS_3, digital::Event::ForceArmVehicle),
            ]),
            digital_binds(&[ // Button D
                (POS_3, digital::Event::DoGyrCalibrate),
            ]),

            digital_binds(&[ /* Button E */ ]),
            digital_binds(&[ /* Button F */
                (POS_3, digital::Event::EskfResetOrigin),
            ]),
        ])
    })
);

const fn bindings(binds: &[Binding]) -> [Binding; NUM_CHANNELS] {
    if binds.len() > NUM_CHANNELS {
        core::panic!("Cannot assign more bindings than what is allocated for");
    }

    let mut bindings = [const { Binding::None }; NUM_CHANNELS];
    let mut index = 0;
    while index < binds.len() {
        bindings[index] = binds[index];
        index += 1
    }

    bindings
}

const fn digital_binds(binds: &[(u16, digital::Event)]) -> Binding {
    if binds.len() > NUM_DIGITALS {
        core::panic!("Cannot assign more bindings than what is allocated for");
    }

    let mut bindings = [digital::Binding {
        event: digital::Event::None,
        value: u16::MAX,
    }; NUM_DIGITALS];

    let mut index = 0;
    while index < binds.len() {
        bindings[index] = digital::Binding {
            value: binds[index].0,
            event: binds[index].1,
        };
        index += 1
    }

    Binding::Digital(bindings)
}

#[repr(u8)]
#[derive(mav_param::Enum, Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(super) enum Binding {
    #[default]
    None = 0,
    Analog(analog::Binding),
    Digital([digital::Binding; NUM_DIGITALS]),
}

/// The parameter table for the angular rate controller
pub static TABLE: ParamTable<Parameters> = ParamTable::default("rc");

pub mod analog {

    #[derive(mav_param::Tree, Debug, Default, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Binding {
        pub flags: Flags,
        pub axis: Axis,
        pub deadband: u8,
        pub min: u16,
        pub max: u16,
    }

    #[derive(mav_param::Node, Debug, Default, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Flags(pub u8);

    bitflags::bitflags! {
        impl Flags: u8 {
            const REVERSE = 1 << 0;
            const HALFRANGE = 1 << 1;
        }
    }

    #[repr(u8)]
    #[derive(mav_param::Enum, Debug, Default, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Axis {
        Roll = 0,
        Pitch,
        Yaw,
        Throttle,
        Auxi0,
        Auxi1,
        Auxi2,
        Auxi3,
        #[default]
        None = 255,
    }

    impl Binding {
        pub fn map(&self, data: u16) -> f32 {
            if self.flags.contains(Flags::HALFRANGE) {
                self.map_half_range(data)
            } else {
                self.map_full_range(data)
            }
        }

        /// Full-range, maps the input range to the output range (-1, 1). Typically
        /// used for roll, pitch and yaw commands.
        pub fn map_full_range(&self, data: u16) -> f32 {
            let Binding {
                flags,
                deadband,
                min,
                max,
                ..
            } = *self;

            // Center the value around 0
            let mut value = data.saturating_sub(min + 1) as i32 - (max - min) as i32 / 2;

            // Apply deadband on small values
            if value.abs() < deadband as i32 {
                value = 0
            }

            // Normalize and clamp the value to the range (-1, 1)
            let value = ((2 * value) as f32 / (max - min - 1) as f32).clamp(-1., 1.);

            if flags.contains(Flags::REVERSE) {
                -value
            } else {
                value
            }
        }

        /// Half-range, maps the input range to the output range (0, 1). Typically
        /// used for throttle commands.
        pub fn map_half_range(&self, data: u16) -> f32 {
            let Binding {
                flags,
                deadband,
                min,
                max,
                ..
            } = *self;

            // Shift the value down to 0
            let mut value = data.saturating_sub(min);

            // Apply deadband on small values
            if value < deadband as u16 {
                value = 0
            }

            // Normalize and clamp the value to the range (0, 1)
            let value = (value as f32 / (max - min) as f32).clamp(0., 1.);

            // Reverse the value if needed
            if flags.contains(Flags::REVERSE) {
                1.0 - value
            } else {
                value
            }
        }
    }
}

pub mod digital {

    #[derive(mav_param::Tree, Debug, Default, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Binding {
        #[param(rename = "val")]
        pub value: u16,
        #[param(rename = "evt")]
        pub event: Event,
    }

    #[repr(u16)]
    #[derive(mav_param::Enum, Debug, Default, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Event {
        #[default]
        None = 0,

        ArmVehicle = 100,
        ForceArmVehicle,
        DisarmVehicle,
        ForceDisarmVehicle,

        EskfResetOrigin = 150,

        DoAccCalibrate = 200,
        DoGyrCalibrate,
        DoMagCalibrate,

        FlightMode0 = 10000,
        FlightMode1 = 10001,
        FlightMode2 = 10002,
        FlightMode3 = 10003,
        FlightMode4 = 10004,
        FlightMode5 = 10005,
        FlightMode6 = 10006,
        FlightMode7 = 10007,
        FlightMode8 = 10008,
        FlightMode9 = 10009,
    }

    impl TryFrom<Event> for crate::tasks::commander::message::Command {
        type Error = ();

        fn try_from(value: Event) -> Result<Self, Self::Error> {
            use crate::tasks::commander::message::*;
            let command = match value {
                Event::None => return Err(()),
                Event::ArmVehicle => Command::ArmDisarm {
                    arm: true,
                    force: false,
                },
                Event::ForceArmVehicle => Command::ArmDisarm {
                    arm: true,
                    force: true,
                },
                Event::DisarmVehicle => Command::ArmDisarm {
                    arm: false,
                    force: false,
                },
                Event::ForceDisarmVehicle => Command::ArmDisarm {
                    arm: false,
                    force: true,
                },
                Event::EskfResetOrigin => Command::EskfResetOrigin,
                Event::DoAccCalibrate => Command::DoCalibration {
                    sensor_id: None,
                    sensor_type: SensorType::Accelerometer,
                },
                Event::DoGyrCalibrate => Command::DoCalibration {
                    sensor_id: None,
                    sensor_type: SensorType::Gyroscope,
                },
                Event::DoMagCalibrate => Command::DoCalibration {
                    sensor_id: None,
                    sensor_type: SensorType::Magnetometer,
                },

                // Use the vehicle-specified try_from implementation for any
                event if (10000..11000u16).contains(&(event as u16)) => {
                    Command::SetFlightMode(TryFrom::try_from(value)?)
                }

                event => {
                    warn!("[rc_binder] {:?} does not map to a command", event);
                    return Err(());
                }
            };

            Ok(command)
        }
    }
}
