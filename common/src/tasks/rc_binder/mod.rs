use crate::{
    signals as s,
    tasks::{commander, rc_binder::params::AnalogAxis},
    types::control::RcAnalog,
};

pub mod rates;

pub mod params {
    use crate::tasks::param_storage::Table;

    pub const NUM_CHANNELS: usize = 16;
    pub const NUM_DIGITALS: usize = 4;

    #[derive(mav_param::Tree, Clone)]
    pub struct Parameters {
        #[param(rename = "ch")]
        pub(super) channel_binding: Bindings,
    }

    crate::const_default!(
        Parameters => {
            channel_binding: Bindings::const_default()
        }
    );

    #[derive(mav_param::Node, Debug, Clone)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Bindings(pub(super) [Binding; NUM_CHANNELS]);

    crate::const_default!(
        Bindings => Bindings({
            const POS_1: u16 = 191;
            const POS_2: u16 = 997;
            const POS_3: u16 = 1792;

            const A_MIN: u16 = 174;
            const A_MAX: u16 = 1811;

            bindings(&[
                Binding::Analog(AnalogBinding {
                    flags: AnalogFlags::empty(),
                    axis: AnalogAxis::Roll,
                    deadband: 2,
                    min: A_MIN,
                    max: A_MAX,
                }),
                Binding::Analog(AnalogBinding {
                    flags: AnalogFlags::REVERSE,
                    axis: AnalogAxis::Pitch,
                    deadband: 2,
                    min: A_MIN,
                    max: A_MAX,
                }),
                Binding::Analog(AnalogBinding {
                    flags: AnalogFlags::HALFRANGE,
                    axis: AnalogAxis::Throttle,
                    deadband: 2,
                    min: A_MIN,
                    max: A_MAX,
                }),
                Binding::Analog(AnalogBinding {
                    flags: AnalogFlags::empty(),
                    axis: AnalogAxis::Yaw,
                    deadband: 2,
                    min: A_MIN,
                    max: A_MAX,
                }),
                digital_binds(&[ // Button A
                    (POS_3, DigitalEvent::DoAccCalibrate),
                ]),
                digital_binds(&[ // Switch B
                    (POS_1, DigitalEvent::SetModeRate),
                    (POS_2, DigitalEvent::SetModeAngle),
                    (POS_3, DigitalEvent::SetModeAutonomous),
                ]),
                digital_binds(&[ // Switch C
                    (POS_1, DigitalEvent::DisarmVehicle),
                    (POS_2, DigitalEvent::ArmVehicle),
                    (POS_3, DigitalEvent::ArmVehicle),
                ]),
                digital_binds(&[ // Button D
                    (POS_3, DigitalEvent::DoGyrCalibrate),
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

    const fn digital_binds(binds: &[(u16, DigitalEvent)]) -> Binding {
        if binds.len() > NUM_DIGITALS {
            core::panic!("Cannot assign more bindings than what is allocated for");
        }

        let mut bindings = [DigitalBinding {
            event: DigitalEvent::None,
            value: u16::MAX,
        }; NUM_DIGITALS];

        let mut index = 0;
        while index < binds.len() {
            bindings[index] = DigitalBinding {
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
        Analog(AnalogBinding),
        Digital([DigitalBinding; NUM_DIGITALS]),
    }

    #[derive(mav_param::Tree, Debug, Default, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub(super) struct AnalogBinding {
        pub(super) flags: AnalogFlags,
        pub(super) axis: AnalogAxis,
        pub(super) deadband: u8,
        pub(super) min: u16,
        pub(super) max: u16,
    }

    #[derive(mav_param::Node, Debug, Default, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub(super) struct AnalogFlags(pub u8);

    bitflags::bitflags! {
        impl AnalogFlags: u8 {
            const REVERSE = 1 << 0;
            const HALFRANGE = 1 << 1;
        }
    }

    #[repr(u8)]
    #[derive(mav_param::Enum, Debug, Default, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub(super) enum AnalogAxis {
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

    #[derive(mav_param::Tree, Debug, Default, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub(super) struct DigitalBinding {
        #[param(rename = "val")]
        pub(super) value: u16,
        #[param(rename = "evt")]
        pub(super) event: DigitalEvent,
    }

    #[repr(u16)]
    #[derive(mav_param::Enum, Debug, Default, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub(super) enum DigitalEvent {
        #[default]
        None = 0,

        ArmVehicle = 100,
        ForceArmVehicle,
        DisarmVehicle,
        ForceDisarmVehicle,

        DoAccCalibrate = 200,
        DoGyrCalibrate,
        DoMagCalibrate,

        SetModeRate = 300,
        SetModeAngle,
        SetModeAutonomous,
    }

    /// The parameter table for the angular rate controller
    pub static TABLE: Table<Parameters> = Table::new("rc", Parameters::const_default());

    impl TryFrom<DigitalEvent> for crate::tasks::commander::message::Command {
        type Error = ();

        fn try_from(value: DigitalEvent) -> Result<Self, Self::Error> {
            use crate::tasks::commander::message::*;
            let command = match value {
                DigitalEvent::None => return Err(()),
                DigitalEvent::ArmVehicle => ArmDisarm {
                    arm: true,
                    force: false,
                }
                .into(),
                DigitalEvent::ForceArmVehicle => ArmDisarm {
                    arm: true,
                    force: true,
                }
                .into(),
                DigitalEvent::DisarmVehicle => ArmDisarm {
                    arm: false,
                    force: false,
                }
                .into(),
                DigitalEvent::ForceDisarmVehicle => ArmDisarm {
                    arm: false,
                    force: true,
                }
                .into(),
                DigitalEvent::DoAccCalibrate => DoCalibration {
                    sensor_id: None,
                    sensor_type: SensorType::Accelerometer,
                }
                .into(),
                DigitalEvent::DoGyrCalibrate => DoCalibration {
                    sensor_id: None,
                    sensor_type: SensorType::Gyroscope,
                }
                .into(),
                DigitalEvent::DoMagCalibrate => DoCalibration {
                    sensor_id: None,
                    sensor_type: SensorType::Magnetometer,
                }
                .into(),
                DigitalEvent::SetModeRate => SetControlMode::Rate.into(),
                DigitalEvent::SetModeAngle => SetControlMode::Angle.into(),
                DigitalEvent::SetModeAutonomous => SetControlMode::Autonomous.into(),
            };

            Ok(command)
        }
    }

    impl AnalogBinding {
        pub fn map(&self, data: u16) -> f32 {
            if self.flags.contains(AnalogFlags::HALFRANGE) {
                self.map_half_range(data)
            } else {
                self.map_full_range(data)
            }
        }

        /// Full-range, maps the input range to the output range (-1, 1). Typically
        /// used for roll, pitch and yaw commands.
        pub fn map_full_range(&self, data: u16) -> f32 {
            let AnalogBinding {
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

            if flags.contains(AnalogFlags::REVERSE) {
                -value
            } else {
                value
            }
        }

        /// Half-range, maps the input range to the output range (0, 1). Typically
        /// used for throttle commands.
        pub fn map_half_range(&self, data: u16) -> f32 {
            let AnalogBinding {
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
            if flags.contains(AnalogFlags::REVERSE) {
                1.0 - value
            } else {
                value
            }
        }
    }
}

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "rc_mapper";
    info!("{}: Task started", ID);

    // Input signals
    let mut rcv_rc_channels = s::RC_CHANNELS_RAW.receiver();

    // Output signals
    let mut snd_rc_controls_unit = s::RC_ANALOG_UNIT.sender();

    let params = params::TABLE.read_initialized().await;
    let rc_bindings = params.channel_binding.clone();

    drop(params);

    // Store previous packet to detect changes in digital channels
    let mut prev_rc_channels = None;

    let mut rc_analog = RcAnalog([0.0; 8]);

    info!("{}: Entering main loop", ID);
    'infinite: loop {
        let Some(rc_channels) = rcv_rc_channels.changed().await else {
            continue 'infinite;
        };

        for index in 0..params::NUM_CHANNELS {
            let rc_value = rc_channels[index];

            match &rc_bindings.0[index] {
                params::Binding::None => continue,
                params::Binding::Analog(analog) => {
                    if analog.axis == AnalogAxis::None {
                        continue;
                    }

                    // Scale raw channel to unit-sized value
                    let unit_value = analog.map(rc_value);

                    // Update the result
                    rc_analog.0[analog.axis as u8 as usize] = unit_value;
                }
                params::Binding::Digital(digital) => {
                    // If value is same as previous, skip to avoid spamming
                    if prev_rc_channels.is_some_and(|rc: [u16; 16]| rc[index] == rc_value) {
                        continue;
                    }

                    // Propagate all all bound events
                    for bind in digital {
                        if rc_value != bind.value {
                            continue;
                        }

                        let Ok(command) = (bind.event).try_into() else {
                            continue;
                        };

                        info!("{}: Sending digital event: {:?}", ID, bind.event);
                        commander::PROCEDURE
                            .send(commander::Request {
                                command,
                                origin: commander::Origin::RemoteControl,
                            })
                            .await;
                    }
                }
            }
        }

        // Save the packet for next iteration and transmit analog values
        prev_rc_channels = Some(rc_channels);
        snd_rc_controls_unit.send(rc_analog);
    }
}
