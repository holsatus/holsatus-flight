use super::*;

pub const TX12_DEFAULT_MAP: TransmitterMap = TransmitterMap ([

    // Channel 1 - Right stick X
    ChannelType::Analog((
        AnalogCommand::Roll,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: true,
            reverse: false,
        }
    )),

    // Channel 2 - Right stick Y
    ChannelType::Analog((
        AnalogCommand::Pitch,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: true,
            reverse: true,
        }
    )),

    // Channel 3 - Left stick Y
    ChannelType::Analog((
        AnalogCommand::Thrust,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: false,
            reverse: false,
        }
    )),

    // Channel 4 - Left stick X
    ChannelType::Analog((
        AnalogCommand::Yaw,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: true,
            reverse: false,
        }
    )),

    // Channel 5 - Switch E
    ChannelType::Discrete([
        (172, EventRequest::Unbound),
        (992, EventRequest::AbortGyrCalib),
        (1810, EventRequest::StartGyrCalib),
    ]),

    // Channel 6 - Switch B
    ChannelType::Discrete([
        (172, EventRequest::AngleMode),
        (992, EventRequest::Unbound),
        (1810, EventRequest::RateMode),
        ]),

    // Channel 7 - Switch C
    ChannelType::Discrete([
        (172, EventRequest::DisarmMotors),
        (992, EventRequest::Unbound),
        (1810, EventRequest::ArmMotors),
    ]),

    // Channel 8 - Switch F
    ChannelType::Discrete([
        (172, EventRequest::Unbound),
        (992, EventRequest::Unbound),
        (1810, EventRequest::SaveConfig),
    ]),
    ChannelType::None,
    ChannelType::None,
    ChannelType::None,
    ChannelType::None,
    ChannelType::None,
    ChannelType::None,
    ChannelType::None,
    ChannelType::None,
]);

