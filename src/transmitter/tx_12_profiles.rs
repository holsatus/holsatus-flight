use super::*;

pub const TX12_8CH_DEFAULT_MAP: TransmitterMap = TransmitterMap ([

    // Channel 0 - Right stick X
    ChannelType::Analog((
        AnalogCommand::Roll,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: true,
            reverse: false,
            rates: Rates::new_standard(0.5, 20., 150.) 
        }
    )),

    // Channel 1 - Right stick Y
    ChannelType::Analog((
        AnalogCommand::Pitch,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: true,
            reverse: true,
            rates: Rates::new_standard(0.5, 20., 150.) 
        }
    )),

    // Channel 2 - Left stick Y
    ChannelType::Analog((
        AnalogCommand::Thrust,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: false,
            reverse: false,
            rates: Rates::None
        }
    )),

    // Channel 3 - Left stick X
    ChannelType::Analog((
        AnalogCommand::Yaw,
        AnalogConfig {
            in_min: 172,
            in_max: 1810,
            deadband: 2,
            fullrange: true,
            reverse: false,
            rates: Rates::new_standard(0.5, 20., 150.) 
        }
    )),

    // Channel 4 - Switch E
    ChannelType::Discrete([
        (172, EventRequest::Unbound),
        (992, EventRequest::AbortAccCalib),
        (1810, EventRequest::StartAccCalib),
    ]),

    // Channel 5 - Switch B
    ChannelType::Discrete([
        (172, EventRequest::AngleMode),
        (992, EventRequest::Unbound),
        (1810, EventRequest::RateMode),
        ]),

    // Channel 6 - Switch C
    ChannelType::Discrete([
        (172, EventRequest::DisarmMotors),
        (992, EventRequest::Unbound),
        (1810, EventRequest::ArmMotors),
    ]),

    // Channel 7 - Switch F
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

