use super::{mav_mode, mav_state, Error};
use mavio::dialects::common::messages::*;

// Pick a lane man..
mod time {
    use embassy_time::Instant;

    pub fn us_u32() -> u32 {
        (0xFFFFFFFF & Instant::now().as_micros()) as u32
    }

    pub fn ms_u32() -> u32 {
        (0xFFFFFFFF & Instant::now().as_millis()) as u32
    }

    pub fn us_u64() -> u64 {
        Instant::now().as_micros()
    }
}

const TARGET_HZ: usize = 30;
const POSITION_SET: &'static [Generator] = &[
    Generator::AttitudeQuaternion,
    Generator::LocalPositionNed,
    Generator::LocalPositionNedSystemGlobalOffset,
    Generator::ServoOutputRaw,
    Generator::ScaledImu,
];

#[embassy_executor::task]
pub(crate) async fn stream_position_set() -> ! {
    let duration_micros = 1e6 / (POSITION_SET.len() * TARGET_HZ) as f32;
    let duration = embassy_time::Duration::from_micros(duration_micros as u64);
    loop {
        for generator in POSITION_SET {
            embassy_time::Timer::after(duration).await;
            super::CHANNEL
                .send(super::Message::SendGenerator {
                    generator: *generator,
                    target: super::Target::Broadcast,
                })
                .await;
        }
    }
}

macro_rules! define_message_set {
    (
        $(#[$meta:meta])*
        $generator_name:ident, $message_name:ident {
            $(
                $(#[$variant_meta:meta])*
                $variant:ident
            ),* $(,)?
        }
    ) => {
        #[repr(u32)]
        #[derive(Debug, Clone, Copy, mav_param::Enum, Default)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        $(#[$meta])*
        pub enum $generator_name {
            #[default]
            $(
                $(#[$variant_meta])*
                $variant = <::mavio::dialects::common::messages::$variant>::ID
            ),*
        }

        pub enum $message_name {
            $(
                $variant(::mavio::dialects::common::messages::$variant)
            ),*
        }

        impl $generator_name {
            pub fn generate(&self) -> Result<$message_name, Error> {
                match self {
                    $(
                        $generator_name::$variant => Ok($message_name::$variant(<::mavio::dialects::common::messages::$variant>::generate()?))
                    ),*
                }
            }

            pub fn from_id(id: u32) -> Option<Self> {
                let generator = match id {
                    $(
                        <::mavio::dialects::common::messages::$variant>::ID => Self::$variant,
                    )*
                    _ => return None
                };
                return Some(generator);
            }
        }

        impl ::mavio::protocol::IntoPayload for $message_name {
            fn encode(
                &self,
                version: mavio::prelude::MavLinkVersion,
            ) -> Result<::mavio::protocol::Payload, mavio::prelude::SpecError> {
                match self {
                    $(
                        $message_name::$variant(msg) => msg.encode(version)
                    ),*
                }
            }
        }

        impl ::mavio::protocol::MessageSpec for $message_name {
            fn id(&self) -> mavio::protocol::MessageId {
                match self {
                    $(
                        $message_name::$variant(msg) => msg.id()
                    ),*
                }
            }

            fn min_supported_mavlink_version(&self) -> mavio::prelude::MavLinkVersion {
                match self {
                    $(
                        $message_name::$variant(msg) => msg.min_supported_mavlink_version()
                    ),*
                }
            }

            fn crc_extra(&self) -> mavio::protocol::CrcExtra {
                match self {
                    $(
                        $message_name::$variant(msg) => msg.crc_extra()
                    ),*
                }
            }
        }

        impl mavio::Message for $message_name {}
    };
}

define_message_set! {
    Generator, MavMessage {
        Heartbeat,
        ScaledImu,
        HighresImu,
        AttitudeQuaternion,
        Attitude,
        ServoOutputRaw,
        RcChannelsRaw,
        RcChannelsScaled,
        AutopilotVersion,
        LocalPositionNed,
        LocalPositionNedSystemGlobalOffset,
        LinkNodeStatus,
    }
}

pub trait Generate: Sized + mavio::Message {
    fn generate() -> Result<Self, Error>;
}

impl Generate for Heartbeat {
    fn generate() -> Result<Self, Error> {
        Ok(Heartbeat {
            type_: mavio::dialects::minimal::enums::MavType::Quadrotor,
            autopilot: mavio::dialects::minimal::enums::MavAutopilot::Generic,
            base_mode: mav_mode::get(),
            custom_mode: 0,
            system_status: mav_state::get(),
            mavlink_version: 0x3,
        })
    }
}

impl Generate for HighresImu {
    fn generate() -> Result<Self, Error> {
        use mavio::default_dialect::enums::HighresImuUpdatedFlags;

        let mut message = HighresImu::default();
        message.time_usec = time::us_u64();

        // Get the IMU and magnetometer data
        if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
            message.xacc = imu.acc[0];
            message.yacc = imu.acc[1];
            message.zacc = imu.acc[2];
            message.xgyro = imu.gyr[0];
            message.ygyro = imu.gyr[1];
            message.zgyro = imu.gyr[2];
            message.fields_updated = message.fields_updated
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YACC
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZACC
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XGYRO
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YGYRO
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZGYRO
        }

        // Get the magnetometer data
        if let Some(mag) = crate::signals::CAL_MULTI_MAG_DATA[0].try_get() {
            message.xmag = mag[0];
            message.ymag = mag[1];
            message.zmag = mag[2];
            message.fields_updated = message.fields_updated
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XMAG
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_YMAG
                | HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_ZMAG;
        }

        Ok(message)
    }
}

impl Generate for ScaledImu {
    fn generate() -> Result<Self, Error> {
        let mut message = ScaledImu::default();
        message.time_boot_ms = time::ms_u32();

        if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
            message.xacc = (imu.acc[0] * 1e3) as i16;
            message.yacc = (imu.acc[1] * 1e3) as i16;
            message.zacc = (imu.acc[2] * 1e3) as i16;
            message.xgyro = (imu.gyr[0] * 1e3) as i16;
            message.ygyro = (imu.gyr[1] * 1e3) as i16;
            message.zgyro = (imu.gyr[2] * 1e3) as i16;
        }

        Ok(message)
    }
}

impl Generate for AttitudeQuaternion {
    fn generate() -> Result<Self, Error> {
        let mut message = AttitudeQuaternion::default();
        message.time_boot_ms = time::ms_u32();

        if let Some(est) = crate::signals::ESKF_ESTIMATE.try_get() {
            message.q1 = est.att[3];
            message.q2 = est.att[0];
            message.q3 = est.att[1];
            message.q4 = est.att[2];
        }

        if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
            message.rollspeed = imu.gyr[0];
            message.pitchspeed = imu.gyr[1];
            message.yawspeed = imu.gyr[2];
        }

        Ok(message)
    }
}

impl Generate for Attitude {
    fn generate() -> Result<Self, Error> {
        let mut message = Attitude::default();
        message.time_boot_ms = time::ms_u32();

        if let Some(att) = crate::signals::AHRS_ATTITUDE.try_get() {
            [message.roll, message.pitch, message.yaw] = att;
        }

        if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
            [message.rollspeed, message.pitchspeed, message.yawspeed] = imu.gyr;
        }

        Ok(message)
    }
}

impl Generate for ServoOutputRaw {
    fn generate() -> Result<Self, Error> {
        let mut message = ServoOutputRaw::default();
        message.time_usec = time::us_u32();

        if let Some(speeds) = crate::signals::MOTORS_STATE.try_get() {
            let speeds = speeds.as_speeds();
            message.servo1_raw = speeds[0];
            message.servo2_raw = speeds[1];
            message.servo3_raw = speeds[2];
            message.servo4_raw = speeds[3];
        }

        Ok(message)
    }
}

impl Generate for RcChannelsScaled {
    fn generate() -> Result<Self, Error> {
        let mut message = RcChannelsScaled::default();

        message.time_boot_ms = time::ms_u32();

        if let Some(channel) = crate::signals::RC_ANALOG_UNIT.try_get() {
            message.chan1_scaled = (channel.0[0] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan2_scaled = (channel.0[1] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan3_scaled = (channel.0[2] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan4_scaled = (channel.0[3] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan5_scaled = (channel.0[4] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan6_scaled = (channel.0[5] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan7_scaled = (channel.0[6] * 1e4).clamp(-1e4, 1e4) as i16;
            message.chan8_scaled = (channel.0[7] * 1e4).clamp(-1e4, 1e4) as i16;
        }

        Ok(message)
    }
}

impl Generate for RcChannelsRaw {
    fn generate() -> Result<Self, Error> {
        let mut message = RcChannelsRaw::default();
        message.time_boot_ms = time::ms_u32();

        if let Some(Some(channel)) = crate::signals::RC_CHANNELS_RAW.try_get() {
            message.chan1_raw = channel[0];
            message.chan2_raw = channel[1];
            message.chan3_raw = channel[2];
            message.chan4_raw = channel[3];
            message.chan5_raw = channel[4];
            message.chan6_raw = channel[5];
            message.chan7_raw = channel[6];
            message.chan8_raw = channel[7];
        }

        Ok(message)
    }
}

impl Generate for AutopilotVersion {
    fn generate() -> Result<Self, Error> {
        // TODO fill this in correctly
        Ok(Self::default())
    }
}

impl Generate for LocalPositionNed {
    fn generate() -> Result<Self, Error> {
        let mut msg = LocalPositionNed::default();

        if let Some(eskf_estimate) = crate::signals::ESKF_ESTIMATE.try_get() {
            msg.time_boot_ms = time::ms_u32();

            msg.x = eskf_estimate.pos[0];
            msg.y = eskf_estimate.pos[1];
            msg.z = eskf_estimate.pos[2];
            msg.vx = eskf_estimate.vel[0];
            msg.vy = eskf_estimate.vel[1];
            msg.vz = eskf_estimate.vel[2];
        }

        Ok(msg)
    }
}

impl Generate for LocalPositionNedSystemGlobalOffset {
    fn generate() -> Result<Self, Error> {
        let mut msg = LocalPositionNedSystemGlobalOffset::default();

        if let Some(eskf_estimate) = crate::signals::ESKF_ESTIMATE.try_get() {
            msg.time_boot_ms = time::ms_u32();

            msg.x = eskf_estimate.acc_bias[0];
            msg.y = eskf_estimate.acc_bias[1];
            msg.z = eskf_estimate.acc_bias[2];
            msg.roll = eskf_estimate.gyr_bias[0];
            msg.pitch = eskf_estimate.gyr_bias[1];
            msg.yaw = eskf_estimate.gyr_bias[2];
        }

        Ok(msg)
    }
}

impl Generate for LinkNodeStatus {
    fn generate() -> Result<Self, Error> {
        let msg = LinkNodeStatus::default();

        Ok(msg)
    }
}

impl Generate for GpsRawInt {
    fn generate() -> Result<Self, Error> {
        use crate::types::measurements::GnssFix;
        use mavio::default_dialect::enums::GpsFixType;
        use nalgebra::Vector3;

        let message = if let Some(gnss) = crate::signals::RAW_GNSS_DATA.try_get() {
            let fix_type = match gnss.fix {
                GnssFix::NoFix => GpsFixType::NoFix,
                GnssFix::Fix2D => GpsFixType::_2dFix,
                GnssFix::Fix3D => GpsFixType::_3dFix,
                GnssFix::TimeOnly => GpsFixType::NoFix,
            };

            GpsRawInt {
                time_usec: gnss.timestamp_us,
                lat: gnss.latitude_raw,
                lon: gnss.longitude_raw,
                alt: (gnss.height_above_msl * 1e3) as i32,
                eph: (gnss.horizontal_accuracy * 1000.) as u16,
                epv: (gnss.vertical_accuracy * 1000.) as u16,
                vel: (Vector3::new(gnss.velocity_north, gnss.velocity_east, gnss.velocity_down)
                    .norm()
                    * 100.) as u16,
                cog: (gnss.heading_motion.to_degrees() * 100.0) as u16,
                fix_type,
                satellites_visible: gnss.num_satellites,
                ..Default::default()
            }
        } else {
            GpsRawInt {
                time_usec: embassy_time::Instant::now().as_micros(),
                fix_type: GpsFixType::NoGps,
                satellites_visible: 0,
                ..Default::default()
            }
        };

        Ok(message)
    }
}
