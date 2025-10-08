use super::{MavServer, MAV_MODE, MAV_STATE};
use crate::errors::MavlinkError;
use embassy_time::Instant;
use mavio::{
    dialects::common::{
        enums::GpsFixType,
        messages::*,
    },
    prelude::V2,
    Frame,
};

macro_rules! define_generators {
    (
        $(
            $variant:ident
        ),*
        $(,)?
    ) => {
        #[repr(u32)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[derive(serde::Serialize, serde::Deserialize)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum Generator {
            $($variant = ::mavio::dialects::common::messages::$variant::ID),*
        }

        impl Generator {
            pub fn generator(&self) -> super::GenFn {
                match self {
                    $(Generator::$variant => ::mavio::dialects::common::messages::$variant::generate),*
                }
            }


            pub fn id(&self) -> ::mavio::protocol::MessageId {
                match self {
                    $(Generator::$variant => ::mavio::dialects::common::messages::$variant::ID),*
                }
            }

            #[allow(dead_code)]
            pub fn iter() -> &'static [Self] {
                &[$(Generator::$variant ),*]
            }
        }

        impl TryFrom<::mavio::protocol::MessageId> for Generator {
            type Error = MavlinkError;

            fn try_from(value: ::mavio::protocol::MessageId) -> Result<Self, Self::Error> {
                match value {
                    $(
                        ::mavio::dialects::common::messages::$variant::ID => Ok(Generator::$variant)
                    ),*,
                    _ => Err(MavlinkError::NoMessageHandler),
                }
            }
        }
    };
}

define_generators!(
    Heartbeat,
    HighresImu,
    AttitudeQuaternion,
    Attitude,
    ServoOutputRaw,
    RcChannelsRaw,
    RcChannelsScaled,
    AutopilotVersion,
    GpsRawInt,
);

trait Generate: Sized + mavio::Message {
    fn generate_self(server: &MavServer) -> Result<Self, MavlinkError>;
    fn generate(server: &MavServer) -> Result<Frame<V2>, MavlinkError> {
        let message = &Self::generate_self(server)?;
        server.build_frame(message)
    }
}

impl Generate for Heartbeat {
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        Ok(Heartbeat {
            type_: mavio::dialects::minimal::enums::MavType::Quadrotor,
            autopilot: mavio::dialects::minimal::enums::MavAutopilot::Generic,
            base_mode: MAV_MODE.get(),
            custom_mode: 0,
            system_status: MAV_STATE.get(),
            mavlink_version: 0x3,
        })
    }
}

impl Generate for HighresImu {
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        let mut message = HighresImu::default();
        message.time_usec = Instant::now().as_micros();

        use mavio::dialects::common::enums::HighresImuUpdatedFlags as F;

        // Get the IMU and magnetometer data
        if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
            message.xacc = imu.acc[0];
            message.yacc = imu.acc[1];
            message.zacc = imu.acc[2];
            message.xgyro = imu.gyr[0];
            message.ygyro = imu.gyr[1];
            message.zgyro = imu.gyr[2];
            message.fields_updated = message.fields_updated
                | F::HIGHRES_IMU_UPDATED_XACC
                | F::HIGHRES_IMU_UPDATED_YACC
                | F::HIGHRES_IMU_UPDATED_ZACC
                | F::HIGHRES_IMU_UPDATED_XGYRO
                | F::HIGHRES_IMU_UPDATED_YGYRO
                | F::HIGHRES_IMU_UPDATED_ZGYRO
        }

        // Get the magnetometer data
        if let Some(mag) = crate::signals::CAL_MULTI_MAG_DATA[0].try_get() {
            message.xmag = mag[0];
            message.ymag = mag[1];
            message.zmag = mag[2];
            message.fields_updated = message.fields_updated
                | F::HIGHRES_IMU_UPDATED_XMAG
                | F::HIGHRES_IMU_UPDATED_YMAG
                | F::HIGHRES_IMU_UPDATED_ZMAG;
        }

        Ok(message)
    }
}

impl Generate for AttitudeQuaternion {
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        let mut message = AttitudeQuaternion::default();
        message.time_boot_ms = Instant::now().as_millis() as u32;

        if let Some(att) = crate::signals::AHRS_ATTITUDE_Q.try_get() {
            message.q1 = att[3];
            message.q2 = att[0];
            message.q3 = att[1];
            message.q4 = att[2];
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
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        let mut message = Attitude::default();
        message.time_boot_ms = Instant::now().as_millis() as u32;

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
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        let mut message = ServoOutputRaw::default();
        // Note, 32-bit micro-seconds does not make much sense, so we use millis instead
        message.time_usec = (0xFFFFFFFF & Instant::now().as_micros()) as u32;

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
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        let mut message = RcChannelsScaled::default();

        // Note, 32-bit micro-seconds does not make much sense, so we use millis instead
        message.time_boot_ms = (0xFFFFFFFF & Instant::now().as_millis()) as u32;

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
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        let mut message = RcChannelsRaw::default();

        // Note, 32-bit micro-seconds does not make much sense, so we use millis instead
        message.time_boot_ms = (0xFFFFFFFF & Instant::now().as_millis()) as u32;

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
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        // TODO fill this in correctly
        Ok(Self::default())
    }
}

impl Generate for GpsRawInt {
    fn generate_self(_server: &MavServer) -> Result<Self, MavlinkError> {
        use crate::types::measurements::GnssFix;
        use nalgebra::Vector3;
        #[cfg(not(feature = "arch-std"))]
        use num_traits::Float;

        let message = if let Some(gnss) = crate::signals::RAW_GNSS_DATA.try_get() {
            let fix_type = match gnss.fix {
                GnssFix::NoFix => GpsFixType::NoFix,
                GnssFix::Fix2D => GpsFixType::_2dFix,
                GnssFix::Fix3D => GpsFixType::_3dFix,
                GnssFix::TimeOnly => GpsFixType::NoFix,
            };

            GpsRawInt {
                time_usec: Instant::now().as_micros(),
                lat: gnss.lat_raw,
                lon: gnss.lon_raw,
                alt: (gnss.altitude * 1e3) as i32,
                eph: (gnss.horiz_accuracy * 1000.) as u16,
                epv: (gnss.vert_accuracy * 1000.) as u16,
                vel: (Vector3::new(gnss.vel_east, gnss.vel_north, gnss.vel_down).norm() * 100.)
                    as u16,
                cog: (gnss.vel_east.atan2(gnss.vel_north).to_degrees() * 100.) as u16,
                fix_type,
                satellites_visible: gnss.satellites,
                ..Default::default()
            }
        } else {
            GpsRawInt {
                time_usec: Instant::now().as_micros(),
                fix_type: GpsFixType::NoGps,
                satellites_visible: 0,
                ..Default::default()
            }
        };

        Ok(message)
    }
}

pub(super) mod osd {}
