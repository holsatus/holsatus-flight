use core::{f32, sync::atomic::Ordering};

use embassy_time::Instant;
use mavio::{dialects::common::{enums::{GpsFixType, HighresImuUpdatedFlags, MavModeFlag, MavState}, messages::{Attitude, AttitudeQuaternion, GpsRawInt, Heartbeat, HighresImu, RcChannelsScaled, ServoOutputRaw}}, prelude::V2, protocol::Payload, Frame, Message};

use crate::errors::MavlinkError;
use super::{MavServer, MAV_MODE, MAV_STATE};

#[derive(Debug, Clone, Copy)]
pub struct Params {
    param1: f32,
    param2: f32,
    param3: f32,
    param4: f32,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            param1: f32::NAN,
            param2: f32::NAN,
            param3: f32::NAN,
            param4: f32::NAN,
        }
    }
}

macro_rules! define_sendable {
    (enum $name:ident { $($variant:ident @ $gen_fn:ident),* $(,)?}) => {
        #[repr(u32)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum $name {
            $($variant = ::mavio::dialects::common::messages::$variant::message_id()),*
        }

        impl $name {
            pub fn gen_fn(&self) -> super::GenFn {
                match self {
                    $($name::$variant => $gen_fn),*
                }
            }

            #[allow(dead_code)]
            pub fn iter() -> impl Iterator<Item = $name> {
                [$( $name::$variant ),*].into_iter()
            }
        }

        impl TryFrom<u32> for $name {
            type Error = MavlinkError;

            fn try_from(value: u32) -> Result<Self, Self::Error> {
                match value {
                    $(id if id == ::mavio::dialects::common::messages::$variant::message_id() => Ok($name::$variant)),*,
                    _ => Err(MavlinkError::NoMessageHandler),
                }
            }
        }
    };
}

define_sendable!(
    enum MavSendable {
        Heartbeat @ heartbeat,
        HighresImu @ highres_imu,
        AttitudeQuaternion @ attitude_quaternion,
        Attitude @ attitude,
        ServoOutputRaw @ servo_output_raw,
        RcChannelsScaled @ rc_channels_scaled,
        AutopilotVersion @ autopilot_version,
        // PidInternalsRate @ pid_internals_rate,
        GpsRawInt @ gps_raw_int,
    }
);

pub(super) fn heartbeat(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {
    let message = Heartbeat {
        type_: mavio::dialects::minimal::enums::MavType::Quadrotor,
        autopilot: mavio::dialects::minimal::enums::MavAutopilot::Generic,
        base_mode: MavModeFlag::from_bits_truncate(MAV_MODE.load(Ordering::Relaxed)),
        custom_mode: 0,
        system_status: MavState::Uninit,
        mavlink_version: 0x3,
    };

    server.build_frame(&message)
}

pub(super) fn highres_imu(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {

    let mut message = HighresImu::default();
    message.time_usec = Instant::now().as_micros();

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

    server.build_frame(&message)
}

pub(crate) fn attitude_quaternion(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {

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

    server.build_frame(&message)
}

pub(crate) fn attitude(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {

    let mut message = Attitude::default();
    message.time_boot_ms = Instant::now().as_millis() as u32;

    if let Some(att) = crate::signals::AHRS_ATTITUDE.try_get() {
        [message.roll, message.pitch, message.yaw] = att;
    }

    if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
        [message.rollspeed, message.pitchspeed, message.yawspeed] = imu.gyr;
    }

    server.build_frame(&message)
}

pub(crate) fn servo_output_raw(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {

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

    server.build_frame(&message)
}

pub(crate) fn rc_channels_scaled(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {

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

    server.build_frame(&message)
}

pub(crate) fn autopilot_version(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {
    use mavio::dialects::common::messages::AutopilotVersion;

    // TODO fill this in correctly
    let message = AutopilotVersion::default();

    server.build_frame(&message)
}

// pub(super) fn pid_internals_rate(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
//     use mavlink::holsatus::PID_INTERNALS_RATE_DATA;
//     let mut message = PID_INTERNALS_RATE_DATA::default();

//     message.time_usec = Instant::now().as_micros();

//     if let Some([x,y,z]) = crate::signals::RATE_PID_TERMS.try_get() {
//         message.x_pid_p = x.p_out;
//         message.x_pid_i = x.i_out;
//         message.x_pid_dr = x.dr_out;
//         message.x_pid_dm = x.dm_out;

//         message.y_pid_p = y.p_out;
//         message.y_pid_i = y.i_out;
//         message.y_pid_dr = y.dr_out;
//         message.y_pid_dm = y.dm_out;

//         message.z_pid_p = z.p_out;
//         message.z_pid_i = z.i_out;
//         message.z_pid_dr = z.dr_out;
//         message.z_pid_dm = z.dm_out;
//     }

//     if let Some(comp_fuse) = crate::signals::COMP_FUSE_GYR.try_get() {
//         message.x_meas = comp_fuse[0];
//         message.y_meas = comp_fuse[1];
//         message.z_meas = comp_fuse[2];
//     }

//     if let Some(slew_rate) = crate::signals::SLEW_RATE_SP.try_get() {
//         message.x_ref = slew_rate[0];
//         message.y_ref = slew_rate[1];
//         message.z_ref = slew_rate[2];
//     }

//     if let Some(ff_pred) = crate::signals::FF_PRED_GYR.try_get() {
//         message.x_pred = ff_pred[0];
//         message.y_pred = ff_pred[1];
//         message.z_pred = ff_pred[2];
//     }

//     server.serialize(raw, message)
// }

pub(super) fn gps_raw_int(server: &MavServer, _params: &Params) -> Result<Frame<V2>, MavlinkError> {
    use nalgebra::Vector3;
    use crate::types::measurements::GnssFix;
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
            vel: (Vector3::new(gnss.vel_east, gnss.vel_north, gnss.vel_down).norm() * 100.) as u16,
            cog: (gnss.vel_east.atan2(gnss.vel_north).to_degrees() * 100.) as u16,
            fix_type,
            satellites_visible: gnss.satellites,
            .. Default::default()
        }
    } else {
        GpsRawInt {
            time_usec: Instant::now().as_micros(),
            fix_type: GpsFixType::NoGps,
            satellites_visible: 0,
            .. Default::default()
        }
    };

    server.build_frame(&message)
}
