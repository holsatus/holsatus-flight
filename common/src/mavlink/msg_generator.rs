use core::sync::atomic::Ordering;

use embassy_time::Instant;
use mavlink::{holsatus::*, MAVLinkV2MessageRaw, MessageData};
use num_traits::FromPrimitive;

use crate::errors::MavlinkError;
use super::{MavServer, MAV_MODE, MAV_STATE};

macro_rules! define_sendable {
    (enum $name:ident { $($variant:ident($data_id:ident @ $gen_fn:ident)),* $(,)?}) => {
        #[repr(u32)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum $name {
            $($variant = $data_id::ID),*
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
                    $($data_id::ID => Ok($name::$variant)),*,
                    _ => Err(MavlinkError::NoMessageHandler),
                }
            }
        }
    };
}

define_sendable!(
    enum MavSendable {
        Heartbeat(HEARTBEAT_DATA @ heartbeat),
        HighresImu(HIGHRES_IMU_DATA @ highres_imu),
        AttitudeQuaternion(ATTITUDE_QUATERNION_DATA @ attitude_quaternion),
        Attitude(ATTITUDE_DATA @ attitude),
        ServoOutputRaw(SERVO_OUTPUT_RAW_DATA @ servo_output_raw),
        RcChannelsScaled(RC_CHANNELS_SCALED_DATA @ rc_channels_scaled),
        PidInternalsRate(PID_INTERNALS_RATE_DATA @ pid_internals_rate),
        GpsRawInt(GPS_RAW_INT_DATA @ gps_raw_int),
    }
);

pub(super) fn heartbeat(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::{HEARTBEAT_DATA, MavState, MavType, MavAutopilot, MavModeFlag};
    
    // TODO: MavType and MavAutopilot should be configurable
    let message = HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::from_bits_truncate(MAV_MODE.load(Ordering::Relaxed)),
        system_status: MavState::from_usize(MAV_STATE.load(Ordering::Relaxed) as usize).unwrap(),
        mavlink_version: 0x3,
    };

    server.serialize(raw, message)
}

pub(super) fn highres_imu(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::HIGHRES_IMU_DATA;

    let mut message = HIGHRES_IMU_DATA::default();
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

    server.serialize(raw, message)
}

pub(crate) fn attitude_quaternion(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::ATTITUDE_QUATERNION_DATA;

    let mut message = ATTITUDE_QUATERNION_DATA::default();
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

    server.serialize(raw, message)
}

pub(crate) fn attitude(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {

    use mavlink::holsatus::ATTITUDE_DATA;

    let mut message = ATTITUDE_DATA::default();
    message.time_boot_ms = Instant::now().as_millis() as u32;

    if let Some(att) = crate::signals::AHRS_ATTITUDE.try_get() {
        [message.roll, message.pitch, message.yaw] = att;
    }

    if let Some(imu) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get() {
        [message.rollspeed, message.pitchspeed, message.yawspeed] = imu.gyr;
    }

    server.serialize(raw, message)
}

pub(crate) fn servo_output_raw(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::SERVO_OUTPUT_RAW_DATA;

    let mut message = SERVO_OUTPUT_RAW_DATA::default();
    // Note, 32-bit micro-seconds does not make much sense, so we use millis instead
    message.time_usec = (0xFFFFFFFF & Instant::now().as_micros()) as u32;

    if let Some(speeds) = crate::signals::MOTORS_STATE.try_get() {
        let speeds = speeds.as_speeds();
        message.servo1_raw = speeds[0];
        message.servo2_raw = speeds[1];
        message.servo3_raw = speeds[2];
        message.servo4_raw = speeds[3];
    }

    server.serialize(raw, message)
}

pub(crate) fn rc_channels_scaled(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::RC_CHANNELS_SCALED_DATA;

    let mut message = RC_CHANNELS_SCALED_DATA::default();

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

    server.serialize(raw, message)
}

pub(super) fn pid_internals_rate(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::PID_INTERNALS_RATE_DATA;
    let mut message = PID_INTERNALS_RATE_DATA::default();

    message.time_usec = Instant::now().as_micros();

    if let Some([x,y,z]) = crate::signals::RATE_PID_TERMS.try_get() {
        message.x_pid_p = x.p_out;
        message.x_pid_i = x.i_out;
        message.x_pid_dr = x.dr_out;
        message.x_pid_dm = x.dm_out;

        message.y_pid_p = y.p_out;
        message.y_pid_i = y.i_out;
        message.y_pid_dr = y.dr_out;
        message.y_pid_dm = y.dm_out;

        message.z_pid_p = z.p_out;
        message.z_pid_i = z.i_out;
        message.z_pid_dr = z.dr_out;
        message.z_pid_dm = z.dm_out;
    }

    if let Some(comp_fuse) = crate::signals::COMP_FUSE_GYR.try_get() {
        message.x_meas = comp_fuse[0];
        message.y_meas = comp_fuse[1];
        message.z_meas = comp_fuse[2];
    }
    
    if let Some(slew_rate) = crate::signals::SLEW_RATE_SP.try_get() {
        message.x_ref = slew_rate[0];
        message.y_ref = slew_rate[1];
        message.z_ref = slew_rate[2];
    }

    if let Some(ff_pred) = crate::signals::FF_PRED_GYR.try_get() {
        message.x_pred = ff_pred[0];
        message.y_pred = ff_pred[1];
        message.z_pred = ff_pred[2];
    }

    server.serialize(raw, message)
}

pub(super) fn gps_raw_int(server: &MavServer, raw: &mut MAVLinkV2MessageRaw) {
    use mavlink::holsatus::{GPS_RAW_INT_DATA, GpsFixType};
    use nalgebra::Vector3;
    use crate::types::measurements::GnssFix;
    #[cfg(not(feature = "arch-std"))]
    use num_traits::Float;

    let message = if let Some(gnss) = crate::signals::RAW_GNSS_DATA.try_get() {

        let fix_type = match gnss.fix {
            GnssFix::NoFix => GpsFixType::GPS_FIX_TYPE_NO_FIX,
            GnssFix::Fix2D => GpsFixType::GPS_FIX_TYPE_2D_FIX,
            GnssFix::Fix3D => GpsFixType::GPS_FIX_TYPE_3D_FIX,
            GnssFix::TimeOnly => GpsFixType::GPS_FIX_TYPE_STATIC,
        };

        GPS_RAW_INT_DATA {
            time_usec: Instant::now().as_micros(),
            lat: gnss.lat_raw,
            lon: gnss.lon_raw,
            alt: (gnss.altitude * 1e3) as i32,
            eph: gnss.accuracy_east as u16,
            epv: gnss.accuracy_north as u16,
            vel: (Vector3::new(gnss.vel_east, gnss.vel_north, gnss.vel_down).norm() * 100.) as u16,
            cog: (gnss.vel_east.atan2(gnss.vel_north).to_degrees() * 100.) as u16,
            fix_type,
            satellites_visible: gnss.sats,
        }
    } else {
        GPS_RAW_INT_DATA {
            time_usec: Instant::now().as_micros(),
            fix_type: GpsFixType::GPS_FIX_TYPE_NO_GPS,
            satellites_visible: 0,
            .. Default::default()
        }
    };

    server.serialize(raw, message)
}
