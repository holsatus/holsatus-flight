use bitflags::bitflags;
bitflags! {
    /// Mavlink system status sensor flags.
    pub struct MavSysStatusSensorFlags: u32 {
        /// 3D gyro.
        const MAV_SYS_STATUS_SENSOR_3D_GYRO = 0b00000001;
        /// 3D accelerometer.
        const MAV_SYS_STATUS_SENSOR_3D_ACCEL = 0b00000010;
        /// 3D magnetometer.
        const MAV_SYS_STATUS_SENSOR_3D_MAG = 0b00000100;
        /// Absolute pressure.
        const MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 0b00001000;
        /// Differential pressure.
        const MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 0b00010000;
        /// GPS.
        const MAV_SYS_STATUS_SENSOR_GPS = 0b00100000;
        /// Optical flow.
        const MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 0b01000000;
        /// Computer vision position.
        const MAV_SYS_STATUS_SENSOR_VISION_POSITION = 0b10000000;
        /// Laser based position.
        const MAV_SYS_STATUS_SENSOR_LASER_POSITION = 0b00000001_00000000;
        /// External ground truth (Vicon or Leica).
        const MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 0b00000010_00000000;
        /// 3D angular rate control.
        const MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 0b00000100_00000000;
        /// Attitude stabilization.
        const MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 0b00001000_00000000;
        /// Yaw position.
        const MAV_SYS_STATUS_SENSOR_YAW_POSITION = 0b00010000_00000000;
        /// Z/altitude control.
        const MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 0b00100000_00000000;
        /// X/Y position control.
        const MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 0b01000000_00000000;
        /// Motor outputs / control.
        const MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 0b10000000_00000000;
        /// RC receiver.
        const MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 0b00000001_00000000_00000000;
        /// 2nd 3D gyro.
        const MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 0b00000010_00000000_00000000;
        /// 2nd 3D accelerometer.
        const MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 0b00000100_00000000_00000000;
        /// 2nd 3D magnetometer.
        const MAV_SYS_STATUS_SENSOR_3D_MAG2 = 0b00001000_00000000_00000000;
        /// Geofence.
        const MAV_SYS_STATUS_GEOFENCE = 0b00010000_00000000_00000000;
        /// AHRS subsystem health.
        const MAV_SYS_STATUS_AHRS = 0b00100000_00000000_00000000;
        /// Terrain subsystem health.
        const MAV_SYS_STATUS_TERRAIN = 0b01000000_00000000_00000000;
        /// Motors are reversed.
        const MAV_SYS_STATUS_REVERSE_MOTOR = 0b10000000_00000000_00000000;
        /// Logging.
        const MAV_SYS_STATUS_LOGGING = 0b00000001_00000000_00000000_00000000;
        /// Battery.
        const MAV_SYS_STATUS_SENSOR_BATTERY = 0b00000010_00000000_00000000_00000000;
        /// Proximity.
        const MAV_SYS_STATUS_SENSOR_PROXIMITY = 0b00000100_00000000_00000000_00000000;
        /// Satellite Communication.
        const MAV_SYS_STATUS_SENSOR_SATCOM = 0b00001000_00000000_00000000_00000000;
        /// Pre-arm check status. Always healthy when armed.
        const MAV_SYS_STATUS_PREARM_CHECK = 0b00010000_00000000_00000000_00000000;
        /// Avoidance/collision prevention.
        const MAV_SYS_STATUS_OBSTACLE_AVOIDANCE = 0b00100000_00000000_00000000_00000000;
        /// Propulsion (actuator, esc, motor or propellor).
        const MAV_SYS_STATUS_SENSOR_PROPULSION = 0b01000000_00000000_00000000_00000000;
        /// Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present only).
        const MAV_SYS_STATUS_EXTENSION_USED = 0b10000000_00000000_00000000_00000000;
    }
}
