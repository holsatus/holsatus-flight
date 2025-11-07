use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ArmingBlocker(u16);

bitflags::bitflags! {
    /// This bitflag represents the possible reasons why the vehicle cannot be armed.
    /// The bitflag is supposed to be `0x0000` when the vehicle is ready to be armed,
    /// which can be checked with the `is_empty()` method.
    // #[cfg_attr(feature = "defmt", derive(defmt::Format))] // Does not work :(
    impl ArmingBlocker: u16 {

        /// **Bit 0** - The gyroscope is not calibrated.
        const NO_GYR_CALIB  = 1 << 0;

        /// **Bit 1** - The accelerometer is not calibrated.
        const NO_ACC_CALIB  = 1 << 1;

        /// **Bit 2** - reserved.
        const RESERVED   = 1 << 2;

        /// **Bit 3** - The vehicle is currently undergoing sensor calibration.
        const CALIBRATING   = 1 << 3;

        /// **Bit 4** - No gyroscope data available.
        const NO_GYR_DATA   = 1 << 4;

        /// **Bit 5** - No accelerometer data available.
        const NO_ACC_DATA   = 1 << 5;

        /// **Bit 6** - The vehicle is at a higher throttle than allowed.
        const HIGH_THROTTLE_CMD = 1 << 6;

        /// **Bit 7** - The vehicle is at a higher attitude than allowed.
        const HIGH_ATTITUDE_CMD = 1 << 7;

        /// **Bit 8** - The vehicle attitude is at an abnormally high angle.
        const HIGH_ATTITUDE = 1 << 8;

        /// **Bit 9** - The vehicle was armed too soon after boot.
        const BOOT_GRACE    = 1 << 9;

        /// **Bit 10** - The system load it too high (low loop frequency)
        const SYSTEM_LOAD   = 1 << 10;

        /// **Bit 11** - The receiver is in failsafe mode.
        const RX_FAILSAFE   = 1 << 11;

        /// **Bit 12** - The flight controller has established a USB connection.
        const USB_CONNECTED = 1 << 12;

        /// **Bit 13** - The ESKF estimator is not producing reliable values.
        const ESKF_BAD_ESTIMATE = 1 << 12;
    }
}

#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidTerms {
    pub p_out: f32,
    pub i_out: f32,
    pub dr_out: f32,
    pub dm_out: f32,
}

#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct RcStatus {
    pub failsafe: bool,
    pub quality: u8,
}
