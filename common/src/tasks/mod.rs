pub mod angle_loop;
pub mod arm_blocker;
pub mod att_estimator;
pub mod blackbox_fat;
pub mod blackbox_collector;
pub mod calibrator;
pub mod commander;
pub mod configurator;
pub mod gnss_reader;
pub mod imu_manager;
pub mod imu_reader;
pub mod motor_governor;
pub mod position_estimator;
pub mod rate_loop;
pub mod rc_reader;
pub mod signal_logger;
pub mod signal_router;
pub mod signal_stats;
pub mod usb_manager;
pub mod rc_mapper;
pub mod motor_test;

// Some more advanced task modules are hidden behind feature flags.

#[cfg(feature = "mavlink")]
pub mod mavlink {
    pub use crate::mavlink::MavServer;
}