pub mod angle_loop;
pub mod arm_blocker;
pub mod att_estimator;
pub mod blackbox_collector;
pub mod blackbox_fat;
pub mod calibrator;
pub mod commander;
// pub mod configurator;
pub mod configurator2;
pub mod eskf;
pub mod gnss_reader;
pub mod imu_manager;
pub mod imu_reader;
pub mod motor_governor;
pub mod motor_test;
pub mod rate_loop;
pub mod rc_mapper;
pub mod rc_reader;
pub mod signal_logger;
pub mod signal_router;
pub mod signal_stats;
pub mod usb_manager;

// Some more advanced task modules are hidden behind feature flags.

#[cfg(feature = "mavlink")]
pub mod mavlink {
    pub use crate::mavlink::MavServer;
}
