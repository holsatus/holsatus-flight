//! This test will succeed since the flight pattern will finish in 120 seconds

#[test]
fn flight_pattern() {
    let report = std_device::test_entry(60, "integration-test-flight-pattern").unwrap();

    assert!(report.report.mission_completed, "mission did not complete");
    assert!(!report.report.ground_collision, "ground collision detected");
    assert!(
        report.report.max_altitude_m > 9.0,
        "never reached 10m target altitude"
    );
    assert!(
        report.report.landing_speed_ms < 1.0,
        "landing speed too high: {:.2} m/s",
        report.report.landing_speed_ms
    );
    assert!(
        report.report.max_acceleration_ms2 < 20.0,
        "max acceleration to high: {:.2} m/s/s",
        report.report.max_acceleration_ms2
    );
}
