//! This test will succeed since the flight pattern will finish in 120 seconds

#[test]
fn flight_pattern() {
    let rerun = std::env::var("HOLSATUS_TEST_RERUN").is_ok_and(|key| key == "1");
    std_device::test_entry(60, "integration-test-flight-pattern", rerun).unwrap()
}
