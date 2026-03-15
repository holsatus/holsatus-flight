//! This test will succeed since the flight pattern will finish in 120 seconds

#[test]
fn flight_pattern() {
    std_device::test_entry(60, "integration-test-flight-pattern").unwrap()
}
