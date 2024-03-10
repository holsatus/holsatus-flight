// TODO - investigate how to use the sequential-storage crate
// to log data to flash in a circular buffer in memory.

// Suggestion for logging contents
// Should also contain system state, such as arming status, and any error flags.
struct LogEntry {
    time_ms: u32,
    acc: Vector3<f32>,
    gyr: Vector3<f32>,
    mag: Vector3<f32>,
    euler: Vector3<f32>,
    setpoint: Vector4<f32>,
    ctrl_output: Vector3<f32>,
    motor_speeds: Vector4<f32>,
}

// Suggestion for logging header
// Header exists to ensure we can separate logs from different runs,
// And only overwrite the oldest logs when the buffer is full.
struct LogHeader {
    id: u32,
}