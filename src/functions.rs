use num_traits::float::Float;

/// Linearly map `num` from the range `[in_min, in_max]` to the range `[out_min, out_max]`
pub fn map<T: Float>(num: T, in_min: T, in_max: T, out_min: T, out_max: T) -> T {
    let out_delta = out_max - out_min;
    let in_delta = in_max - in_min;
    ((num - in_min) / in_delta) * out_delta + out_min
}

/// Wrap the value of `num` such that it lies between `[min,max)` (i.e. min <= num < max)
pub fn wrap<T: Float>(mut num: T, min: T, max: T) -> T {
    defmt::assert!(min < max, "Invalid wrapping bounds");
    let width = max - min;

    while num < min {
        num = num + width;
    }
    while num >= max {
        num = num - width;
    }

    num
}

/// Get the time since boot in ms, u32 representation.
/// This counter will overflow after ~50 days.
pub fn time_boot_ms() -> u32 {
    use embassy_time::Instant;

    // The 0xFFFFFFFF flag will ensure the cast to u32 will not clamp values > 2^32
    // but will instead make it behave as an overflow.
    (Instant::now().as_millis() & 0xFFFFFFFF) as u32
}
