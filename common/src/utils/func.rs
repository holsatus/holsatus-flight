use num_traits::float::Float;

/// Linearly map `num` from the range `[in_min, in_max]` to the range `[out_min, out_max]`
pub fn linear_map<T: Float>(num: T, in_min: T, in_max: T, out_min: T, out_max: T) -> T {
    let out_delta = out_max - out_min;
    let in_delta = in_max - in_min;
    ((num - in_min) / in_delta) * out_delta + out_min
}

/// Wrap the value of `num` such that it lies between `[min,max)` (i.e. min <= num < max)
pub fn wrap<T: Float>(mut num: T, min: T, max: T) -> T {
    assert!(min < max, "Invalid wrapping bounds");
    let width = max - min;

    while num < min {
        num = num + width;
    }
    while num >= max {
        num = num - width;
    }

    num
}

/// Helper function to get a fixed-size array at the start of an immutable slice
pub fn ref_array_start<const N: usize>(buf: &[u8]) -> Option<&[u8; N]> {
    buf.get(..N.min(buf.len()))?.try_into().ok()
}

/// Helper function to get a fixed-size array at the start of a mutable slice
pub fn mut_array_start<const N: usize>(buf: &mut [u8]) -> Option<&mut [u8; N]> {
    buf.get_mut(..N.min(buf.len()))?.try_into().ok()
}
