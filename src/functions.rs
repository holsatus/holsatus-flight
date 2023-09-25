use num_traits::float::Float;

pub fn map<T: Float>(num: T, in_min: T, in_max: T, out_min: T, out_max: T) -> T {
    let out_delta = out_max - out_min;
    let in_delta = in_max - in_min;
    ((num - in_min) / in_delta) * out_delta + out_min
}

pub fn wrap<T: Float>(num: T, min: T, max: T) -> T {
    let frac = ((num - min) / (max - min)).floor();
    num - frac * (max - min)
}

/// Compare the variant (ignoring the values) of two enum instances
pub fn variant_eq<T>(a: &T, b: &T) -> bool {
    core::mem::discriminant(a) == core::mem::discriminant(b)
}
