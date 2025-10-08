use num_traits::Float;

/// Simple model to linearize the command -> thrust curve of a motor.
pub struct MotorLin<T: Float> {
    a: T,
    b: T,
    c_min: T,
    c_max: T,
    f_min: T,
    f_max: T,
}

impl<T: Float> MotorLin<T> {
    pub fn new(a: T, b: T, min: T, max: T) -> Self {
        let mut this = Self {
            a,
            b,
            c_min: min,
            c_max: max,
            f_min: T::zero(),
            f_max: T::zero(),
        };

        this.f_min = this.command_to_force(min);
        this.f_max = this.command_to_force(max);

        this
    }

    pub fn command_to_force(&self, command: T) -> T {
        let x = command.clamp(self.c_min, self.c_max);
        self.a * x * x / (T::one() + self.b * x * x)
    }

    pub fn force_to_command(&self, thrust: T) -> T {
        let x = thrust.clamp(self.f_min, self.f_max);
        (x / (self.a - x * self.b)).sqrt()
    }

    /// Map a thrust command from the range [0,1] to [0,1] with the inverse mapping function
    /// of `MotorLin::force_to_command`.
    pub fn command_linearized(&self, command: T) -> T {
        self.force_to_command(command * self.max_force())
    }

    pub fn min_force(&self) -> T {
        self.f_min
    }

    pub fn max_force(&self) -> T {
        self.f_max
    }
}
