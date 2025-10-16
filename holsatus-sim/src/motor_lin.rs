/// Simple model to linearize the command -> thrust curve of a motor.
pub struct MotorLin {
    a: f32,
    b: f32,
    c_min: f32,
    c_max: f32,
    f_min: f32,
    f_max: f32,
}

impl MotorLin {
    pub fn new(a: f32, b: f32, min :f32, max: f32) -> Self {
        let mut this = Self {
            a,
            b,
            c_min: min,
            c_max: max,
            f_min: 0.0,
            f_max: 0.0,
        };

        this.f_min = this.command_to_force(min);
        this.f_max = this.command_to_force(max);

        this
    }

    pub fn command_to_force(&self, command: f32) -> f32 {
        let x = command.clamp(self.c_min, self.c_max);
        self.a*x*x/(1.0 + self.b*x*x)
    }

    pub fn force_to_command(&self, thrust: f32) -> f32 {
        let x = thrust.clamp(self.f_min, self.f_max);
        (x/(self.a - x*self.b)).sqrt()
    }

    /// Map a thrust command from the range [0,1] to [0,1] with the inverse mapping function
    /// of `MotorLin::force_to_command`.
    pub fn command_linearized(&self, command: f32) -> f32 {
        self.force_to_command(command * self.max_force())
    }

    pub fn min_force(&self) -> f32 {
        self.f_min
    }

    pub fn max_force(&self) -> f32 {
        self.f_max
    }
}