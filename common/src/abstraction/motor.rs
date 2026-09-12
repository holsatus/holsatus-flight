pub trait MotorGroup {
    fn set_motor_speeds(&mut self, speeds: [u16; 4]) -> impl Future<Output = ()>;
    fn set_motor_speeds_min(&mut self) -> impl Future<Output = ()>;
    fn set_reverse_dir(&mut self, rev: [bool; 4]) -> impl Future<Output = ()>;
    fn make_beep(&mut self) -> impl Future<Output = ()>;
}
