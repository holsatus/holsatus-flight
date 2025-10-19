use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};

pub mod acc_routine;
pub mod gyr_routine;
pub mod mag_routine;
pub mod sens3d;

#[derive(Debug, Clone)]
pub enum Calibrate {
    Acc(AccCalib, Option<u8>),
    Gyr(GyrCalib, Option<u8>),
    Mag(MagCalib, Option<u8>),
}

#[derive(Debug, Clone)]
pub struct AccCalib {
    pub max_var: f32,
    pub max_dropped: usize,
}

impl Default for AccCalib {
    fn default() -> Self {
        Self {
            max_var: 0.01,
            max_dropped: 10,
        }
    }
}

#[derive(Debug, Clone)]
pub struct GyrCalib {
    pub max_var: f32,
    pub duration_s: u8,
    pub max_dropped: usize,
}

impl Default for GyrCalib {
    fn default() -> Self {
        Self {
            max_var: 0.01,
            duration_s: 5,
            max_dropped: 10,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MagCalib {
    pub duration_s: u8,
    pub pre_scalar: f32,
    pub std_limit: f32,
    pub max_dropped: usize,
}

pub struct Feedback<T> {
    sig: Signal<NoopRawMutex, Option<T>>,
}

pub struct FeedbackHandle<'a, T> {
    sig: &'a Signal<NoopRawMutex, Option<T>>,
}

impl<'a, T> Drop for FeedbackHandle<'a, T> {
    fn drop(&mut self) {
        self.sig.signal(None);
    }
}

impl<'a, T> FeedbackHandle<'a, T> {
    pub fn send(&self, data: T) {
        self.sig.signal(Some(data));
    }
}

impl<T> Feedback<T> {
    pub fn new() -> Self {
        Self { sig: Signal::new() }
    }

    pub fn handle(&self) -> FeedbackHandle<'_, T> {
        FeedbackHandle { sig: &self.sig }
    }

    pub async fn receive(&self) -> Option<T> {
        self.sig.wait().await
    }
}
