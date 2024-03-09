use embassy_time::{Duration, Instant};

pub struct LoopHealth {
    expected_frequency: u16,
    health: f32,
    avg_frequency: f32,
    curr_loop_count: u64,
    prev_loop_count: u64,
    loop_time: Instant,
    shortest: Duration,
    longest: Duration,
    prev_loop_time: Instant,
}

impl LoopHealth {
    pub fn new(expected_frequency: u16) -> Self {
        Self {
            expected_frequency,
            health: 0.0f32,
            avg_frequency: f32::NAN,
            prev_loop_count: 0,
            curr_loop_count: 0,
            loop_time: Instant::now(),
            shortest: Duration::MAX,
            longest: Duration::MIN,
            prev_loop_time: Instant::now(),
        }
    }

    pub fn evaluate(&mut self) -> bool {
        self.curr_loop_count += 1;

        // Detect shortest and longest loop times
        let loop_time = self.prev_loop_time.elapsed();
        if loop_time < self.shortest {
            self.shortest = loop_time;
        } else if loop_time > self.longest {
            self.longest = loop_time;
        }
        self.prev_loop_time = Instant::now();

        let hz = 5;

        // Every second calculate and print the frequency
        if self.loop_time < Instant::now() {
            let frequency = hz * (self.curr_loop_count - self.prev_loop_count) as u16;
            if self.avg_frequency.is_normal() {
                self.avg_frequency = self.avg_frequency * 0.5 + (frequency as f32) * 0.5;
            } else {
                self.avg_frequency = frequency as f32;
            }
            self.prev_loop_count = self.curr_loop_count;
            self.loop_time += Duration::from_hz(hz as u64);

            self.health = self.avg_frequency as f32 / self.expected_frequency as f32;

            self.shortest = Duration::MAX;
            self.longest = Duration::MIN;

            true
        } else {
            false
        }
    }

    pub fn get_frequency(&self) -> f32 {
        self.avg_frequency
    }

    pub fn get_health(&self) -> f32 {
        self.health
    }
}
