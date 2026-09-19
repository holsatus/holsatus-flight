use embassy_time::{Duration, Instant, Ticker, Timer};

/// Number of times certain commands are repeated to ensure transmission.
const DEFAULT_REPEATS: usize = 10;

/// Minimum number of milliseconds to wait after issuing a beep command.
const BEEP_WAIT_MILLIS: u64 = 1000;

/// Group of 4 DShot compatible motor outputs.
pub trait DshotGroup {
    /// Send a batch of [`DshotPacket`]s to the DShot ESCs.
    ///
    /// To extract the raw value, use the [`DshotPacket::get_raw`] method.
    fn send_packets(&mut self, packets: [DshotPacket; 4]) -> impl Future<Output = ()>;

    /// Send batch of [`DshotPacket`]s to the DShot ESCs a number (default: 20) of times.
    fn send_packets_repeated(&mut self, packets: [DshotPacket; 4]) -> impl Future<Output = ()> {
        async move {
            let mut ticker = Ticker::every(Duration::from_millis(1));
            for _ in 0..DEFAULT_REPEATS {
                self.send_packets(packets).await;
                ticker.next().await;
            }
        }
    }

    /// Send a normalized command (0.0..=1.0) per motor.
    ///
    /// Note that sending 0.0 is NOT the same as calling `stop_motors`. Sending 0.0 is the lowest
    /// valid speed command, and the motors may not actually spin, but it is not safe to assume so.
    fn set_speeds(&mut self, speeds: [f32; 4]) -> impl Future<Output = ()> {
        let packets = speeds.map(|speed| {
            let speed = if speed.is_finite() {
                speed.clamp(0.0, 1.0)
            } else {
                0.0
            };
            DshotPacket::encode_payload((speed * 1999.0) as u16 + THROTTLE_MIN, false)
        });

        self.send_packets(packets)
    }

    /// Send a stop command to the motors.
    ///
    /// The motors do not spin when this command is sent. This is called continuously when soft-disarmed.
    fn stop_motors(&mut self) -> impl Future<Output = ()> {
        let packets = [DshotPacket::encode_command(DshotCommand::MotorStop, false); 4];

        self.send_packets(packets)
    }

    /// Send a SpinDirection{Reversed/Normal} command to the DShot ESCs
    fn set_reversed(&mut self, reversed: [bool; 4]) -> impl Future<Output = ()> {
        let packets = reversed.map(|reversed| {
            DshotPacket::encode_command(
                match reversed {
                    true => DshotCommand::SpinDirectionReversed,
                    false => DshotCommand::SpinDirectionNormal,
                },
                true,
            )
        });

        self.send_packets_repeated(packets)
    }

    /// Play one of 5 beeps through the motors.
    ///
    /// Note: This will not return for *at least* 1000 millis to give the tune time to finish.
    /// Internally keeps sending [`DshotGroup::stop_motors`] to prevent motors from disarming.
    fn play_beep(&mut self, tone: Beep) -> impl Future<Output = ()> {
        let packet = DshotPacket::encode_command(
            match tone {
                Beep::Kind1 => DshotCommand::Beep1,
                Beep::Kind2 => DshotCommand::Beep2,
                Beep::Kind3 => DshotCommand::Beep3,
                Beep::Kind4 => DshotCommand::Beep4,
                Beep::Kind5 => DshotCommand::Beep5,
            },
            true,
        );

        async move {
            self.send_packets([packet; 4]).await;

            // Important that we keep the ESC armed. It will still beep during this.
            let expire = Instant::now() + Duration::from_millis(BEEP_WAIT_MILLIS);
            while Instant::now() < expire {
                self.stop_motors().await;
                Timer::after_millis(10).await;
            }
        }
    }
}

pub enum Beep {
    Kind1,
    Kind2,
    Kind3,
    Kind4,
    Kind5,
}

/// Minimum allowed DShot throttle value
pub const THROTTLE_MIN: u16 = 48;

/// Maximum allowed DShot throttle value
pub const THROTTLE_MAX: u16 = 2047;

#[derive(Clone, Copy)]
#[repr(u16)]
pub enum DshotCommand {
    /// Command all motors to stop spinning
    MotorStop,
    /// Wait at least length of beep (260ms) before next command
    Beep1,
    /// Wait at least length of beep (260ms) before next command
    Beep2,
    /// Wait at least length of beep (280ms) before next command
    Beep3,
    /// Wait at least length of beep (280ms) before next command
    Beep4,
    /// Wait at least length of beep (1020ms) before next command
    Beep5,
    /// Wait at least 12ms before next command
    EscInfo,
    /// Need 6x, no wait required
    SpinDirection1,
    /// Need 6x, no wait required
    SpinDirection2,
    /// Need 6x, no wait required
    Mode3DOff,
    /// Need 6x, no wait required
    Mode3DOn,
    /// Currently not implemented
    SettingsRequest,
    /// Need 6x, wait at least 35ms before next command
    SaveSettings,
    /// Need 6x, no wait required
    SpinDirectionNormal = 20,
    /// Need 6x, no wait required
    SpinDirectionReversed,
    /// No wait required
    Led0On,
    /// No wait required
    Led1On,
    /// No wait required
    Led2On,
    /// No wait required
    Led3On,
    /// No wait required
    Led0Off,
    /// No wait required
    Led1Off,
    /// No wait required
    Led2Off,
    /// No wait required
    Led3Off,
}

#[derive(Clone, Copy)]
pub struct DshotPacket(u16);

impl DshotPacket {
    /// Calculate the CRC checksum for a packet
    const fn calc_checksum(raw: u16, telemetry: bool) -> u16 {
        // Concatenate throttle value with telemetry request
        let packet = (raw << 1) | (telemetry as u16);

        // Calculate and return checksum
        (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F
    }

    pub const fn encode_payload(mut payload: u16, telemetry: bool) -> DshotPacket {
        // Clamp value rather than shift out invalid bits
        if payload > THROTTLE_MAX {
            payload = THROTTLE_MAX;
        }

        // Compute checksum of payload
        let checksum = Self::calc_checksum(payload, telemetry);

        // Assemble packet
        DshotPacket(payload << 5 | (telemetry as u16) << 4 | checksum)
    }

    pub const fn encode_command(command: DshotCommand, telemetry: bool) -> DshotPacket {
        Self::encode_payload(command as u16, telemetry)
    }

    pub const fn get_raw(&self) -> u16 {
        self.0
    }

    pub const fn as_speed(&self) -> Option<u16> {
        let payload = self.0 >> 5;
        if payload < THROTTLE_MIN || payload > THROTTLE_MAX {
            None
        } else {
            Some(payload)
        }
    }
}
