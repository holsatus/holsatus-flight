use super::super::{params::TABLE, Message, CHANNEL};
use crate::mavlink2::{messages::Generator, Target};
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn service_heartbeat() -> ! {
    let dur_ms = TABLE.read().await.hb_dur_ms;
    let mut ticker = Ticker::every(Duration::from_millis(dur_ms as u64));

    loop {
        CHANNEL
            .send(Message::SendGenerator {
                generator: Generator::Heartbeat,
                target: Target::Broadcast,
            })
            .await;

        ticker.next().await;
    }
}
