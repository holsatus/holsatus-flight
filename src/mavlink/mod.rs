use embassy_rp::{uart::{Uart, Async}, peripherals::UART0};
use embassy_sync::pubsub::PubSubChannel;
use mavlink::{self,common::MavMessage, MavHeader};

use crate::{channels::*, config::definitions::Configuration};

const MAVLINK_MSG_LEN: usize = 25;
const MAVLINK_MSG_PUBS: usize = 10;
pub type MavlinkMsgPub = PubQ< MavMessage, MAVLINK_MSG_LEN, MAVLINK_MSG_PUBS >;
pub type MavlinkMsgSub = SubQ< MavMessage, MAVLINK_MSG_LEN, MAVLINK_MSG_PUBS >;
pub static MAVLINK_MSG: ChQ< MavMessage, MAVLINK_MSG_LEN, MAVLINK_MSG_PUBS > = PubSubChannel::new();

use embassy_executor::Spawner;
use embassy_time::Duration;

use defmt::*;
mod heartbeat;

static TASK_ID : &str = "MAVLINK-SERVER";

pub type MavlinkUart = Uart<'static,UART0,Async>;

#[embassy_executor::task]
pub async fn mavlink_task(
    spawner : Spawner,
    config: &'static Configuration,
    uart_mavlink : MavlinkUart,
) {

    let (mut tx,_rx) = uart_mavlink.split();

    let mut header = MavHeader {
        system_id: 1,       // ID of system (eg. for swarms/fleets)
        component_id: 1,    // ID of component: 1 is from autopilot
        sequence: 0,
    };

    // Grab subscriber to mavlink messaging channel
    let mut subscriber: MavlinkMsgSub = MAVLINK_MSG.subscriber()
        .expect("[MAVLINK-SERVER]: Unable to aquire subscriber to Mavlink message queue");

    // Spawn heartbeat microservice
    spawner.must_spawn(heartbeat::mav_heartbeat(
        heartbeat::INTERVAL.subscriber().unwrap(),
        MAVLINK_MSG.publisher().unwrap()
    ));

    let heartbeat_interval: Pub<Option<Duration>,1> = unwrap!(heartbeat::INTERVAL.publisher());   

    heartbeat_interval.publish_immediate(config.mav_freq.heartbeat); 
    info!("[{}]: Entering main loop",TASK_ID);
    loop {

        // Retrieve message from buffer
        let message = subscriber.next_message_pure().await;
        
        // TODO Handle failed transmissions
        let _ = mavlink::write_v2_msg(&mut tx, header, &message);
        
        // Increment message sequence counter
        header.sequence = header.sequence.wrapping_add(1);
    }
}