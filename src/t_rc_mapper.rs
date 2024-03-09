
use embassy_futures::select::{select, Either};

use crate::messaging as msg;
use crate::transmitter::AnalogCommand::{Pitch, Roll, Thrust, Yaw};
use crate::transmitter::{analog_map, ChannelType, ControlRequest, EventRequest};

#[embassy_executor::task]
pub async fn rc_mapper() {
    // Input messages
    let mut rcv_sbus_data = msg::SBUS_DATA.receiver().unwrap();
    let mut rcv_cfg_transmitter_map = msg::CFG_TRANSMITTER_MAP.receiver().unwrap();

    // Output messages
    let snd_request_controls = msg::REQUEST_CONTROLS.sender();
    let snd_request_queue = msg::REQUEST_QUEUE.sender();

    // Await initial transmitter map
    let mut transmitter_map = rcv_cfg_transmitter_map.changed().await;
    let mut prev_sbus_packet: Option<sbus::SBusPacket> = None;
    let mut control = ControlRequest {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        thrust: 0.0,
    };

    '_infinite: loop {
        match select(rcv_sbus_data.changed(), rcv_cfg_transmitter_map.changed()).await {
            
            // Received new sbus packet, map it
            Either::First(Ok(sbus_packet)) => {

                // Similar to sbus_packet.channels.iter().zip(transmitter_map.iter()).enumerate().for_each(|(i, (channel, map))| { ... }),
                // but we avoid using a closure, where we are not able to do an async send into the request queue channel.
                for i in 0..sbus_packet.channels.len().min(transmitter_map.len()) {
                    match &transmitter_map[i] {
                        ChannelType::None => {},
                        ChannelType::Analog((Roll,cfg)) => control.roll = analog_map(sbus_packet.channels[i], cfg),
                        ChannelType::Analog((Pitch,cfg)) => control.pitch = analog_map(sbus_packet.channels[i], cfg),
                        ChannelType::Analog((Yaw,cfg)) => control.yaw = analog_map(sbus_packet.channels[i], cfg),
                        ChannelType::Analog((Thrust,cfg)) => control.thrust = analog_map(sbus_packet.channels[i], cfg),
                        ChannelType::Discrete(positions) => {
                            for &(pos, cmd) in positions {
                                if pos == sbus_packet.channels[i] {
                                    
                                    // If value is same as previous, skip
                                    if let Some(prev) = prev_sbus_packet && prev.channels[i] == sbus_packet.channels[i] {
                                        continue;
                                    }

                                    // If mapping is unbound, skip
                                    if cmd == EventRequest::Unbound {
                                        continue;
                                    } 

                                    // Log command if bound
                                    defmt::info!("[RC MAPPER]: From channel {} - {} received discrete command: {:?}", i, pos, defmt::Debug2Format(&cmd));
                                    
                                    // Send command to request queue
                                    snd_request_queue.send(cmd).await;
                                }
                            }
                        },
                    }
                }

                prev_sbus_packet = Some(sbus_packet);
                snd_request_controls.send(control);
            }

            // Failsafe, notify commander and reset prev_sbus_packet
            Either::First(Err(_)) => {
                prev_sbus_packet = None;
                snd_request_queue.send(EventRequest::RcFailsafe).await;
            }

            // Received new map, save it
            Either::Second(new_map) => transmitter_map = new_map,
        }
    }
}