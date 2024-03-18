
use embassy_futures::select::{select, Either};

use crate::messaging as msg;
use crate::transmitter::{ChannelType, ControlRequest, EventRequest};

#[embassy_executor::task]
pub async fn rc_mapper() {
    // Input messages
    let mut rcv_sbus_data = msg::SBUS_DATA.receiver().unwrap();
    let mut rcv_cfg_transmitter_map = msg::CFG_TRANSMITTER_MAP.receiver().unwrap();

    // Output messages
    let snd_request_controls = msg::REQUEST_CONTROLS.sender();
    let snd_request_queue = msg::REQUEST_QUEUE.sender();

    // Await initial transmitter map (or loop until valid map is received)
    let mut transmitter_map = rcv_cfg_transmitter_map.changed().await;
    while let Err(e) = transmitter_map.sanity_check() {
        defmt::error!("[RC MAPPER]: Loaded an invalid transmitter from config: {}", e);
        transmitter_map = rcv_cfg_transmitter_map.changed().await;
    }

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

                use crate::transmitter::AnalogCommand::{Pitch, Roll, Thrust, Yaw};
                
                // Similar to sbus_packet.channels.iter().zip(transmitter_map.iter()).enumerate().for_each(|(i, (channel, map))| { ... }),
                // but we avoid using a closure, where we are not able to do an async send into the request queue channel.
                for ch in 0..sbus_packet.channels.len().min(transmitter_map.len()) {
                    match &transmitter_map[ch] {
                        ChannelType::None => {},
                        ChannelType::Analog((Roll, cfg)) => control.roll = cfg.apply(sbus_packet.channels[ch]),
                        ChannelType::Analog((Pitch, cfg)) => control.pitch = cfg.apply(sbus_packet.channels[ch]),
                        ChannelType::Analog((Yaw, cfg)) => control.yaw = cfg.apply(sbus_packet.channels[ch]),
                        ChannelType::Analog((Thrust, cfg)) => control.thrust = cfg.apply(sbus_packet.channels[ch]),
                        ChannelType::Discrete(bindings) => {
                            // Check each value-event binding pair for this channel
                            for &(value, event) in bindings {
                                if value == sbus_packet.channels[ch] {
                                    
                                    // If value is same as previous, skip
                                    if let Some(prev) = prev_sbus_packet 
                                    && prev.channels[ch] == sbus_packet.channels[ch] {
                                        continue;
                                    }
                                    
                                    // Log command if it has changed
                                    defmt::trace!("[RC MAPPER]: From channel {} - {} received discrete command: {:?}", ch, value, defmt::Debug2Format(&event));
                                    
                                    // If mapping is unbound, skip sending it
                                    if event == EventRequest::Unbound {
                                        continue;
                                    } 
                                    // Send command to request queue
                                    snd_request_queue.send(event.into()).await;
                                }
                            }
                        },
                    }
                }

                prev_sbus_packet = Some(sbus_packet);
                snd_request_controls.send(control);
            }

            // Failsafe, notify commander and reset prev_sbus_packet
            Either::First(Err(error)) => {
                use crate::t_sbus_reader::RxError as E;
                match error {

                    // We need to not reset prev_sbus_packet on E::SerialRead, as this
                    // error may occur naturally when the vehicle is on the ground.
                    E::SerialRead => { defmt::warn!("[RC MAPPER]: Received a serial read error"); },
                    E::SerialTimeout => prev_sbus_packet = None,
                    E::ParseTimeout => prev_sbus_packet = None,
                    E::Failsafe => prev_sbus_packet = None,
                };
                snd_request_queue.send(EventRequest::RcFailsafe.into()).await;
            }

            // Received new map, save it
            Either::Second(new_map) => {
                match transmitter_map.sanity_check() {
                    Ok(()) => transmitter_map = new_map,
                    Err(e) => defmt::warn!("[RC MAPPER]: Received an invalid transmitter map: {}", e),
                }
            },
        }
    }
}