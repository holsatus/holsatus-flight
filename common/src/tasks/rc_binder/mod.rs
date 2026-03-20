use crate::{
    signals as s,
    tasks::{commander, rc_binder::params::analog},
    types::control::RcAnalog,
};

pub mod rates;

pub mod params;

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "rc_mapper";
    info!("{}: Task started", ID);

    // Input signals
    let mut rcv_rc_channels = s::RC_CHANNELS_RAW.receiver();

    // Output signals
    let mut snd_rc_controls_unit = s::RC_ANALOG_UNIT.sender();

    let params = params::TABLE.read().await;
    let rc_bindings = params.channel_binding.clone();

    drop(params);

    // Store previous packet to detect changes in digital channels
    let mut prev_rc_channels = None;

    let mut rc_analog = RcAnalog([0.0; 8]);

    info!("{}: Entering main loop", ID);
    'infinite: loop {
        let Some(rc_channels) = rcv_rc_channels.changed().await else {
            continue 'infinite;
        };

        for index in 0..params::NUM_CHANNELS {
            let rc_value = rc_channels[index];

            match &rc_bindings.0[index] {
                params::Binding::None => continue,
                params::Binding::Analog(analog) => {
                    if analog.axis == analog::Axis::None {
                        continue;
                    }

                    // Scale raw channel to unit-sized value
                    let unit_value = analog.map(rc_value);

                    // Update the result
                    rc_analog.0[analog.axis as u8 as usize] = unit_value;
                }
                params::Binding::Digital(digital) => {
                    // If value is same as previous, skip to avoid spamming
                    if prev_rc_channels.is_some_and(|rc: [u16; 16]| rc[index] == rc_value) {
                        continue;
                    }

                    // Propagate all all bound events
                    for bind in digital {
                        if rc_value != bind.value {
                            continue;
                        }

                        let Ok(command) = (bind.event).try_into() else {
                            continue;
                        };

                        info!("{}: Sending digital event: {:?}", ID, bind.event);
                        commander::PROCEDURE
                            .send(commander::Request {
                                command,
                                origin: commander::Origin::RemoteControl,
                            })
                            .await;
                    }
                }
            }
        }

        // Save the packet for next iteration and transmit analog values
        prev_rc_channels = Some(rc_channels);
        snd_rc_controls_unit.send(rc_analog);
    }
}
