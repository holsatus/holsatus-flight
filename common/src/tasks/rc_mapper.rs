use crate::{
    get_or_warn,
    rc_mapping::{analog::AnalogFlags, Binding, RcBindings},
    signals as s,
    tasks::commander,
    types::control::RcAnalog,
};
use embassy_futures::select::{select, Either};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "rc_mapper";
    info!("{}: Task started", ID);

    // Input signals
    let mut rcv_rc_bindings = s::CFG_RC_BINDINGS.receiver();
    let mut rcv_rc_channels = s::RC_CHANNELS_RAW.receiver();

    // Output signals
    let mut snd_rc_controls_unit = s::RC_ANALOG_UNIT.sender();
    let mut snd_rc_controls_rate = s::RC_ANALOG_RATE.sender();

    let mut rc_binding =
        crate::tasks::configurator2::load_or_default::<RcBindings>("rc_mapper").await;

    // Store previous packet to detect changes in digital channels
    let mut prev_packet = None;

    let mut rc_analog_unit = RcAnalog([0.0; 8]);
    let mut rc_analog_rate = RcAnalog([0.0; 8]);

    info!("{}: Entering main loop", ID);
    'infinite: loop {
        match select(rcv_rc_bindings.changed(), rcv_rc_channels.changed()).await {
            // Received a new binding configuration
            Either::First(rc_bindings) => rc_binding = rc_bindings,

            // No new RC packet, continue to next iteration
            Either::Second(None) => continue 'infinite,

            // Received a new RC packet to map!
            Either::Second(Some(rc_channels)) => {
                // Filter out unbound channels and non-analog bindings
                let bound_channels = rc_channels
                    .iter()
                    .zip(&rc_binding.0)
                    .enumerate()
                    .filter_map(|(i, (rc_value, opt_binding))| {
                        opt_binding.as_ref().map(|binding| (i, (rc_value, binding)))
                    });

                // let analogs = bound_channels.clone().filter_map(|(_, (_, channel))| match channel {
                //     Binding::Analog(analog) => Some(analog),
                //     _ => None
                // });

                // let rc_events = bound_channels.filter_map(|(i, (rc_value, channel))| {
                //     let Binding::Digital(digital) = channel else {
                //         return None;
                //     };

                //     // Filter out all unchanged values
                //     if prev_packet.is_some_and(|p: [u16; 16]| p[i] == *rc_value) {
                //         return None;
                //     }

                //     // Filter out unbound and non-matching channels
                //     let bound = digital.0.iter().filter_map(move |x| match x.as_ref() {
                //         Some((rc_val, rc_event)) if rc_val == rc_value => Some(*rc_event),
                //         _ => None,
                //     });

                //     Some(bound)
                // });

                // let dingus = rc_events.flatten();

                // Iterate over all bound channels
                for (i, (rc_value, channel)) in bound_channels {
                    match channel {
                        Binding::Analog(analog) => {
                            // Scale raw channel to unit-sized value
                            let unit = if analog.flags.contains(AnalogFlags::FULLRANGE) {
                                analog.map_full_range(*rc_value)
                            } else {
                                analog.map_half_range(*rc_value)
                            };

                            // Apply rate (e.g. expo, linear) mapping
                            use crate::rc_mapping::analog::RatesTrait;
                            let rate = analog.rates.apply(unit);

                            // Update the result
                            rc_analog_unit.0[analog.axis as usize] = unit;
                            rc_analog_rate.0[analog.axis as usize] = rate;
                        }
                        Binding::Digital(digital) => {
                            // If value is same as previous, skip to avoid spamming,
                            // since events are supposed to signal a change.
                            if prev_packet.is_some_and(|p: [u16; 16]| p[i] == *rc_value) {
                                continue;
                            }

                            // Filter out unbound and non-matching channels
                            let bound = digital.0.iter().filter_map(|x| x.as_ref());

                            // Apply all events (multiple can be bound to same value)
                            for (_, event) in bound.filter(|y| y.0 == *rc_value) {

                                let Ok(command) = (*event).try_into() else {
                                    continue;
                                };

                                info!("{}: Digital event: {:?}", ID, event);
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
                prev_packet = Some(rc_channels);
                snd_rc_controls_unit.send(rc_analog_unit);
                snd_rc_controls_rate.send(rc_analog_rate);
            }
        }
    }
}
