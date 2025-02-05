use embassy_futures::select::{select, Either};
use crate::{get_or_warn, rc_mapping::Binding, signals as s, types::control::RcAnalog};

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

    let mut rc_binding = get_or_warn!(rcv_rc_bindings).await;

    // Store previous packet to detect changes in digital channels
    let mut prev_packet = None;

    let mut rc_analog_unit = RcAnalog([0.0; 8]);
    let mut rc_analog_rate = RcAnalog([0.0; 8]);

    info!("{}: Entering main loop", ID);
    'infinite: loop {

        match select(
            rcv_rc_bindings.changed(),
            rcv_rc_channels.changed()
        ).await {

            // Received a new binding configuration
            Either::First(rc_bindings) => rc_binding = rc_bindings,

            // No new RC packet, continue to next iteration
            Either::Second(None) => continue 'infinite,

            // Received a new RC packet to map!
            Either::Second(Some(rc_channels)) => {

                // Filter out unbound channels and non-analog bindings
                let bound_channels = rc_channels.iter().zip(&rc_binding.0).enumerate()
                .filter_map(|(i, (rc_value, opt_binding))|
                    opt_binding.map(|binding| (i, (rc_value, binding))));

                // Iterate over all bound channels
                for (i, (rc_value, channel)) in bound_channels {
                    match channel {
                        Binding::Analog(analog) => {

                            // Scale raw channel to unit-sized value
                            let unit = if analog.fullrange {
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
                        },
                        Binding::Digital(digital) => {
                            // If value is same as previous, skip to avoid spamming,
                            // since events are supposed to signal a change.
                            if prev_packet.is_some_and(|p: [u16; 16]| p[i] == *rc_value) {
                                continue;
                            }

                            // Filter out unbound and non-matching channels
                            let bound = digital
                                .0
                                .iter()
                                .filter_map(|x| x.as_ref());

                            // Apply all events (multiple can be bound to same value)
                            for (_, event) in bound.filter(|y| y.0 == *rc_value) {
                                info!("{}: Digital event: {:?}", ID, event);
                                s::COMMANDER_REQUEST.send((*event).into()).await;
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
