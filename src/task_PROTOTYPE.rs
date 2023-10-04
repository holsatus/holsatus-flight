///*
/// This file should be used as a template for new tasks 
///*/

use defmt::*;
use crate::channels;

static TASK_ID : &str = "[PROTOTYPE]";

#[embassy_executor::task]
pub async fn prototype(
    mut s_config_ref: channels::ConfigRefSub,
) {

    // Await system configuration reference
    let mut config = s_config_ref.next_message_pure().await;

    // Define task frequency
    let ticker = Ticker::after(Duration::from_hz(50));

    info!("{}: Entering main loop",TASK_ID);
    loop {

        // Get new reference to configuration. Only valid within 80 milliseconds!
        if let Some(new_config) = s_config_ref.try_next_message_pure() { config = new_config };

        // Place looping code here, never break out of this loop!

        // Wait for next time to run
        ticker.next().await;
    }
}
