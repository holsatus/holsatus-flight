///*
/// This file should be used as a template for new tasks 
///*/

use defmt::*;
use crate::channels;

static TASK_ID : &str = "[PROTOTYPE]";

#[embassy_executor::task]
pub async fn prototype(
    mut sub_attitude_sense: channels::SgAttitudeSense,
) {

    info!("{}: Entering main loop",TASK_ID);
    loop {

        // Place looping code here, never break out of this loop!

    }
}
