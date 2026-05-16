//! Open Drone ID smoke test.
//!
//! Spawns only the ODID transmitter on UART5 PB6 plus a blinking blue LED
//! liveness indicator. Run with the Supermini powered and wired to UART5;
//! the module LED should turn solid green within ~5 s as all five
//! OPEN_DRONE_ID_* messages start arriving in fresh windows. Drone Scout on
//! iOS / Android then shows operator_id, uas_id, description, takeoff
//! location, and live position (currently the Kiel fallback).
//!
//! No motors are armed and no IMU is touched. Safe to run on a bench with a
//! battery connected.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;

use micoairh743v2::log as ulog;
use micoairh743v2::odid;
use micoairh743v2::resources;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());
    let r = resources::split(p);

    let mut led_blue = Output::new(r.leds.blue, Level::Low, Speed::Low);
    let _led_red     = Output::new(r.leds.red, Level::Low, Speed::Low);
    let _led_green   = Output::new(r.leds.green, Level::Low, Speed::Low);

    ulog::log("[odid_test] boot");
    ulog::log(concat!("[odid_test] git=", env!("GIT_SHA")));

    spawner.spawn(odid::odid_tx_task(r.odid).unwrap());

    loop {
        led_blue.set_high();
        Timer::after_millis(500).await;
        led_blue.set_low();
        Timer::after_millis(500).await;
    }
}
