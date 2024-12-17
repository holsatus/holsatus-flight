use core::sync::atomic::Ordering;

use crate::signals as s;
use crate::types::device::HardwareInfo;
use embassy_futures::join::join;
use embassy_futures::yield_now;
use embassy_time::{with_timeout, Duration};
use embassy_usb::class::cdc_acm::{Receiver, Sender};
use embassy_usb::UsbDevice;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::Driver,
    Builder, Config,
};
use static_cell::StaticCell;

pub async fn main(driver: impl Driver<'static>, info: &'static HardwareInfo) -> ! {
    const ID: &str = "usb_manager/main";
    info!("{}: Starting Shell task.", ID);

    const MAX_PACKET_SIZE: u8 = 64;

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = info.make.as_ref().map(|s| s.as_str());
    config.product = info.model.as_ref().map(|s| s.as_str());
    config.serial_number = info.serial_nr.as_ref().map(|s| s.as_str());
    config.max_packet_size_0 = MAX_PACKET_SIZE;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static DEVICE_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let device_descriptor = DEVICE_DESCRIPTOR.init([0; 256]).as_mut_slice();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]).as_mut_slice();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]).as_mut_slice();
    let control_buf = CONTROL_BUF.init([0; 256]).as_mut_slice();

    static STATE: StaticCell<State<'_>> = StaticCell::new();
    let state = STATE.init(State::new());

    let mut usb_builder = Builder::new(
        driver,
        config,
        device_descriptor,
        config_descriptor,
        bos_descriptor,
        control_buf,
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut usb_builder, state, MAX_PACKET_SIZE as u16);

    let (tx, rx) = class.split();

    // Build the builder.
    let usb = usb_builder.build();

    // Join the futures, never to return
    join(run_future(usb), echo_future(tx, rx)).await.0
}

pub async fn echo_future<D: Driver<'static>>(
    mut tx: Sender<'static, D>,
    mut rx: Receiver<'static, D>,
) -> ! {
    const TASK_ID: &str = "usb_manager/echo";

    let mut rcv_usb_connected = unwrap!(s::USB_CONNECTED.receiver());

    let mut buffer = [0u8; 64];

    loop {
        // Wait for USB to be connected
        rcv_usb_connected.get_and(|usb| *usb).await;
        rx.wait_connection().await;
        tx.wait_connection().await;

        while let Ok(Ok(n)) =
            with_timeout(Duration::from_millis(500), rx.read_packet(&mut buffer)).await
        {
            let slice = &buffer[..n];
            while !rx.dtr() {
                yield_now().await;
            }
            info!("{}: Echoing back..", TASK_ID);
            if tx.write_packet(slice).await.is_err() {
                error!("{}: Write packet error!", TASK_ID);
                break;
            }
        }
    }
}

pub(super) async fn run_future<D: Driver<'static>>(mut usb: UsbDevice<'static, D>) -> ! {
    const TASK_ID: &str = "usb_manager/run";
    let snd_usb_connected = s::USB_CONNECTED.sender();
    let mut rcv_cmd_arm_motors = unwrap!(s::CMD_ARM_MOTORS.receiver());

    info!("{}: Task started", TASK_ID);
    loop {
        // Wait for the USB to be connected
        info!("{}: Waiting for USB connection", TASK_ID);
        usb.wait_resume().await;

        // After USB is connected, check if the motors are armed without override
        // TODO: Replace this with a check for whether we are in the air.
        if rcv_cmd_arm_motors.get().await.0 && !s::OUTPUT_OVERRIDE.load(Ordering::Relaxed) {
            info!("{}: Motors armed without output override, aborting", TASK_ID);
            usb.disable().await;
            continue;
        }

        // Signal the USB connection status and run USB until it is suspended
        snd_usb_connected.send(true);
        info!("{}: USB connection resumed", TASK_ID);
        usb.run_until_suspend().await;
        snd_usb_connected.send(false);
    }
}
