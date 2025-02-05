use crate::signals as s;
use crate::types::device::HardwareInfo;
use embassy_futures::join::join;
use embassy_time::Timer;
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
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor_buf = CONFIG_DESCRIPTOR.init([0; 256]).as_mut_slice();
    let bos_descriptor_buf = BOS_DESCRIPTOR.init([0; 256]).as_mut_slice();
    let msos_descriptor_buf = MSOS_DESCRIPTOR.init([0; 256]).as_mut_slice();
    let control_buf = CONTROL_BUF.init([0; 256]).as_mut_slice();

    static STATE: StaticCell<State<'_>> = StaticCell::new();
    let state = STATE.init(State::new());

    let mut usb_builder = Builder::new(
        driver,
        config,
        config_descriptor_buf,
        bos_descriptor_buf,
        msos_descriptor_buf,
        control_buf,
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut usb_builder, state, MAX_PACKET_SIZE as u16);

    // Build the builder.
    let usb = usb_builder.build();

    // Join the futures, never to return
    join(run_future(usb), shell_future(class)).await.0
}

pub async fn shell_future<D: Driver<'static>>(
    mut serial: CdcAcmClass<'static, D>,
) -> ! {

    const TASK_ID: &str = "usb_manager/echo";

    let mut rcv_usb_connected = s::USB_CONNECTED.receiver();

    loop {
        // Wait for USB to be connected
        rcv_usb_connected.get_and(|connected| *connected).await;
        serial.wait_connection().await;

        if let Err(e) = crate::shell::run_cli(UsbEio{ usb: &mut serial} ).await {
            let h_error: crate::errors::adapter::embedded_io::EmbeddedIoError = e.into();
            error!("{}: Error running shell: {:?}", TASK_ID, h_error);
        }
    }
}


pub(super) async fn run_future<D: Driver<'static>>(mut usb: UsbDevice<'static, D>) -> ! {
    const TASK_ID: &str = "usb_manager/run";
    let mut snd_usb_connected = s::USB_CONNECTED.sender();

    info!("{}: Task started", TASK_ID);
    loop {
        // Wait for the USB to be connected
        info!("{}: Waiting for USB connection", TASK_ID);
        usb.wait_resume().await;

        // Signal the USB connection status and run USB until it is suspended
        snd_usb_connected.send(true);
        info!("{}: USB connection resumed", TASK_ID);

        usb.run_until_suspend().await;
        snd_usb_connected.send(false);

        Timer::after_millis(100).await;
    }
}

// Implement the embedded-io-async trait on a wrapped embassy-usb device.

struct UsbEio<'a, D: Driver<'static>> {
    usb: &'a mut CdcAcmClass<'static, D>,
}

impl <'a, D: Driver<'static>> embedded_io_async::ErrorType for UsbEio<'a, D> {
    type Error = embedded_io_async::ErrorKind;
}

impl <'a, D: Driver<'static>> embedded_io_async::Read for UsbEio<'a, D> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.usb.read_packet(buf).await.map_err(err_map)
    }
}

impl <'a, D: Driver<'static>> embedded_io_async::Write for UsbEio<'a,D> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        trace!("Writing {} bytes to USB: {:?}", buf.len(), buf);
        let mut written = 0;
        for chunk in buf.chunks(self.usb.max_packet_size() as usize) {
            self.usb.write_packet(chunk).await.map_err(err_map)?;
            written += chunk.len();
        }
        Ok(written)
    }
}

impl <'a, D: Driver<'static>> embedded_io::Write for UsbEio<'a,D> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embassy_futures::block_on(async {
            <Self as embedded_io_async::Write>::write(self, buf).await
        })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn err_map(e: embassy_usb::driver::EndpointError) -> embedded_io::ErrorKind {
    match e {
        embassy_usb::driver::EndpointError::BufferOverflow => embedded_io::ErrorKind::OutOfMemory,
        embassy_usb::driver::EndpointError::Disabled => embedded_io::ErrorKind::NotConnected,
    }
}
