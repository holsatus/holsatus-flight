use crate::errors::adapter::embedded_io::EmbeddedIoError;
use crate::serial::IoStreamRaw;
use crate::signals as s;
use crate::types::device::HardwareInfo;
use embassy_futures::join::join3;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::class::cdc_acm::{BufferedReceiver, Sender};
use embassy_usb::UsbDevice;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::Driver,
    Builder, Config,
};
use embedded_io::ErrorKind;
use grantable_io::GrantableIo;
use static_cell::StaticCell;

pub async fn main(driver: impl Driver<'static>, info: HardwareInfo) -> ! {
    const ID: &str = "usb";
    info!("{}: Starting usb manager task.", ID);

    const MAX_PACKET_SIZE: u8 = 64;

    // Safety: The task never exits, and we shadow the original name
    let info: &'static HardwareInfo = unsafe { core::mem::transmute(&info) };

    // Configure USB device
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = info.make.as_ref().map(|s| s.as_str());
    config.product = info.model.as_ref().map(|s| s.as_str());
    config.serial_number = info.serial_nr.as_ref().map(|s| s.as_str());

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
    let usb = usb_builder.build();

    let (usb_writer, usb_reader) = class.split();

    static GRANTABLE_RX: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();
    static GRANTABLE_TX: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();

    let (mut dev_cons, app_prod) = GRANTABLE_RX.claim_writer();
    let (mut dev_prod, app_cons) = GRANTABLE_TX.claim_reader();

    let io_stream_raw = IoStreamRaw::new("usb", app_cons, app_prod);

    static IO_STREAM_RAW: StaticCell<IoStreamRaw<'static>> = StaticCell::new();
    let io_stream_ref = IO_STREAM_RAW.init(io_stream_raw);

    crate::serial::insert(io_stream_ref).unwrap();

    // Make receiver buffered to uphold embedded_io_async::Read invariants
    static READ_BUFFER: StaticCell<[u8; MAX_PACKET_SIZE as usize]> = StaticCell::new();
    let buffer = READ_BUFFER.init([0u8; MAX_PACKET_SIZE as usize]);
    let usb_reader = usb_reader.into_buffered(buffer);
    let usb_reader = UsbReader { usb_reader };

    let usb_writer = UsbWriter { usb_writer };

    info!("{}: Running all futures", ID);

    let map_err = |error: ErrorKind| error.into();

    join3(
        dev_prod.embedded_io_connect_mapped(usb_reader, map_err),
        dev_cons.embedded_io_connect_mapped(usb_writer, map_err),
        connect_runner_future(usb),
    )
    .await
    .2
}

pub(super) async fn connect_runner_future<D: Driver<'static>>(mut usb: UsbDevice<'static, D>) -> ! {
    const ID: &str = "usb_manager";
    let mut snd_usb_connected = s::USB_CONNECTED.sender();

    usb.disable().await;

    // This is flaky as f... on stm32. Until a connection is made,
    // it will false believe there is a USB connection
    snd_usb_connected.send(false);

    loop {
        info!("{}: USB connection resumed", ID);
        Timer::after_millis(100).await;
        usb.run_until_suspend().await;
        snd_usb_connected.send(false);

        info!("{}: Waiting for USB connection", ID);
        Timer::after_millis(100).await;
        usb.wait_resume().await;
        snd_usb_connected.send(true);
    }
}

// Implement the embedded-io-async trait on a wrapped embassy-usb device.

struct UsbWriter<D: Driver<'static>> {
    usb_writer: Sender<'static, D>,
}

impl<D: Driver<'static>> embedded_io_async::ErrorType for UsbWriter<D> {
    type Error = embedded_io_async::ErrorKind;
}

impl<D: Driver<'static>> embedded_io_async::Write for UsbWriter<D> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !self.usb_writer.dtr() {
            Timer::after_millis(100).await;
            return Err(embedded_io::ErrorKind::NotConnected);
        }

        let mut count = 0;
        for chunk in buf.chunks(self.usb_writer.max_packet_size() as usize) {
            self.usb_writer.write_packet(chunk).await.map_err(err_map)?;
            count += chunk.len()
        }

        if buf.len() % self.usb_writer.max_packet_size() as usize == 0 {
            self.usb_writer.write_packet(&[]).await.map_err(err_map)?;
        }

        Ok(count)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.usb_writer.flush().await.map_err(err_map)
    }
}

impl<D: Driver<'static>> embedded_io::Write for UsbWriter<D> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !self.usb_writer.dtr() {
            return Err(embedded_io::ErrorKind::NotConnected);
        }

        // Block for at most 20 milli seconds. Be very careful only to use the sync UsbWriter
        // in executors of lower priority than the one driving the USB device.
        let maybe_timeout = embassy_futures::block_on(async {
            with_timeout(
                Duration::from_millis(20),
                <Self as embedded_io_async::Write>::write(self, buf),
            )
            .await
        });

        match maybe_timeout {
            Ok(result) => return result,
            Err(_) => return Err(embedded_io::ErrorKind::TimedOut),
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        if !self.usb_writer.dtr() {
            return Err(embedded_io::ErrorKind::NotConnected);
        }
        
        // Block for at most 20 milli seconds. Be very careful only to use the sync UsbWriter
        // in executors of lower priority than the one driving the USB device.
        let maybe_timeout = embassy_futures::block_on(async {
            with_timeout(
                Duration::from_millis(20),
                <Self as embedded_io_async::Write>::flush(self),
            )
            .await
        });

        match maybe_timeout {
            Ok(result) => return result,
            Err(_) => return Err(embedded_io::ErrorKind::TimedOut),
        }
    }
}

struct UsbReader<D: Driver<'static>> {
    usb_reader: BufferedReceiver<'static, D>,
}

impl<D: Driver<'static>> embedded_io_async::ErrorType for UsbReader<D> {
    type Error = embedded_io_async::ErrorKind;
}

impl<D: Driver<'static>> embedded_io_async::Read for UsbReader<D> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if !self.usb_reader.dtr() {
            Timer::after_millis(100).await;
            return Err(embedded_io::ErrorKind::NotConnected);
        }

        self.usb_reader.read(buf).await.map_err(err_map)
    }
}

fn err_map(e: embassy_usb::driver::EndpointError) -> embedded_io::ErrorKind {
    match e {
        embassy_usb::driver::EndpointError::BufferOverflow => embedded_io::ErrorKind::OutOfMemory,
        embassy_usb::driver::EndpointError::Disabled => embedded_io::ErrorKind::NotFound,
    }
}
