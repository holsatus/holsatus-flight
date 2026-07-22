/// Helper macro to create an interrupt executor.
#[macro_export]
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_executor::InterruptExecutor;
        use embassy_stm32::interrupt;
        use embassy_stm32::interrupt::{InterruptExt, Priority};

        interrupt::$interrupt.set_priority(Priority::$prio);
        static EXECUTOR: InterruptExecutor = InterruptExecutor::new();
        let spawner = EXECUTOR.start(interrupt::$interrupt);

        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $interrupt() {
            EXECUTOR.on_interrupt()
        }

        spawner
    }};
}

#[allow(unused)]
pub struct UsartBuffered<'d> {
    pub rx: embassy_stm32::usart::RingBufferedUartRx<'d>,
    pub tx: embassy_stm32::usart::UartTx<'d, embassy_stm32::mode::Async>,
}

#[macro_export]
macro_rules! impl_usart_setup {
    (
        $UsartX:ty,
        $USARTX:ident => $USARTX_irq:ident,
        $rx_dma:ident => $rx_dma_irq:ident,
        $tx_dma:ident => $tx_dma_irq:ident,
        runner = $fn_name:ident,
        ring_buf = $rb_size:literal,
        rx_buf = $rx_size:literal,
        tx_buf = $tx_size:literal,
    ) => {
        #[allow(unused)]
        impl $UsartX {
            pub fn setup<'d>(&'d mut self, uart_cfg: common::types::config::UartConfig) -> $crate::setup_macros::UsartBuffered<'d> {
                defmt::info!("Creating: {}", stringify!($UsartX));

                embassy_stm32::bind_interrupts!(struct Irqs {
                    $USARTX => embassy_stm32::usart::InterruptHandler<::embassy_stm32::peripherals::$USARTX_irq>;
                    $rx_dma => embassy_stm32::dma::InterruptHandler<::embassy_stm32::peripherals::$rx_dma_irq>;
                    $tx_dma => embassy_stm32::dma::InterruptHandler<::embassy_stm32::peripherals::$tx_dma_irq>;
                });

                let mut config = ::embassy_stm32::usart::Config::default();
                config.baudrate = uart_cfg.baud;

                let usart = embassy_stm32::usart::Uart::new(
                    self.periph.reborrow(),
                    self.rx_pin.reborrow(),
                    self.tx_pin.reborrow(),
                    self.tx_dma.reborrow(),
                    self.rx_dma.reborrow(),
                    Irqs,
                    config,
                );

                let (tx, rx) = usart.unwrap().split();

                // Provide a static buffer for the ring buffer.
                use ::static_cell::ConstStaticCell;

                #[link_section = ".ram_d3"]
                static USART_BUFFER: ConstStaticCell<[u8; $rb_size]> = ConstStaticCell::new([0; $rb_size]);
                let rx = rx.into_ring_buffered(USART_BUFFER.take());

                $crate::setup_macros::UsartBuffered { rx, tx }
            }
        }

        #[embassy_executor::task]
        pub(crate) async fn $fn_name(mut usart: $UsartX, uart_cfg: common::types::config::UartConfig, serial_id: &'static str) {
            let usart = usart.setup(uart_cfg);

            let (rx, tx) = (usart.rx, usart.tx);

            use common::serial::IoStreamRaw;
            use common::grantable_io::GrantableIo;
            use common::errors::adapter::embedded_io::EmbeddedIoError;

            static BUF_TX: GrantableIo<$tx_size, EmbeddedIoError> = GrantableIo::new();
            let (mut dev_prod, app_cons) = BUF_TX.claim_reader(); // Why does this panic!?

            static BUF_RX: GrantableIo<$rx_size, EmbeddedIoError> = GrantableIo::new();
            let (mut dev_cons, app_prod) = BUF_RX.claim_writer(); // Why does this panic!?

            let io_stream_raw = IoStreamRaw::new(serial_id, app_cons, app_prod);

            use static_cell::StaticCell;
            static IO_STREAM_RAW: StaticCell<IoStreamRaw<'static>> = StaticCell::new();
            let io_stream_ref = IO_STREAM_RAW.init(io_stream_raw);

            if common::serial::insert(io_stream_ref).is_err() {
                defmt::error!("[{}/setup]: Failed to register serial device", serial_id);
                return;
            }

            let map_err = |error: embassy_stm32::usart::Error| {
                <embassy_stm32::usart::Error as common::embedded_io::Error>::kind(&error).into()
            };

            common::embassy_futures::join::join(
                dev_prod.embedded_io_connect(rx, map_err),
                dev_cons.embedded_io_connect(tx, map_err),
            ).await;

            defmt::warn!("[{}] Stream disconnected unexpectedly", serial_id)
        }
    };
}

/// Generate an `impl I2cN { fn setup(...) }` block from an `assign_resources!` peripheral.
///
/// # Usage
///
/// ```ignore
/// impl_i2c_setup!(
///     I2c1,
///     I2C1_EV => I2C1,
///     I2C1_ER => I2C1,
///     DMA1_STREAM0 => DMA1_CH0,
///     DMA1_STREAM7 => DMA1_CH7,
/// );
/// ```
///
/// Expands to the `impl` block with `bind_interrupts!` and `I2c::new(...)`.
#[macro_export]
macro_rules! impl_i2c_setup {
    (
        $Peripheral:ty,   // struct from assign_resources, e.g. I2c1
        $ev_irq:ident => $ev_periph:ident,       // e.g. I2C1_EV
        $er_irq:ident => $er_periph:ident,       // e.g. I2C1_ER
        $rx_irq:ident => $rx_ch:ident,
        $tx_irq:ident => $tx_ch:ident,
    ) => {
        impl $Peripheral {
            pub fn setup(
                self,
                cfg: common::types::config::I2cConfig,
            ) -> impl common::embedded_hal_async::i2c::I2c {
                embassy_stm32::bind_interrupts!(
                    struct Irqs {
                        $ev_irq =>
                            embassy_stm32::i2c::EventInterruptHandler<
                                embassy_stm32::peripherals::$ev_periph
                            >;
                        $er_irq =>
                            embassy_stm32::i2c::ErrorInterruptHandler<
                                embassy_stm32::peripherals::$er_periph
                            >;
                        $rx_irq =>
                            embassy_stm32::dma::InterruptHandler<
                                embassy_stm32::peripherals::$rx_ch
                            >;
                        $tx_irq =>
                            embassy_stm32::dma::InterruptHandler<
                                embassy_stm32::peripherals::$tx_ch
                            >;
                    }
                );

                let mut config = embassy_stm32::i2c::Config::default();
                config.sda_pullup = cfg.sda_pullup;
                config.scl_pullup = cfg.scl_pullup;
                config.frequency = embassy_stm32::time::Hertz::hz(cfg.frequency);

                embassy_stm32::i2c::I2c::new(
                    self.periph,
                    self.scl,
                    self.sda,
                    self.tx_dma,
                    self.rx_dma,
                    Irqs,
                    config,
                )
            }
        }
    };
}

// ---- EXTI Input Pin ----

#[macro_export]
macro_rules! impl_exti_setup {
    (
        $Pin:ty,
        $exti_irq:ident,
        $pull:expr
        $(,)?
    ) => {
        impl $Pin {
            pub fn _setup(self) -> impl common::embedded_hal_async::digital::Wait {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $exti_irq => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::$exti_irq>;
                });

                embassy_stm32::exti::ExtiInput::new(self.pin, self.exti, $pull, Irqs)
            }
        }
    };
}

// ---- SPI ----

#[macro_export]
macro_rules! impl_spi_setup {
    (
        $Spi:ty,
        $tx_irq:ident => $tx_periph:ident,
        $rx_irq:ident => $rx_periph:ident
        $(,)?
    ) => {
        impl $Spi {
            pub fn setup(self) -> impl common::embedded_hal_async::spi::SpiBus {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $tx_irq => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::$tx_periph>;
                    $rx_irq => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::$rx_periph>;
                });

                embassy_stm32::spi::Spi::new(
                    self.periph,
                    self.sck,
                    self.mosi,
                    self.miso,
                    self.tx_dma,
                    self.rx_dma,
                    Irqs,
                    Default::default(),
                )
            }
        }
    };
}

// ---- FLASH ----

#[macro_export]
macro_rules! impl_flash_setup {
    (
        $Flash:ty,
        $flash_irq:ident
        $(,)?
    ) => {
        impl $Flash {
            pub fn setup<'d>(self) -> impl common::embedded_storage_async::nor_flash::NorFlash {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $flash_irq => embassy_stm32::flash::InterruptHandler;
                });

                embassy_stm32::flash::Flash::new(self.periph, Irqs)
            }
        }
    };
}

// ---- DShot Motor Driver ----

#[macro_export]
macro_rules! impl_up_dma_dshot_setup {
    (
        $MotorDriver:ty,
        $dma_irq:ident => $dma_periph:ident
        $(,)?
    ) => {
        impl $MotorDriver {
            pub fn setup(&mut self, dshot: common::types::config::DshotConfig) -> impl common::hw_abstraction::OutputGroup + '_ {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $dma_irq => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::$dma_periph>;
                });

                $crate::dshot_driver::DshotDriver::new(
                    self.timer.reborrow(),
                    self.pin_1.reborrow(),
                    self.pin_2.reborrow(),
                    self.pin_3.reborrow(),
                    self.pin_4.reborrow(),
                    $crate::dshot_driver::UpDmaWaveform::new(self.up_dma.reborrow(), Irqs),
                    dshot as u32,
                )
            }
        }
    };
}

#[macro_export]
macro_rules! impl_qs_dshot_setup {
    (
        $MotorDriver:ty,
        $dma_irq:ident => $dma_periph:ident
        $(,)?
    ) => {
        impl $MotorDriver {
            pub fn setup(&mut self, dshot: common::types::config::DshotConfig) -> impl common::hw_abstraction::OutputGroup + '_ {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $dma_irq => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::$dma_periph>;
                });

                $crate::dshot_driver::DshotDriver::new(
                    self.timer.reborrow(),
                    self.pin_1.reborrow(),
                    self.pin_2.reborrow(),
                    self.pin_3.reborrow(),
                    self.pin_4.reborrow(),
                    $crate::dshot_driver::UpDmaWaveform::new(self.up_dma.reborrow(), Irqs),
                    dshot as u32,
                )
            }
        }
    };
}

// ---- SDMMC / SDCard (call inside module that defines SdmmcDevice) ----

#[macro_export]
macro_rules! impl_sdmmc_setup {
    (
        $Sdcard:ty,
        $periph_irq:ident => $periph:ident,
        $dma_irq:ident => $dma_periph:ident
        $(,)?
    ) => {
        impl $Sdcard {
            pub fn setup(self, config: common::types::config::SdmmcConfig) -> impl common::tasks::blackbox_fat::BlockDevice<512> + common::tasks::blackbox_fat::Reset {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $periph_irq => embassy_stm32::sdmmc::InterruptHandler<embassy_stm32::peripherals::$periph>;
                    $dma_irq => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::$dma_periph>;
                });

                use static_cell::StaticCell;
                static SDMMC: StaticCell<embassy_stm32::sdmmc::Sdmmc<'static>> = StaticCell::new();
                let sdmmc = SDMMC.init(embassy_stm32::sdmmc::Sdmmc::new_4bit(
                    self.periph,
                    self.dma,
                    Irqs,
                    self.clk,
                    self.cmd,
                    self.d0,
                    self.d1,
                    self.d2,
                    self.d3,
                    Default::default(),
                ));

                $crate::block_device::SdmmcDevice {
                    storage: embassy_stm32::sdmmc::sd::StorageDevice::new_uninit_sd_card(sdmmc),
                    freq: embassy_stm32::time::Hertz::hz(config.frequency),
                }
            }
        }
    };

    (
        $Sdcard:ty,
        $periph_irq:ident => $periph:ident
        $(,)?
    ) => {
        impl $Sdcard {
            pub fn setup(self, config: common::types::config::SdmmcConfig) -> impl common::tasks::blackbox_fat::BlockDevice<512> + common::tasks::blackbox_fat::Reset {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $periph_irq => embassy_stm32::sdmmc::InterruptHandler<embassy_stm32::peripherals::$periph>;
                });

                use static_cell::StaticCell;
                static SDMMC: StaticCell<embassy_stm32::sdmmc::Sdmmc<'static>> = StaticCell::new();
                let sdmmc = SDMMC.init(embassy_stm32::sdmmc::Sdmmc::new_4bit(
                    self.periph,
                    Irqs,
                    self.clk,
                    self.cmd,
                    self.d0,
                    self.d1,
                    self.d2,
                    self.d3,
                    Default::default(),
                ));

                $crate::block_device::SdmmcDevice {
                    storage: embassy_stm32::sdmmc::sd::StorageDevice::new_uninit_sd_card(sdmmc),
                    freq: embassy_stm32::time::Hertz::hz(config.frequency),
                }
            }
        }
    };
}

// ---- USB ----

#[macro_export]
macro_rules! impl_usb_setup {
    (
        $Usb:ty,
        $usb_irq:ident => $usb_periph:ident,
        $buf_size:literal
        $(,)?
    ) => {
        impl $Usb {
            pub fn setup(self) -> impl common::embassy_usb::driver::Driver<'static> {
                embassy_stm32::bind_interrupts!(struct Irqs {
                    $usb_irq => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::$usb_periph>;
                });

                use static_cell::StaticCell;
                static USB_BUFFER: StaticCell<[u8; $buf_size]> = StaticCell::new();
                let usb_buffer = USB_BUFFER.init([0u8; $buf_size]);

                let mut config = embassy_stm32::usb::Config::default();
                config.vbus_detection = true;

                embassy_stm32::usb::Driver::new_fs(
                    self.usb,
                    Irqs,
                    self.dp,
                    self.dm,
                    usb_buffer,
                    Default::default(),
                )
            }
        }
    };
}
