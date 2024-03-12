//! This module is extremely experimental, and any implemented code is on a proof-of-concept level.
//! 
//! The goal of this module is to provide a shell interface to the Holsatus Flight firmware, and to get
//! there we need a more extensive system for handling commands and responses. This is a work in progress.

use core::{fmt::Write, future::poll_fn, task::Poll};

use embassy_futures::{join::join, select::select};
use embassy_rp::{peripherals::USB, usb::{Driver, InterruptHandler}};
use embassy_time::{with_timeout, Duration, Ticker, Timer};
use embassy_usb::{class::cdc_acm::{CdcAcmClass, State}, Builder, Config, UsbDevice};
use embedded_cli::cli::{CliBuilder, CliHandle};
use embedded_io::ErrorKind;
use static_cell::StaticCell;
use embedded_cli::Command;

const TASK_ID: &str = "[SHELL]";

embassy_rp::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
pub async fn holsatus_shell(usb: USB) {

    defmt::info!("{}: Starting Shell task.", TASK_ID);

    // Create the driver, from the HAL.
    let driver = Driver::new(usb, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Holsatus");
    config.product = Some("Holsatus Flight USB CLI");
    config.serial_number = Some("none applicable");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

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
        &mut [], // no msos descripto _iors
        control_buf,
    );
    
    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut usb_builder, state, 64);
    
    // Split the class into a sender and receiver. Sender is owned by the Cli struct
    let (tx, mut rx, _) = class.split_with_control();

    // Build the builder.
    let usb = usb_builder.build();

    // Spawn USB task in the current executor
    embassy_executor::Spawner::for_current_executor().await
        .must_spawn(embassy_usb_task(usb));

    // Create blocking writer for USB tx
    let writer = UsbWriter::new(tx);

    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer([0; 64])
        .history_buffer([0; 64])
        .prompt(">> ")
        .build()
        .ok().unwrap();

    let request_sender = msg::REQUEST_QUEUE.sender();
    
    let snd_attitude_pids = msg::CFG_ATTITUDE_PIDS.sender();
    let mut rcv_usb_connected = msg::USB_CONNECTED.receiver().unwrap();

    let mut request_option = None;
    let mut buffer = [0;64];

    let mut arming_blockers = false;

    loop {
        
        // Future to allow waiting for the DTR to be ready
        let async_dtr = || poll_fn(|cx| {
            match rx.dtr() {
                true => Poll::Ready(()),
                false => {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            }
        });

        // If expecting response, wait for async_dtr().await to be ready
        // then print out response.

        match select(rcv_usb_connected.changed_and(|c|c == &true), async_dtr()).await {

            // Upon new USB connection, clear the screen and print the Holsatus graphic
            embassy_futures::select::Either::First(_) => {
                async_dtr().await;
                Timer::after(Duration::from_millis(500)).await;
                cli.write(|w|{
                    w.write_str(CLEAR_SCREEN)?;
                    w.write_str(CLEAR_SCREEN)
                }).expect("Failed to write to USB");
                Timer::after(Duration::from_millis(100)).await;
                cli.write(|w|{
                    ufmt::uwrite!(w, "{}\n{}\n\n", HOLSATUS_GRAPHIC, "  Holsatus Shell - Preview")
                }).expect("Failed to write to USB");
            },
            embassy_futures::select::Either::Second(_) => {

                if request_option.is_some() {

                    // Remove prompt since we are awaiting a response.
                    cli.set_prompt("").unwrap();

                    cli.write(|w|{
                        ufmt::uwrite!(w, "Waiting for response..\n")
                    }).expect("Failed to write to USB");

                    // Wait for response
                    Timer::after_millis(10).await;
                    while msg::GYR_CALIBRATING.spin_get().await {
                        Timer::after_millis(10).await;
                    }

                    cli.write(|w|{
                        ufmt::uwrite!(w, "Calibration complete\n")
                    }).expect("Failed to write to USB");

                    // Re-enable prompt
                    cli.set_prompt(">> ").unwrap();
                
                } else if arming_blockers {

                    let mut rcv_arm_blocker = msg::ARM_BLOCKER.receiver().unwrap();

                    // Remove prompt since we are awaiting a response.
                    cli.set_prompt("").unwrap();

                    cli.write(|w| {
                        w.writeln_str("")
                    }).expect("Failed to write to USB");

                    let mut ticker = Ticker::every(Duration::from_hz(5));
                    loop {
                        match with_timeout(Duration::from_millis(10), rx.read_packet(&mut buffer)).await {
                            Ok(Err(_)) => defmt::unreachable!("Failed to read packet from USB"),

                            // If any element in the buffer is a space character (ascii 32), break from the loop
                            Ok(Ok(n)) if n > 0 => if buffer.iter().take(n).any(|e| e == &32 ) { break },
                            _ => { /* timeout is expected here */ },
                        }
                        let flag = rcv_arm_blocker.get().await;
                        cli.write(|w| {
                            w.write_fmt(format_args!("\x1B[1A\x1B[2KBlockers: {:?}", flag)).unwrap();
                            Ok(())
                        }).expect("Failed to write to USB");
                        ticker.next().await;
                    }
                    cli.set_prompt(">> ").unwrap();

                }
            },
        }
        
        arming_blockers = false;
        request_option = None;

        let n = rx.read_packet(&mut buffer).await.unwrap();
        
        for byte in buffer[..n].iter() {        

            let process_res = cli.process_byte::<Base, _>(
                *byte,
                &mut Base::processor(|cli: &mut CliHandle<'_, _, _>, command| {
                    match command {
                        
                        Base::Info => on_info(cli)?,

                        Base::Clear => {
                            cli.writer().write_str(CLEAR_SCREEN)?;
                        },

                        Base::Time => {
                            let time = time_boot_ms();
                            write(cli.writer(), format_args!("Time is: {} seconds", time as f32 / 1e3))?;
                        },

                        Base::Sensors => {
                            ufmt::uwrite!(cli.writer(), "error: This command is not currently implemented")?
                        }

                        Base::Arming { blockers } => {
                            if blockers {
                                arming_blockers = true;
                            }
                        }

                        Base::Attitude { rate } => {
                            let rate = rate.unwrap_or(1);
                            defmt::info!("{}: Streaming attitude in shell at rate: {}", TASK_ID, rate);
                        },

                        Base::Get { parameter } => {
                            use crate::config::ParamLookup;
                            use crate::config::Type as T;
                            let pids = snd_attitude_pids.try_get().expect("Failed to get attitude pids");
                            let mut splits = parameter.split_inclusive('_');
                            match splits.next() {
                                Some("attpid_") => {
                                    if let T::F32(val) = pids.get(splits) {
                                        cli.writer().write_fmt(format_args!("{}: {}", parameter, val)).unwrap();
                                        return Ok(());
                                    }
                                },
                                _ => {},
                            }
                            cli.writer().write_fmt(format_args!("No value could be found for: {}", parameter)).unwrap()
                        }

                        Base::Set { parameter: _p, value: _v } => {
                            todo!()
                        }

                        Base::Cal {command} => {
                            match command {
                                CalibrateCommand::Gyr { id, seconds, variance: _v } => {
                                    match id {
                                        Some(id) if (id as usize) < crate::N_IMU => ufmt::uwrite!(cli.writer(), "Calibrating gyroscope {} over {:?} seconds\n", id, seconds)?,
                                        Some(id) => ufmt::uwrite!(cli.writer(), "error: Gyroscope {} is out of range. Valid are 0-{}\n", id, crate::N_IMU - 1)?,
                                        None => ufmt::uwrite!(cli.writer(), "Calibrating all available gyroscopes..\n")?,
                                    }
                                    request_option = Some(EventRequest::StartGyrCalib)
                                },
                                CalibrateCommand::Acc { id } => {
                                    match id {
                                        Some(id) if (id as usize) < crate::N_IMU => ufmt::uwrite!(cli.writer(), "Calibrating accelerometers {}\n", id)?,
                                        Some(id) => ufmt::uwrite!(cli.writer(), "error: Accelerometer {} is out of range. Valid are 0-{}\n", id, crate::N_IMU - 1)?,
                                        None => ufmt::uwrite!(cli.writer(), "Calibrating all available accelerometers..\n")?,
                                    }
                                    request_option = Some(EventRequest::StartAccCalib)
                                },
                                CalibrateCommand::Mag { id } => {
                                    match id {
                                        Some(id) if (id as usize) < crate::N_IMU => ufmt::uwrite!(cli.writer(), "Calibrating magnetometers {}\n", id)?,
                                        Some(id) => ufmt::uwrite!(cli.writer(), "error: Magnetometer {} is out of range. Valid are 0-{}\n", id, crate::N_IMU - 1)?,
                                        None => ufmt::uwrite!(cli.writer(), "Calibrating all available magnetometers..\n")?,
                                    }
                                    request_option = Some(EventRequest::StartMagCalib)
                                },
                            }
                        },

                        Base::Config { command } => {
                            match command {
                                ConfigCommand::Save => {
                                    ufmt::uwrite!(cli.writer(), "Saving configuration..")?;
                                    request_option = Some(EventRequest::SaveConfig);
                                }
                                ConfigCommand::Reload => todo!(),
                                ConfigCommand::Show { config_to_show: _c } => todo!(),
                                
                            }
                        }
                    }
                    Ok(())
                }),
            );

            // Log on error
            if let Err(_) = process_res {
                defmt::error!("{}: Failed to process incoming byte", TASK_ID);
            }

            // Send the request, if any
            if let Some(request) = request_option {
                request_sender.send(request).await;
            }
        }
    }
}

const CLEAR_SCREEN: &str = "\x1B[2J";

// This looks scuffed since we need to use \\ to escape the backslashes in the string
const HOLSATUS_GRAPHIC: &str = 
"   _   _       _           _
  | | | |     | |         | |
  | |_| | ___ | |___  __ _| |_ _   _ ___ 
  |  _  |/ _ \\| / __|/ _` | __| | | / __|
  | | | | (_) | \\__ \\ (_| | |_| |_| \\__ \\
  \\_| |_/\\___/|_|___/\\__,_|\\__|\\__,_|___/
";

/// This is a thin wrapper around `core::fmt::write`. Mostly usedful to make formatting which is too
/// restrictive for the equivalent `ufmt` writers. Prefer those whenever possible.
fn write<W: core::fmt::Write>(output: &mut W , args: core::fmt::Arguments<'_>) -> Result<(),ErrorKind> {
    core::fmt::write(output, args).map_err(|_| ErrorKind::Other)
}


/*

async fn cli_parse<W,E>(writer: W, bytes_rx: embassy_sync::pipe::Reader<'static, ThreadModeRawMutex, 10>)
where
    W: embedded_io::Write<Error = E>,
    E: embedded_io::Error,
{

    // Setup the CLI
    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer([0; 64])
        .history_buffer([0; 64])
        .prompt(">> ")
        .build()
        .ok().unwrap();

    let request_sender = msg::REQUEST_QUEUE.sender();

    let mut buffer = [0;1];
    loop {

        // Await incoming bytes
        bytes_rx.read(&mut buffer).await;

        // TODO Create a heapless Vec instead
        let mut request_option = None;

        let _ = cli.process_byte::<Base, _>(
            buffer[0],
            &mut Base::processor(|cli: &mut CliHandle<'_, _, _>, command| {
                match command {
                    
                    Base::Info => on_info(cli)?,

                    Base::Clear => {
                        ufmt::uwriteln!(cli.writer(), "{}", 27 as char)?;
                        ufmt::uwriteln!(cli.writer(), "[2J")?;
                    },

                    Base::Time => {
                        let time = time_boot_ms();
                        ufmt::uwrite!(cli.writer(), "Time: {} ms", time)?;
                    },

                    Base::Calibrate {command} => {
                        match command {
                            CalibrateCommand::Gyroscope { id } => {
                                match id {
                                    Some(id) if (id as usize) < crate::N_IMU => ufmt::uwrite!(cli.writer(), "Calibrating gyroscope {}\n", id)?,
                                    Some(id) => ufmt::uwrite!(cli.writer(), "error: Gyroscope {} is out of range. Valid are 0-{}\n", id, crate::N_IMU - 1)?,
                                    None => ufmt::uwrite!(cli.writer(), "Calibrating all available gyroscopes..\n")?,
                                }
                                request_option = Some(EventRequest::StartGyrCalib)
                            },
                            CalibrateCommand::Accelerometer { id } => {
                                match id {
                                    Some(id) if (id as usize) < crate::N_IMU => ufmt::uwrite!(cli.writer(), "Calibrating accelerometers {}\n", id)?,
                                    Some(id) => ufmt::uwrite!(cli.writer(), "error: Accelerometer {} is out of range. Valid are 0-{}\n", id, crate::N_IMU - 1)?,
                                    None => ufmt::uwrite!(cli.writer(), "Calibrating all available accelerometers..\n")?,
                                }
                                request_option = Some(EventRequest::StartAccCalib)
                            },
                            CalibrateCommand::Magnetometer { id } => {
                                match id {
                                    Some(id) if (id as usize) < crate::N_IMU => ufmt::uwrite!(cli.writer(), "Calibrating magnetometers {}\n", id)?,
                                    Some(id) => ufmt::uwrite!(cli.writer(), "error: Magnetometer {} is out of range. Valid are 0-{}\n", id, crate::N_IMU - 1)?,
                                    None => ufmt::uwrite!(cli.writer(), "Calibrating all available magnetometers..\n")?,
                                }
                                request_option = Some(EventRequest::StartMagCalib)
                            },
                        }
                    },

                    Base::Config { command } => {
                        match command {
                            ConfigCommand::Save => {
                                ufmt::uwrite!(cli.writer(), "Saving configuration..")?;
                                request_option = Some(EventRequest::SaveConfig);
                            }
                            ConfigCommand::Reload => todo!(),
                            ConfigCommand::Show { config_to_show } => todo!(),
                        }
                    }
                }
                Ok(())
            }),
        );
        if let Some(request) = request_option {
            request_sender.send(request).await;
        }
    }
}

 */



fn on_info<'a, W>(cli: &mut CliHandle<'a, W, ErrorKind>) -> Result<(),ErrorKind>
where
    W: embedded_io::Write<Error = ErrorKind>,
{
    ufmt::uwrite!(cli.writer(), "{}{}", HOLSATUS_GRAPHIC, 
        "\n  \
        https://github.com/holsatus/holsatus-flight/\n  \
        \n  \
        Version:      Holsatus Flight 0.2.0\n  \
        Build date:   12-03-2023\n  \
        Platform:     RP2040-dev\n  \
        \n\
        "
    )
}

#[derive(Command)]
enum Base<'a> {

    /// Display information about the current build of the firmware.
    Info,
    
    /// Clear the terminal
    Clear,

    /// Display the current system time of the vehicle
    Time,

    /// Display the currently active sensors and their calibration state
    Sensors,

    /// Functions related to arming and disarming the vehicle
    Arming {
            
            /// Print out the flags indicating why the vehicle is not arming
            #[arg(short = 'b', long)]
            blockers: bool,
    },

    /// Stream attitude data
    Attitude {

        #[arg(short = 'r', long)]
        rate: Option<u8>,
    },

    /// Start a calibration routine
    #[arg(alias = "cal")]
    Cal {
        
        #[command(subcommand)] 
        command: CalibrateCommand,
    },

    /// Interact with the configuration of the vehicle
    Config {

        #[command(subcommand)] 
        command: ConfigCommand<'a>,
    },


    /// Get a parameter from the vehicle
    Get {

        /// The name of the parameter to get
        parameter: &'a str,
    },

    /// Set a parameter on the vehicle
    Set {
            
        /// The name of the parameter to set
        parameter: &'a str,

        /// The value to set the parameter to
        value: &'a str,
    }
}

#[derive(Command)]
enum ConfigCommand<'a> {

    /// Save the current configuration to persistent flash storage
    Save,

    /// Reload the config from flash storage. This overwrites any unsaved changes.
    Reload,

    /// Show the current configuration in the shell
    Show {
        config_to_show: Option<&'a str>
    },
}

#[derive(Command)]
enum CalibrateCommand {

    /// Calibrate the gyroscopes
    Gyr {

        /// Number of seconds to run the calibration for
        #[arg(short = 's', long)]
        seconds: Option<u8>,

        /// Upper bound for allowed variance of the accelerometer during the calibration
        #[arg(short = 's', long)]
        variance: Option<f32>,
            
        /// The ID of the gyroscope to calibrate. If none is provided, all gyros are calibrated.
        id: Option<u8>,
    },

    /// Calibrate the accelerometers
    Acc {
            
            /// The ID of the accelerometer to calibrate. If none is provided, all accelerometers are calibrated.
            id: Option<u8>,
    },

    /// Calibrate the magnetometers
    Mag {
                
        /// The ID of the magnetometer to calibrate. If none is provided, all magnetometers are calibrated.
        id: Option<u8>,
    }

}

use crate::{functions::time_boot_ms, messaging as msg, transmitter::EventRequest};

use super::writer_impl::writer_embassy_usb::UsbWriter;

// #[cfg(feature = "rp2040")] 
type UsbInstance = USB;

/// This simple task is responsible for signaling the USB connection state to other tasks.
#[embassy_executor::task]
async fn embassy_usb_task(mut usb: UsbDevice<'static, Driver<'static, UsbInstance>>) -> ! {
    
    let usb_connected = msg::USB_CONNECTED.sender();
    let mut arm_vehicle = msg::CMD_ARM_VEHICLE.receiver().unwrap();

    loop {
        
        // Wait for both the USB to be connected and the vehicle to be disarmed
        join(usb.wait_resume(), arm_vehicle.get_and(|b|b == &false)).await;

        usb_connected.send(true);
        defmt::info!("{}: USB connection resumed", TASK_ID);
        usb.run_until_suspend().await;
        usb_connected.send(false);
        defmt::info!("{}: USB connection suspended", TASK_ID);
    }
}
