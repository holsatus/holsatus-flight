//! This module is extremely experimental, and any implemented code is on a proof-of-concept level.
//! 
//! The goal of this module is to provide a shell interface to the Holsatus Flight firmware, and to get
//! there we need a more extensive system for handling commands and responses. This is a work in progress.

use core::fmt::Write as _;
use core::{future::poll_fn, task::Poll};

use embassy_futures::select::select;
use embassy_sync::watch::WatchBehavior;
use embassy_time::{with_timeout, Duration, Instant, Ticker, Timer};
use embassy_usb::class::cdc_acm::Receiver;
use embassy_usb::{class::cdc_acm::{CdcAcmClass, State}, Builder, Config};
use embedded_cli::cli::Cli;
use embedded_cli::{buffer::Buffer, cli::{CliBuilder, CliHandle}, writer::Writer};
use embedded_io::{ErrorKind, Write};
use num_traits::Float;
use static_cell::StaticCell;
use embedded_cli::Command;

use crate::t_commander::CommanderRequest;
use crate::messaging as msg;
use crate::t_gyr_calibration::CalGyrConfig;
use super::writer_impl::writer_embassy_usb::UsbWriter;

const TASK_ID: &str = "[SHELL]";
const PROMPT: &str = "[holsatus]$ ";

// Error messages
const ERR_SENSOR_ID_OUT_OF_RANGE: &str =  "error: Sensor ID is out of range";

#[embassy_executor::task]
pub async fn holsatus_shell(driver: crate::bsp::UsbPeripheral) {

    defmt::info!("{}: Starting Shell task.", TASK_ID);

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
        .must_spawn(super::usb_task::embassy_usb_task(usb));

    // Create blocking writer for USB tx
    let writer = UsbWriter::new(tx);

    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer([0; 64])
        .history_buffer([0; 128])
        .prompt(PROMPT)
        .build()
        .ok().unwrap();

    let request_sender = msg::REQUEST_QUEUE.sender();
    
    let snd_attitude_pids = msg::CFG_ATTITUDE_PIDS.sender();
    let mut rcv_usb_connected = msg::USB_CONNECTED.receiver().unwrap();

    let mut request_option = None;
    let mut buffer = [0;64];

    let mut response_command = None;

    loop {
    
        // If expecting response, wait for async_dtr().await to be ready
        // then print out response.

        match select(rcv_usb_connected.changed_and(|c|c == &true), dtr_ready(&rx)).await {

            // Upon new USB connection, clear the screen and print the Holsatus graphic
            embassy_futures::select::Either::First(_) => {
                dtr_ready(&rx).await;
                Timer::after(Duration::from_millis(500)).await;
                cli.write(|w|{
                    w.write_str(CLEAR_SCREEN)
                }).expect("Failed to write to USB");
                Timer::after(Duration::from_millis(100)).await;
                cli.write(|w|{
                    ufmt::uwrite!(w, "{}\n{}\n\n", HOLSATUS_GRAPHIC, "  Holsatus Shell - Preview")
                }).expect("Failed to write to USB");
            },
            embassy_futures::select::Either::Second(_) => {

                // Send commander request if any
                if let Some(request) = request_option.take() {
                    request_sender.send(request).await;
                }

                match response_command.take() {
                    Some(Base::Stream { command }) => match_stream_command(&command, &mut cli, &mut rx, &mut buffer).await,
                    Some(Base::Cal { command: CalibrateCommand::Gyro { seconds, variance, id } }) => {
                        use crate::t_gyr_calibration::GyrCalStatus;

                        let mut completed = false;
                        'calibration: {

                            cli.set_prompt("").unwrap();

                            cli.write(|w|{
                                ufmt::uwriteln!(w, "Calibraiting gyroscopes..")
                            }).expect("Failed to write to USB");

                            Timer::after_millis(100).await;
                            while let Some(GyrCalStatus::Collecting { sensors, timeleft_ms }) = msg::CMD_GYR_STATUS.try_get() {

                                // Read a packet from the USB receiver (with timeout) to figure out if we should exit
                                match with_timeout(Duration::from_millis(10), rx.read_packet(&mut buffer)).await {
                                    Ok(Err(_)) => {
                                        break 'calibration
                                    },

                                    // If any element in the buffer is a space character (ascii 32), break from the loop
                                    Ok(Ok(n)) if n > 0 => if buffer.iter().take(n).any(|e| e == &32 ) {
                                        break 'calibration
                                    },
                                    _ => { /* timeout is expected here */ },
                                }

                                let time_rounded_s = (timeleft_ms as f32 / 1000.0).round() as u32;

                                cli.write(|w|{
                                    ufmt::uwriteln!(w, "Time left: {} s", time_rounded_s)
                                }).expect("Failed to write to USB");
                                Timer::after_secs(1).await;
                            }

                            if let Some(GyrCalStatus::Done(calibs)) = msg::CMD_GYR_STATUS.try_get() {
                                for (cid, calib) in  calibs.iter().enumerate() {
                                    if let Some(variance) = calib {

                                        if Some(cid as u8) == id || id.is_none() {
                                            cli.write(|w|{
                                                w.write_fmt(format_args!("Gyro {} calibration complete with variance: {}",cid , variance))
                                                    .map_err(|_|ErrorKind::Other)
                                            }).expect("Failed to write to USB");
                                        }

                                    } else {
                                        if Some(cid as u8) == id || id.is_none() {
                                            cli.write(|w|{
                                                w.write_fmt(format_args!("Gyro {} calibration could not be completed",cid ))
                                                    .map_err(|_|ErrorKind::Other)
                                            }).expect("Failed to write to USB");
                                        } 
                                    }
                                }
                                completed = true;

                            }
                        }
                        
                        if completed {
                            cli.write(|w|{
                                ufmt::uwriteln!(w, "Gyro calibration complete")
                            }).expect("Failed to write to USB");
                        } else {
                            cli.write(|w|{
                                ufmt::uwriteln!(w, "Gyro calibration failed")
                            }).expect("Failed to write to USB");
                            msg::REQUEST_QUEUE.sender().send(CommanderRequest::CalGyrCommand(
                                crate::t_gyr_calibration::GyrCalCommand::Stop
                            )).await;
                        }

                        cli.set_prompt(PROMPT).unwrap();
                        
                    },
                    Some(Base::Cal { command: CalibrateCommand::Accel { seconds, variance, id } }) => {
                        // Handle gyro clibration
                    },
                    Some(Base::Cal { command: CalibrateCommand::Mag { variance, id } }) => {
                        // Handle gyro clibration
                    },
                    Some(_) => {defmt::warn!("{}: Unhandled response command", TASK_ID);}
                    _ => { },
                }
            },
        }

        let n = rx.read_packet(&mut buffer).await.unwrap();
        
        for byte in buffer[..n].iter() {        

            let process_res = cli.process_byte::<Base, _>(
                *byte,
                &mut Base::processor(|cli: &mut CliHandle<'_, _, _>, command| {
        
                    macro_rules! cli_error {
                        ($text:literal) => {
                            {               
                                ufmt::uwrite!(cli.writer(), "error: {}",$text)?;
                                defmt::error!("{}: {}", TASK_ID, $text);
                            }
                        };
                    }

                    match command {
                        
                        Base::Info => on_info(cli)?,

                        Base::Clear => cli.writer().write_str(CLEAR_SCREEN)?,

                        Base::Time => {
                            let time_float = Instant::now().as_millis() as f32 / 1e3;
                            write(cli.writer(), format_args!("Time is: {} seconds", time_float))?;
                        },

                        Base::Sensors => cli_error!("Command 'Sensors' not yet implemented"),

                        Base::Stream { command } => {
                            response_command = Some(Base::Stream { command });
                        },

                        Base::Txmap => {
                            if let Some(map) = msg::CFG_TRANSMITTER_MAP.try_get() {
                                ufmt::uwrite!(cli.writer(), "{}", map)?
                            } else {
                                ufmt::uwrite!(cli.writer(), "error: No transmitter map is available")?
                            }
                        }

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
                            cli_error!("Command 'Set' not yet implemented")
                        }

                        Base::Cal {command} => {
                            match command {
                                CalibrateCommand::Gyro { id, seconds, variance } => {
                                    response_command = Some(Base::Cal{command: CalibrateCommand::Gyro { seconds, variance, id } });
                                    request_option = Some(CommanderRequest::CalGyrCommand(
                                        crate::t_gyr_calibration::GyrCalCommand::Start( CalGyrConfig{
                                            sensor: id,
                                            duration: seconds,
                                            max_var: variance,
                                        }
                                    )))
                                },
                                CalibrateCommand::Accel { seconds, variance, id } => {
                                    response_command = Some(Base::Cal{command: CalibrateCommand::Accel { seconds, variance, id } });
                                    request_option = Some(CommanderRequest::StartAccCalib {
                                        sensor: id,
                                        duration: seconds.map(|x|x as f32),
                                        max_var: variance,
                                    })
                                },
                                CalibrateCommand::Mag { variance, id  } => {
                                    response_command = Some(Base::Cal{command: CalibrateCommand::Mag { variance, id } });
                                    request_option = Some(CommanderRequest::StartMagCalib {
                                        sensor: id,
                                        max_var: variance,
                                    })
                                },
                            }
                        },

                        Base::Config { command } => {
                            match command {
                                ConfigCommand::Save { index } => {
                                    ufmt::uwrite!(cli.writer(), "Saving configuration..")?;
                                    request_option = Some(CommanderRequest::SaveConfig { config_id: index });
                                }
                                ConfigCommand::Load { index } => {
                                    ufmt::uwrite!(cli.writer(), "Loading configuration..")?;
                                    request_option = Some(CommanderRequest::LoadConfig { config_id: index });
                                },
                                ConfigCommand::Show { config_to_show: _c } => cli_error!("Command 'Config Show' not yet implemented"),
                                ConfigCommand::Clear => {
                                    ufmt::uwrite!(cli.writer(), "Clearing configuration..")?;
                                    request_option = Some(CommanderRequest::ClearConfig)
                                },
                            }
                        }

                        Base::Blackbox { command } => {
                            match command {
                                BlackboxCommand::Enable { frequency, force } => {
                                    ufmt::uwrite!(cli.writer(), "Enabling logging..")?;
                                    defmt::info!("{}: Logging enabled", TASK_ID);
                                    request_option = Some(CommanderRequest::ConfigureLogging {
                                        frequency, enabled: Some(true), force_enable: force });
                                },
                                BlackboxCommand::Disable => {
                                    ufmt::uwrite!(cli.writer(), "Disabling logging..")?;
                                    defmt::info!("{}: Logging disabled", TASK_ID);
                                    request_option = Some(CommanderRequest::ConfigureLogging {
                                        frequency: None, enabled: Some(false), force_enable: None });
                                },
                                BlackboxCommand::SetRate { rate } => {
                                    ufmt::uwrite!(cli.writer(), "Setting logging rate..")?;
                                     defmt::info!("{}: Logging rate changed", TASK_ID);
                                    request_option = Some(CommanderRequest::ConfigureLogging {
                                        frequency: Some(rate), enabled: None , force_enable: None  })   
                                }

                            
                            BlackboxCommand::Print => cli_error!("Command 'Set' not yet implemented"),
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

async fn match_stream_command<'a, W, CB, HB, D>(
    command: &StreamCommand,
    cli: &mut Cli<W, ErrorKind, CB, HB>,
    rx: &mut Receiver<'static, D>,
    buffer: &mut [u8],
)
where
    W: Write<Error = ErrorKind>,
    CB: Buffer,
    HB: Buffer,
    D:  embassy_usb::driver::Driver<'static>,
{
 
    let mut handle_response_future = async || {
        match command {
            StreamCommand::ArmBlocker => {
                // Start stream of arm blocker flag
                stream(&msg::ARM_BLOCKER, cli, rx, buffer, true, 5, None, |att, w| {
                    w.write_fmt(format_args!("{:?}", att))
                        .map_err(|_|ErrorKind::Other)
                }).await
            },

            StreamCommand::Attitude{rate, newline, deg} => {

                let header = if *deg {
                    "time[ms], roll[deg], pitch[deg], yaw[deg]" 
                } else {
                    "time[ms], roll[rad], pitch[rad], yaw[rad]"
                };

                // Start stream of attitude data
                stream(&msg::ATTITUDE_EULER, cli, rx, buffer, *newline, *rate, Some(header), |mut att, w| {
                    if *deg { att = att.map(|x|x.to_degrees()); }
                    let time = Instant::now().as_millis();
                    w.write_fmt(format_args!("{}, {:9.5}, {:9.5}, {:9.5}", time, att.x, att.y, att.z))
                        .map_err(|_|ErrorKind::Other)
                }, ).await
            },

            StreamCommand::Gyro{rate, newline, sensor, deg} => {

                let header = if *deg {
                    "time[ms], roll[deg/s], pitch[deg/s], yaw[deg/s]" 
                } else {
                    "time[ms], roll[rad/s], pitch[rad/s], yaw[rad/s]"
                };

                if let Some(id) = sensor && *id as usize >= crate::N_IMU {
                    cli.write(|w|{
                        w.write_str(ERR_SENSOR_ID_OUT_OF_RANGE)
                    }).map_err(|_|ErrorKind::Other)?;
                    defmt::error!("{}: Sensor ID is out of range", TASK_ID);
                    return Err(ErrorKind::InvalidInput)
                }

                // Start stream of gyro
                stream(if let Some(id) = sensor {
                    &msg::IMU_SENSOR[*id as usize]
                } else {
                    &msg::IMU_DATA
                }, cli, rx, buffer, *newline, *rate, Some(header), |mut imu, w| {
                    if *deg { imu.gyr = imu.gyr.map(|x|x.to_degrees()); }
                    let time = Instant::now().as_millis();
                    w.write_fmt(format_args!("{}, {:9.4}, {:9.4}, {:9.4}", time, imu.gyr.x, imu.gyr.y, imu.gyr.z))
                        .map_err(|_|ErrorKind::Other)
                }, ).await
            },

            StreamCommand::Accel { rate, sensor, newline} => {
                
                let header = "time[ms], x[m/s^2], y[m/s^2], z[m/s^2]";

                if let Some(id) = sensor && *id as usize >= crate::N_IMU {
                    cli.write(|w|{
                        w.write_str(ERR_SENSOR_ID_OUT_OF_RANGE)
                    }).map_err(|_|ErrorKind::Other)?;
                    defmt::error!("{}: Sensor ID is out of range", TASK_ID);
                    return Err(ErrorKind::InvalidInput)
                }

                // Start stream of accel data
                stream(if let Some(id) = sensor {
                    &msg::IMU_SENSOR[*id as usize]
                } else {
                    &msg::IMU_DATA
                }, cli, rx, buffer, *newline, *rate, Some(header), |imu, w| {
                    let time = Instant::now().as_millis();
                    w.write_fmt(format_args!("{}, {:9.5}, {:9.5}, {:9.5}", time, imu.acc.x, imu.acc.y, imu.acc.z))
                        .map_err(|_|ErrorKind::Other)
                }).await
            },

            StreamCommand::Mag { rate , sensor, newline} => {
                let header = "time[ms], x[uTesla], y[uTesla], z[uTesla]";

                if let Some(id) = sensor && *id as usize >= crate::N_IMU {
                    cli.write(|w|{
                        w.write_str(ERR_SENSOR_ID_OUT_OF_RANGE)
                    }).map_err(|_|ErrorKind::Other)?;
                    defmt::error!("{}: Sensor ID is out of range", TASK_ID);
                    return Err(ErrorKind::InvalidInput)
                }

                // Print csv-formatted attitude data as degrees
                stream(if let Some(id) = sensor {
                    &msg::MAG_SENSOR[*id as usize]
                } else {
                    &msg::MAG_DATA
                }, cli, rx, buffer, *newline, *rate, Some(header), |imu, w| {
                    let time = Instant::now().as_millis();
                    w.write_fmt(format_args!("{}, {:9.5}, {:9.5}, {:9.5}", time, imu.x, imu.y, imu.z))
                        .map_err(|_|ErrorKind::Other)
                }).await
            },
        }
    };

    if let Err(e) = handle_response_future().await {
        defmt::error!("{}: Error in shell stream: {:?}", TASK_ID, defmt::Debug2Format(&e));
    }
}

/// Async function to enable streaming of data from the vehicle to the shell.
async fn stream<'a, T, W, CB, HB, D, F>(
    watch: &dyn WatchBehavior<T>,
    cli: &mut Cli<W, ErrorKind, CB, HB>,
    rx: &mut Receiver<'static, D>,
    buffer: &mut [u8],
    newline: bool,
    rate: u8,
    title: Option<&str>,
    mut writefn: F,
) -> Result<(), ErrorKind>
where
    F: FnMut(T, &mut Writer<'_,W, ErrorKind>) -> Result<(), ErrorKind>,
    T: Clone,
    W: Write<Error = ErrorKind>,
    CB: Buffer,
    HB: Buffer,
    D:  embassy_usb::driver::Driver<'static>,
{
    // Ensure rate is non-zero
    if rate == 0 { 
        cli.write(|w| {
            w.writeln_str("error: rate must be non-zero")
        })?;
        return Ok(());
    }

    // Remove prompt since we are awaiting a response.
    cli.set_prompt("")?;

    // Write title if available
    cli.write(|w| {
        w.writeln_str("")?;
        if let Some(title) = title {
            w.writeln_str(title)?;
        }
        if !newline {
            w.writeln_str("")?;
        }
        Ok(())
    })?;

    let mut ticker = Ticker::every(Duration::from_hz(rate as u64));

    loop {

        // Read a packet from the USB receiver (with timeout) to figure out if we should exit
        match with_timeout(Duration::from_millis(10), rx.read_packet(buffer)).await {
            Ok(Err(e)) => match e {
                embassy_usb::driver::EndpointError::BufferOverflow => return Err(embedded_io::ErrorKind::OutOfMemory),
                embassy_usb::driver::EndpointError::Disabled => return Err(embedded_io::ErrorKind::NotConnected),
            },

            // If any element in the buffer is a space character (ascii 32), break from the loop
            Ok(Ok(n)) if n > 0 => if buffer.iter().take(n).any(|e| e == &32 ) { break },
            _ => { /* timeout is expected here */ },
        }

        // Read value from watch, returning if not possible
        match watch.try_get_anon() {
            Some(data) => cli.write(|w| {
                if !newline { w.write_str("\x1B[1A\x1B[2K")?; }
                writefn(data, w)
            })?,
            None => {
                // Re-enable prompt
                cli.write(|w|{
                    w.write_str("\x1B[1A\x1B[2K")?;
                    w.write_str("error: No data available\n\n")
                })?;
                cli.set_prompt(PROMPT)?;
                return Err(embedded_io::ErrorKind::InvalidData)
            },
        }
        
        // Wait for the next tick
        ticker.next().await;
    }
    
    // End off with whitespace
    cli.write(|w| {
        w.writeln_str("")
    })?;

    // Re-enable prompt
    cli.set_prompt(PROMPT)
}


const CLEAR_SCREEN: &str = "\x1B[2J";

/// This looks scuffed since we need to use \\ to escape the backslashes in the string
const HOLSATUS_GRAPHIC: &str = 
"\x1B[32m   _   _       _           _
  | | | |     | |         | |
  | |_| | ___ | |___  __ _| |_ _   _ ___ 
  |  _  |/ _ \\| / __|/ _` | __| | | / __|
  | | | | (_) | \\__ \\ (_| | |_| |_| \\__ \\
  \\_| |_/\\___/|_|___/\\__,_|\\__|\\__,_|___/
\x1B[0m";

/// This is a thin wrapper around `core::fmt::write`. Mostly usedful to make formatting which is too
/// restrictive for the equivalent `ufmt` writers. Prefer those whenever possible.
fn write<W: core::fmt::Write>(output: &mut W , args: core::fmt::Arguments<'_>) -> Result<(),ErrorKind> {
    core::fmt::write(output, args).map_err(|_| ErrorKind::Other)
}

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

    /// Stream values live from the vehicle
    Stream {

        #[command(subcommand)]
        command: StreamCommand,
    },

    /// Show the current transmitter map
    Txmap,

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

    /// Interact with the blackbox logger of the vehicle
    Blackbox {

        #[command(subcommand)] 
        command: BlackboxCommand,
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
    Save {

        /// The index of the configuration to save. If none is provided, the active configuration is saved.
        index: Option<u8>
    },

    /// Load the config from flash storage. This overwrites any unsaved changes.
    Load {

        /// The index of the configuration to load. If none is provided, the active configuration is reloaded.
        index: Option<u8>
    },

    /// Show the current configuration in the shell
    Show {
        config_to_show: Option<&'a str>
    },

    /// Clear the flash storage of all configuration data
    Clear,
}


#[derive(Command)]
enum BlackboxCommand {

    /// Enable blackbox logging
    Enable {

        /// Frequency at which to log data (default 50 Hz)
        #[arg(short = 'f', long)]
        frequency: Option<u16>,

        /// Force-enable logging, even if the vehicle is not armed
        #[arg(long)]
        force: Option<bool>,
    },

    /// Disable blackbox logging
    Disable,

    /// Set the frequency at which data is logged.
    SetRate {
        rate: u16
    },
        
    /// Print the latest log entry to the shell
    Print,
}


#[derive(Command)]
enum CalibrateCommand {

    /// Calibrate the gyroscopes
    Gyro {

        /// Number of seconds to run the calibration for
        #[arg(short = 's', long)]
        seconds: Option<u8>,

        /// Upper bound for allowed variance of the accelerometer
        #[arg(short = 'v', long)]
        variance: Option<f32>,
            
        /// The ID of the gyroscope to calibrate. If none is provided, all gyros are calibrated.
        id: Option<u8>,
    },

    /// Calibrate the accelerometers
    Accel {

        /// Number of seconds to run the calibration for
        #[arg(short = 's', long)]
        seconds: Option<u8>,

        /// Upper bound for allowed variance of the accelerometer
        #[arg(short = 'v', long)]
        variance: Option<f32>,
            
        /// The ID of the accelerometer to calibrate. If none is provided, all accelerometers are calibrated.
        id: Option<u8>,
    },

    /// Calibrate the magnetometers
    Mag {

        /// Upper bound for allowed variance of the fitting after calibration
        #[arg(short = 'v', long)]
        variance: Option<f32>,
                
        /// The ID of the magnetometer to calibrate. If none is provided, all magnetometers are calibrated.
        id: Option<u8>,
    }

}

#[derive(Command)]
enum StreamCommand {

    /// Stream the arming blocker flag to the shell
    ArmBlocker,

    /// Stream attitude data to the shell
    Attitude {
        
        /// Stream the data in CSV format
        #[arg(short = 'n', long, default_value = "false")]
        newline: bool,

        /// Use degrees instead of radians
        #[arg(short = 'd', long, default_value = "false")]
        deg: bool,

        /// Rate at which to stream the data (default 10 Hz)
        #[arg(short = 'r', long, default_value = "10")]
        rate: u8,
    },

    /// Stream gyroscope data to the shell
    Gyro {
        
        /// Stream the data in CSV format
        #[arg(short = 'n', long, default_value = "false")]
        newline: bool,

        /// Use degrees instead of radians
        #[arg(short = 'd', long, default_value = "false")]
        deg: bool,

        /// ID of the sensor to use (defaults to active sensor)
        #[arg(short = 's', long)]
        sensor: Option<u8>,

        /// Rate at which to stream the data (default 10 Hz)
        #[arg(short = 'r', long, default_value = "10")]
        rate: u8,
    },

    /// Stream accelerometer data to the shell
    Accel {
        
        /// Stream the data in CSV format
        #[arg(short = 'n', long, default_value = "false")]
        newline: bool,

        /// ID of the sensor to use (defaults to active sensor)
        #[arg(short = 's', long)]
        sensor: Option<u8>,

        /// Rate at which to stream the data (default 10 Hz)
        #[arg(short = 'r', long, default_value = "10")]
        rate: u8,
    },

    /// Stream magnetometer data to the shell
    Mag {

        /// Stream the data in CSV format
        #[arg(short = 'n', long, default_value = "false")]
        newline: bool,

        /// Id of the sensor to use (defaults to active sensor)
        #[arg(short = 's', long)]
        sensor: Option<u8>,

        /// Rate at which to stream the data (default 10 Hz)
        #[arg(short = 'r', long, default_value = "10")]
        rate: u8,
    }
}

async fn dtr_ready<'d, D: embassy_usb_driver::Driver<'d>>(rx: &embassy_usb::class::cdc_acm::Receiver<'d, D>) {
    poll_fn(|cx|{
        if rx.dtr() {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }).await
}