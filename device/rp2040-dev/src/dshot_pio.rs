pub use super::DshotPioTrait;
use dshot_encoder as dshot;
use paste::paste;

use embassy_rp::{
    interrupt::typelevel::Binding,
    pio::{Config, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection::{Right, Left}},
    Peripheral,
};
#[allow(dead_code)]
pub struct DshotPio<'a, const N: usize, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
}

fn configure_pio_instance<'a, PIO: Instance>(
    pio: impl Peripheral<P = PIO> + 'a,
    irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
    clk_div: (u16, u8),
) -> (Config<'a, PIO>, Pio<'a, PIO>) {
    // Define program
    let dshot_pio_program = pio_proc::pio_asm!(
        "set pindirs, 1",
        "entry:"
        "   pull"
        "   out null 16"
        "   set x 15"
        "loop:"
        "   set pins 1"
        "   out y 1"
        "   jmp !y zero"
        "   nop [2]"
        "one:" // 6 and 2
        "   set pins 0"
        "   jmp x-- loop"
        "   jmp reset"
        "zero:" // 3 and 5
        "   set pins 0 [3]"
        "   jmp x-- loop"
        "   jmp reset"
        "reset:" // Blank frame
        "   nop [31]"
        "   nop [31]"
        "   nop [31]"
        "   jmp entry [31]"
    );

    // Configure program
    let mut cfg = Config::default();
    let mut pio = Pio::new(pio, irq);
    cfg.use_program(&pio.common.load_program(&dshot_pio_program.program), &[]);
    cfg.clock_divider = clk_div.0.into();

    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        direction: Right,
        threshold: 32,
    };

    cfg.shift_out = ShiftConfig {
        auto_fill: Default::default(),
        direction: Left,
        threshold: Default::default(),
    };

    (cfg, pio)
}

macro_rules! impl_dshot_pio_new {
    ($num:literal: {$($sm:literal),+}) => {
        paste!{
            impl<'a, PIO: Instance> DshotPio<'a, $num, PIO> {
                /// Construct a new DshotPio instance
                pub fn new(
                    pio: impl Peripheral<P = PIO> + 'a,
                    irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
                    $([<pin $sm>]: impl PioPin),+,
                    clk_div: (u16, u8),
                ) -> DshotPio<'a, $num, PIO> {
                    let (mut cfg, mut pio) = configure_pio_instance(pio, irq, clk_div);

                    // Set pins and enable all state machines
                    $(
                        let pin = pio.common.make_pio_pin(paste!{[<pin $sm>]});
                        cfg.set_set_pins(&[&pin]);
                        pio.[<sm $sm>].set_config(&cfg);
                        pio.[<sm $sm>].set_enable(true);
                    )+

                    // Return struct of configured DShot state machines
                    DshotPio { pio_instance: pio }
                }
            }

            impl<'d, PIO: Instance> super::DshotPioTrait<$num> for DshotPio<'d, $num, PIO> {
                /// Send any valid DShot value to the ESC.
                fn command(&mut self, command: [u16; $num]) {
                    $(
                        self.pio_instance.[<sm $sm>].tx().push(command[$sm].min(dshot::THROTTLE_MAX) as u32);
                    )+
                }
            }
        }

    };
}

impl_dshot_pio_new!(1: {0});
impl_dshot_pio_new!(2: {0, 1});
impl_dshot_pio_new!(3: {0, 1, 2});
impl_dshot_pio_new!(4: {0, 1, 2, 3});
