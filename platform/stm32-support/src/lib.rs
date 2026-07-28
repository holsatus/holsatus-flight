#![no_std]

pub mod block_device;
pub mod dshot_driver;
pub mod setup_macros;

#[cfg(feature = "stm32f405rg")]
pub fn default_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();

    use embassy_stm32::rcc::*;
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32::time::Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL168,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV7),
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;

    config
}

/// Define the clock configuration for the board.
#[cfg(feature = "stm32h743vi")]
pub fn default_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();

    use embassy_stm32::rcc::*;
    config.rcc.hsi = Some(HSIPrescaler::DIV1);
    config.rcc.csi = true;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL50,
        fracn: None,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV4),
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
    config.rcc.voltage_scale = VoltageScale::Scale1;

    config
}
