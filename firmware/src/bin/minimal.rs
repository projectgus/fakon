#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use stm32g4xx_hal as _; // global logger + panicking-behavior + memory layout
use fakon as _;

#[rtic::app(
    device = stm32g4xx_hal::stm32,
    dispatchers = [USBWAKEUP]
)]
mod app {
    use fugit::RateExtU32;
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::rcc::{PllConfig, RccExt};

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let dp = cx.device;
        let cp = cx.core;

        let rcc = dp.RCC.constrain();
        let mut pll_config = PllConfig::default();

        // Sysclock is based on PLL_R
        pll_config.mux = stm32g4xx_hal::rcc::PllSrc::HSE(24_u32.MHz()); // Nucleo board X3 OSC
        pll_config.n = stm32g4xx_hal::rcc::PllNMul::MUL_32;
        pll_config.m = stm32g4xx_hal::rcc::PllMDiv::DIV_3; // f(vco) = 24MHz*32/3 = 256MHz
        pll_config.r = Some(stm32g4xx_hal::rcc::PllRDiv::DIV_2); // f(sysclock) = 256MHz/2 = 128MHz

        let clock_config = stm32g4xx_hal::rcc::Config::default()
            .pll_cfg(pll_config)
            .clock_src(stm32g4xx_hal::rcc::SysClockSrc::PLL);

        let pwr = dp.PWR.constrain().freeze();
        let rcc = rcc.freeze(clock_config, pwr);

        // After clock configuration, the following should be true:
        // Sysclock is 128MHz
        // AHB clock is 128MHz
        // APB1 clock is 128MHz
        // APB2 clock is 128MHz
        // The ADC will ultimately be put into synchronous mode and will derive
        // its clock from the AHB bus clock, with a prescalar of 2 or 4.

        unsafe {
            let flash = &(*stm32g4xx_hal::stm32::FLASH::ptr());
            flash.acr.modify(|_, w| {
                w.latency().bits(0b1000) // 8 wait states
            });
        }

        // TODO setup monotonic if used
        // let sysclk = { /* clock setup + returning sysclk as an u32 */ };
        // let token = rtic_monotonics::create_systick_token!();
        // rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);


        task1::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1)]
    async fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }
}
