#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use stm32g4xx_hal as hal; // global logger + panicking-behavior + memory layout
use fakon as _;

// Type aliases for hardware functions, to move into a hardware module
type PCAN = hal::can::Can<hal::stm32::FDCAN1>;
type PwmSrsCrashOutput = hal::gpio::gpioa::PA4::<hal::gpio::Output<hal::gpio::PushPull>>;

#[rtic::app(
    device = stm32g4xx_hal::stm32,
    dispatchers = [USBWAKEUP, COMP1_2_3, COMP4_5_6, COMP7, SAI, I2C4_EV, I2C4_ER]
)]
mod app {
    use can_bit_timings;
    use crate::{PCAN, PwmSrsCrashOutput};
    use fakon::can_queue;
    use fakon::can_queue::Tx;
    use fugit::{ExtU32, RateExtU32};
    use hex_literal::hex;
    use stm32g4xx_hal::can::CanExt;
    use stm32g4xx_hal::gpio::Speed;
    use stm32g4xx_hal::gpio::GpioExt;
    use stm32g4xx_hal::prelude::OutputPin;
    use stm32g4xx_hal::pwm::PwmExt;
    use stm32g4xx_hal::pwr::PwrExt;
    use stm32g4xx_hal::rcc::{PllConfig, RccExt};
    use stm32g4xx_hal::stm32;
    use stm32g4xx_hal::rcc;
    use rtic_monotonics;
    use rtic_monotonics::Monotonic;

    pub const MONOTONIC_FREQUENCY: u32 = 1_000;
    rtic_monotonics::systick_monotonic!(Systick, MONOTONIC_FREQUENCY);

    // Shared resources go here
    #[shared]
    struct Shared {
        pcan_tx: can_queue::TxQueue<PCAN>,
    }

    // Local resources go here
    #[local]
    struct Local {
        srs_crash_out: PwmSrsCrashOutput,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let dp = cx.device;

        let rcc = dp.RCC.constrain();
        let mut pll_config = PllConfig::default();

        // Sysclock is based on PLL_R
        pll_config.mux = rcc::PllSrc::HSE(24_u32.MHz()); // Nucleo board X3 OSC
        pll_config.n = rcc::PllNMul::MUL_32;
        pll_config.m = rcc::PllMDiv::DIV_3; // f(vco) = 24MHz*32/3 = 256MHz
        pll_config.r = Some(rcc::PllRDiv::DIV_2); // f(sysclock) = 256MHz/2 = 128MHz

        let clock_config = rcc::Config::default()
            .pll_cfg(pll_config)
            .clock_src(rcc::SysClockSrc::PLL)
            .ahb_psc(rcc::Prescaler::Div2)
            .apb1_psc(rcc::Prescaler::Div2)
            .apb2_psc(rcc::Prescaler::Div2);

        let pwr = dp.PWR.constrain().freeze();
        let mut rcc = rcc.freeze(clock_config, pwr);

        // After clock configuration, the following should be true:
        // Sysclock is 128MHz
        // AHB clock is 64MHz
        // APB1 clock is 64MHz
        // APB2 clock is 64MHz

        unsafe {
            let flash = &(*stm32::FLASH::ptr());
            flash.acr.modify(|_, w| {
                w.latency().bits(0b1000) // 8 wait states
            });
        }

        //let token = rtic_monotonics::create_systick_token!();
        //Systick::new(cx.core.SYST, rcc.clocks.sysclk, token);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);
        let gpiod = dp.GPIOD.split(&mut rcc);

        // Based on 64MHz APB1 (see above)
        let can_timing_500kbps = can_bit_timings::can_timings!(64.mhz(), 500.khz());

        // CAN1
        let (_can1_rx, _can1_receiver, can1_tx) = {
            let rx = gpioa.pa11.into_alternate().set_speed(Speed::VeryHigh);
            let tx = gpioa.pa12.into_alternate().set_speed(Speed::VeryHigh);
            let can = dp.FDCAN1.fdcan(tx, rx, &rcc);
            can_queue::init(can, &can_timing_500kbps)
        };

        // CAN2
        let (_can2_rx, _can2_receiver, _can2_tx) = {
            let rx = gpiob.pb12.into_alternate().set_speed(Speed::VeryHigh);
            let tx = gpiob.pb13.into_alternate().set_speed(Speed::VeryHigh);
            let can = dp.FDCAN2.fdcan(tx, rx, &rcc);
            can_queue::init(can, &can_timing_500kbps)
        };

        // CAN3
        let (_can3_rx, _can3_receiver, _can3_tx) = {
            let rx = gpiob.pb3.into_alternate().set_speed(Speed::VeryHigh);
            let tx = gpiob.pb4.into_alternate().set_speed(Speed::VeryHigh);
            let can = dp.FDCAN3.fdcan(tx, rx, &rcc);
            can_queue::init(can, &can_timing_500kbps)
        };


        // GPIOs from the dev board assignment
        // TODO: abstract this into a hardware module somehow

        // Signal Inputs
        let _pin_in1 = gpioc.pc9; // 12V
        let _pin_in2 = gpiob.pb8; // 12V
        let _pin_in3 = gpiob.pb9; // 12V
        let _pin_in4 = gpioa.pa5; // 12V
        let _pin_in5 = gpioa.pa6; // 12V
        let _pin_in6 = gpioa.pa7; // 12V
        let _pin_in7 = gpioc.pc7; // 12V
        let _pin_in8 = gpioa.pa9; // 12V
        let _pin_in9 = gpioa.pa8; // 12V
        let _pin_in10 = gpioa.pa0; // 12V
        let _pin_in11 = gpiob.pb7; // 12V
        let _pin_in12 = gpioa.pa15; // 12V
        let pin_in13 = gpiod.pd2; // 5V
        let _pin_in14 = gpioc.pc12; // 5V
        let _pin_in15 = gpioc.pc11; // 5V
        let _pin_in16 = gpioc.pc10; // 5V

        // Signal outputs
        let pin_out1 = gpioa.pa4; // 12V, TIM3_CH2
        let _pin_out2 = gpiob.pb0; // 12V, TIM3_CH3
        let _pin_out3 = gpioc.pc1; // 12V, TIM1_CH2
        let _pin_out4 = gpioc.pc0; // 12V, TIM1_CH1
        let pin_out5 = gpioc.pc3; // 5V, TIM1_CH4
        let _pin_out6 = gpioc.pc2; // 5V, TIM1_CH3
        let _pin_out7 = gpioa.pa1; // 5V, TIM2_CH2 or TIM5_CH2

        // Relay coil drivers
        let _pin_coil_l1 = gpiob.pb6;
        let _pin_coil_l2 = gpioc.pc4; // also LED4, oops
        let _pin_coil_h = gpioc.pc5;

        // LEDs, all green, all active high
        let _led1 = gpiob.pb10.into_push_pull_output();
        let _led2 = gpiob.pb5.into_push_pull_output();
        let _led3 = gpioa.pa10.into_push_pull_output();
        // led4 accidentally shared with pin_coil_l2

        // Functions assigned to pins

        // CAN1 => Kona PCAN
        let pcan_tx = can1_tx;

        // OUT5 => SCU Park TX PWM
        let _pwm_scu_park_tx = {
            let pin = pin_out5.into_alternate();
            let pwm = dp.TIM1.pwm(pin, 1000.Hz(), &mut rcc);
            pwm
        };

        // IN13 => SCU Park RX (soft PWM)
        let _scu_park_rx = pin_in13;

        // OUT1 => SRS Crash signal, 50Hz soft PWM
        let srs_crash_out = pin_out1.into_push_pull_output();

        task1::spawn().ok();

        (
            Shared { pcan_tx },
            Local {
                srs_crash_out,
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

    #[task(shared = [pcan_tx], local = [srs_crash_out], priority = 3)]
    async fn srscm(mut cx: srscm::Context) {
        // SRS/Airbag control system task. Currently very simple:
        // 1Hz CAN message (constant)
        // 50Hz PWM output, 80% high for "not crashed"
        let can_1hz = can_queue::QueuedFrame::new_std(0x5a0, &hex!("000000C025029101"));
        let duty_pct = 80;
        // TODO: calculate this from a Rate in a less icky way
        //let cycle_time = 50.Hz().try_into_duration().unwrap();
        //let time_high = Systick::delay(cycle_time * duty_pct / 100);
        //let time_low = Systick::delay(cycle_time * (100 - duty_pct) / 100);
        let time_high = (20_u32 * duty_pct / 100).millis();
        let time_low = (20_u32 * duty_pct / 100).millis();

        let crash_out = &mut cx.local.srs_crash_out;

        loop {
            // Every 1Hz
            cx.shared.pcan_tx.lock(|tx| tx.transmit(&can_1hz));

            for _ in 0..50 {
                crash_out.set_low().unwrap();
                Systick::delay(time_low).await;
                crash_out.set_high().unwrap();
                Systick::delay(time_high).await;
            }
        }
    }

    // #[task(binds = USB_HP_CAN_TX, shared = [can_tx], priority = 5)]
    // fn can1_tx(mut cx: can1_tx::Context) {
    //     cx.shared.can_tx.lock(|can_tx| can_tx.on_tx_irq());
    // }

    // #[task(binds = USB_LP_CAN_RX0, local = [can_rx], priority = 5)]
    // fn can1_rx(cx: can1_rx::Context) {
    //     cx.local.can_rx.on_rx_irq();
    // }

}
