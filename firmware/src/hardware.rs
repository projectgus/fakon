// "Board level" hardware abstractions, ie pin assignments, etc.

use can_bit_timings;
use defmt::info;
use fdcan::ConfigMode;
use fdcan::FdCan;
use fugit::RateExtU32;
use hal::gpio::gpioa;
use hal::gpio::gpiob;
use hal::gpio::gpioc;
use hal::gpio::Floating;
use hal::gpio::Input;
use hal::gpio::Output;
use hal::gpio::PushPull;
use inverted_pin::InvertedPin;
use stm32g4xx_hal::gpio::gpiod;
use stm32g4xx_hal::gpio::ExtiPin;
use stm32g4xx_hal::gpio::SignalEdge;
use stm32g4xx_hal as hal;
use stm32g4xx_hal::can::CanExt;
use stm32g4xx_hal::gpio::GpioExt;
use stm32g4xx_hal::gpio::Speed;
use stm32g4xx_hal::pwr::PwrExt;
use stm32g4xx_hal::syscfg::SysCfgExt;
use stm32g4xx_hal::rcc;
use stm32g4xx_hal::rcc::{PllConfig, RccExt};
use stm32g4xx_hal::stm32;

// Type aliases for hardware peripherals
pub type PCAN = hal::can::Can<hal::stm32::FDCAN1>;

// Type aliases for I/O pins
pub type AcuCrashOutput = InvertedPin<gpioa::PA4<Output<PushPull>>>;

// NOTE: For final hardware this should be a WKUP pin
pub type IG1OnInput = gpioc::PC9<Input<Floating>>;

pub type BrakeInput = gpiob::PB8<Input<Floating>>;

pub type EVReadyInput = InvertedPin<gpiob::PB9<Input<Floating>>>;

pub type RelayIG3Output = gpiob::PB6<Output<PushPull>>;

pub type LEDIgnitionOutput = gpiob::PB10<Output<PushPull>>;

pub type ScuParkTx = InvertedPin<gpioc::PC3<Output<PushPull>>>;

pub type ScuParkRx = gpiod::PD2<Input<Floating>>;

// Currently modelled as an H-Bridge "on" pin
pub type ChargeLockDriveOutput = gpioc::PC5<Output<PushPull>>;

// Currently modelled as an H-Bridge "direction" pin
pub type ChargeLockDirOutput = InvertedPin<gpiob::PB0<Output<PushPull>>>;

pub type ChargeLockSensorInput = gpioc::PC11<Input<Floating>>;

// Struct to encompass all the board resources, as their functions
pub struct Board {
    pub pcan_config: FdCan<PCAN, ConfigMode>,
    pub srs_crash_out: AcuCrashOutput,
    pub can_timing_500kbps: can_bit_timings::CanBitTiming,
    pub brake_input: BrakeInput,
    pub ig1_on_input: IG1OnInput,
    pub relay_ig3: RelayIG3Output,
    pub led_ignition: LEDIgnitionOutput,
    pub ev_ready: EVReadyInput,
    pub scu_park_tx: ScuParkTx,
    pub scu_park_rx: ScuParkRx,
    pub charge_lock_drive: ChargeLockDriveOutput,
    pub charge_lock_dir: ChargeLockDirOutput,
    pub charge_lock_sensor: ChargeLockSensorInput,
}

// Systick Based Timer
pub const MONOTONIC_FREQUENCY: u32 = 1_000;
rtic_monotonics::systick_monotonic!(Mono, MONOTONIC_FREQUENCY);

// Hardware init function
pub fn init(core: cortex_m::Peripherals, mut dp: stm32::Peripherals) -> Board {
    info!("hardware init");

    let mut syscfg = dp.SYSCFG.constrain();
    let rcc = dp.RCC.constrain();

    // Sysclock is based on PLL_R
    let pll_config = PllConfig {
        mux: rcc::PllSrc::HSE(24_u32.MHz()), // Nucleo board X3 OSC
        n: rcc::PllNMul::MUL_32,
        m: rcc::PllMDiv::DIV_3,       // f(vco) = 24MHz*32/3 = 256MHz
        r: Some(rcc::PllRDiv::DIV_2), // f(sysclock) = 256MHz/2 = 128MHz
        q: None,
        p: None,
    };

    let clock_config = rcc::Config::default()
        .pll_cfg(pll_config)
        .clock_src(rcc::SysClockSrc::PLL)
        .ahb_psc(rcc::Prescaler::NotDivided)
        .apb1_psc(rcc::Prescaler::Div2)
        .apb2_psc(rcc::Prescaler::Div2);

    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(clock_config, pwr);

    // After clock configuration, the following should be true:
    // Sysclock is 128MHz
    // AHB clock is 128MHz
    // APB1 clock is 64MHz
    // APB2 clock is 64MHz

    Mono::start(core.SYST, rcc.clocks.sys_clk.to_Hz());

    unsafe {
        let flash = &(*stm32::FLASH::ptr());
        flash.acr.modify(|_, w| {
            w.latency().bits(0b1000) // 8 wait states
        });
    }

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    assert!(rcc.clocks.apb1_clk.to_MHz() == 64); // Macro requires literal
    let can_timing_500kbps = can_bit_timings::can_timings!(64.mhz(), 500.khz());

    // CAN1
    let can1_config = {
        let rx = gpioa.pa11.into_alternate().set_speed(Speed::VeryHigh);
        let tx = gpioa.pa12.into_alternate().set_speed(Speed::VeryHigh);
        dp.FDCAN1.fdcan(tx, rx, &rcc)
    };

    // CAN2
    let _can2_config = {
        let rx = gpiob.pb12.into_alternate().set_speed(Speed::VeryHigh);
        let tx = gpiob.pb13.into_alternate().set_speed(Speed::VeryHigh);
        dp.FDCAN2.fdcan(tx, rx, &rcc)
    };

    // CAN3
    let _can3_config = {
        let rx = gpiob.pb3.into_alternate().set_speed(Speed::VeryHigh);
        let tx = gpiob.pb4.into_alternate().set_speed(Speed::VeryHigh);
        dp.FDCAN3.fdcan(tx, rx, &rcc)
    };

    // GPIOs from the dev board assignments

    // Signal Inputs
    let pin_in1 = gpioc.pc9; // 12V
    let pin_in2 = gpiob.pb8; // 12V
    let pin_in3 = gpiob.pb9; // 12V
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
    let pin_in15 = gpioc.pc11; // 5V
    let _pin_in16 = gpioc.pc10; // 5V

    // Signal outputs
    let pin_out1 = gpioa.pa4; // 12V, TIM3_CH2
    let pin_out2 = gpiob.pb0; // 12V, TIM3_CH3
    let _pin_out3 = gpioc.pc1; // 12V, TIM1_CH2
    let _pin_out4 = gpioc.pc0; // 12V, TIM1_CH1
    let pin_out5 = gpioc.pc3; // 5V, TIM1_CH4
    let _pin_out6 = gpioc.pc2; // 5V, TIM1_CH3
    let _pin_out7 = gpioa.pa1; // 5V, TIM2_CH2 or TIM5_CH2

    // Relay coil drivers
    let pin_coil_l1 = gpiob.pb6;
    let _pin_coil_l2 = gpioc.pc4; // also LED4, oops
    let pin_coil_h = gpioc.pc5;

    // LEDs, all green, all active high
    let led1 = gpiob.pb10.into_push_pull_output();
    let _led2 = gpiob.pb5.into_push_pull_output();
    let _led3 = gpioa.pa10.into_push_pull_output();
    // led4 accidentally shared with pin_coil_l2

    // Functions assigned to pins

    // OUT5 => SCU Park TX (5V 10Hz soft PWM)
    // (inverted due to drive FET)
    let scu_park_tx = InvertedPin::new(pin_out5.into_push_pull_output());

    // IN13 => SCU Park RX (5V 10Hz PWM via interrupt)
    let mut scu_park_rx = pin_in13.into_floating_input();
    scu_park_rx.make_interrupt_source(&mut syscfg);
    scu_park_rx.trigger_on_edge(&mut dp.EXTI, SignalEdge::RisingFalling);
    scu_park_rx.enable_interrupt(&mut dp.EXTI);

    // OUT1 => SRS Crash signal, 50Hz soft PWM
    // Inverted as MCU pin drives a FET gate for open drain output
    let srs_crash_out = InvertedPin::new(pin_out1.into_push_pull_output());

    // IN1 => IG1 ignition on input
    let ig1_on_input = pin_in1.into_floating_input();

    // IN2 => Brake pedal input signal
    let brake_input = pin_in2.into_floating_input();

    // IN3 => EV Ready input (active low)
    let ev_ready = InvertedPin::new(pin_in3.into_floating_input());

    // RELAY L1 - IG3 relay coil enable
    let relay_ig3 = pin_coil_l1.into_push_pull_output();

    // LED 1 - Ignition Status
    let led_ignition = led1.into_push_pull_output();

    // OUT2 => Charge Lock Direction output
    // Inverted as MCU drives FET gate
    let charge_lock_dir = InvertedPin::new(pin_out2.into_push_pull_output());

    // Hardware design flaw: I planned to use high and low relay coil drivers for
    // the charge port but misunderstood the drive design. Still using high side driver
    // here, it's overkill but has the property of being low on reset!
    let charge_lock_drive = pin_coil_h.into_push_pull_output();

    // IN15 => Charge Port Lock input, no pullup, 5K Pulldown.
    let charge_lock_sensor = pin_in15.into_floating_input();

    Board {
        pcan_config: can1_config,
        srs_crash_out,
        can_timing_500kbps,
        ig1_on_input,
        relay_ig3,
        led_ignition,
        brake_input,
        ev_ready,
        scu_park_tx,
        scu_park_rx,
        charge_lock_drive,
        charge_lock_dir,
        charge_lock_sensor,
    }
}
