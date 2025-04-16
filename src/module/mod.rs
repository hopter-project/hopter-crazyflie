pub mod accel_gyro;
pub mod controller;
#[allow(unused)]
pub mod crtp;
pub mod estimator;
pub mod filter;
pub mod flow;
pub mod led;
pub mod motor;
pub mod radio_link;
pub mod stabilizer;
mod test;
pub mod timestamp;
pub mod tof;

use self::{
    accel_gyro::{AccelGyro, I2C3DmaIrq},
    crtp::SystemPacket,
    estimator::Estimator,
    flow::Flow,
    radio_link::{NrfUartDmaIrq, RadioRxLink, RadioTxLink},
    stabilizer::Stabilizer,
    timestamp::Tim7Irq,
    tof::{I2C1DmaIrq, ToF},
};
use crate::driver::serial::{
    i2c::{I2c1DMAHandle, I2c3DMAHandle},
    usart::NRFSerialDMAHandle,
};
use alloc::sync::Arc;
use cfg_if::cfg_if;
use cortex_m::peripheral::NVIC;
use hopter::{
    config,
    interrupt::mask::MaskableIrq,
    sync::{
        self, Mutex, MutexGuard, MutexIrqSafe, MutexIrqSafeGuard, Semaphore, {Consumer, Producer},
    },
};
use led::LEDs;
use motor::Motor;
use owning_ref::OwningRefMut;
use spin::Once;
use stm32f4xx_hal::{
    dma::{StreamX, StreamsTuple},
    gpio::{Alternate, Input, Output, Pin, PushPull, Speed, PB4},
    pac::{self, DMA1, DMA2, I2C1, I2C3, RCC, SPI1, TIM2, TIM4, TIM7, USART6},
    prelude::*,
    rcc::{Clocks, Rcc, RccExt},
    serial::{config::DmaConfig, Config},
    spi::{self, Spi},
};

cfg_if! { if #[cfg(feature = "irq_latency_test")] {

use stm32f4xx_hal::{
    syscfg::SysCfg,
    gpio::Edge,
    pac::EXTI,
};

}} // end if #[cfg(feature = "irq_latency_test")]

cfg_if! { if #[cfg(not(feature = "no_dbg_print"))] {

use crate::debug::{self, DebugString, DebugUsartDmaIrq};
use crate::driver::serial::usart::DebugSerialDMAHandle;
use stm32f4xx_hal::pac::USART3;

}} // end if #[cfg(not(feature = "no_dbg_print"))]

type Spi1Handle = Spi<SPI1, false>;
type Spi1Cs = PB4<Output<PushPull>>;

static LEDS: Once<LEDs> = Once::new();
static SYS_CLOCKS: Mutex<Option<Clocks>> = Mutex::new(None);
static NRF_USART: MutexIrqSafe<Option<NRFSerialDMAHandle>, NrfUartDmaIrq> = MutexIrqSafe::new(None);
static SEM_NRF_USART: Semaphore = Semaphore::new(1, 0);

static MOTOR: Mutex<Option<Motor>> = Mutex::new(None);
static SENSOR_ACCEL_GYRO: Mutex<Option<AccelGyro>> = Mutex::new(None);
static SENSOR_TOF: Mutex<Option<ToF>> = Mutex::new(None);
static SENSOR_FLOW: Mutex<Option<Flow>> = Mutex::new(None);
static TIM7_HANDLE: MutexIrqSafe<Option<TIM7>, Tim7Irq> = MutexIrqSafe::new(None);

static ESTIMATOR: Mutex<Option<Estimator>> = Mutex::new(None);
static STABILIZER: Mutex<Option<Arc<Stabilizer>>> = Mutex::new(None);
static SEM_STABILIZER: Semaphore = Semaphore::new(1, 0);

static RADIO_RX_LINK: MutexIrqSafe<Option<RadioRxLink>, NrfUartDmaIrq> = MutexIrqSafe::new(None);

static RADIO_TX_LINK: Mutex<Option<RadioTxLink>> = Mutex::new(None);

static I2C3_DMA_HANDLE: MutexIrqSafe<Option<I2c3DMAHandle>, I2C3DmaIrq> = MutexIrqSafe::new(None);
static I2C1_DMA_HANDLE: MutexIrqSafe<Option<I2c1DMAHandle>, I2C1DmaIrq> = MutexIrqSafe::new(None);
static SEM_I2C3: Semaphore = Semaphore::new(1, 0);
static SEM_I2C1: Semaphore = Semaphore::new(1, 0);

static SEM_SYSTEM_START_WAIT: Semaphore = Semaphore::new(3, 0);

static SPI1_HANDLE: Mutex<Option<Spi1Handle>> = Mutex::new(None);
static SPI1_CS: Mutex<Option<Spi1Cs>> = Mutex::new(None);

static RADIO_TX_PRODUCER: Once<Producer<SystemPacket, 16>> = Once::new();
static RADIO_TX_CONSUMER: Once<Consumer<SystemPacket, 16>> = Once::new();

cfg_if! { if #[cfg(not(feature = "no_dbg_print"))] {

static DEBUG_USART_DMA_HANDLE: MutexIrqSafe<Option<DebugSerialDMAHandle>, DebugUsartDmaIrq> =
    MutexIrqSafe::new(None);
static SEM_DEBUG_USART: Semaphore = Semaphore::new(1, 0);

static DEBUG_PRINT_PRODUCER: Once<Producer<DebugString, 32>> = Once::new();
static DEBUG_PRINT_CONSUMER: Once<Consumer<DebugString, 32>> = Once::new();
}} // end if #[cfg(not(feature = "no_dbg_print"))]

fn init_clock(rcc: Rcc) -> Clocks {
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();
    return clocks;
}

fn init_leds(
    pc0: Pin<'C', 0, Input>,
    pc1: Pin<'C', 1, Input>,
    pc2: Pin<'C', 2, Input>,
    pc3: Pin<'C', 3, Input>,
    pd2: Pin<'D', 2, Input>,
) {
    let left_red_led = pc0.into_push_pull_output();
    let right_red_led = pc3.into_push_pull_output();
    let left_green_led = pc1.into_push_pull_output();
    let right_green_led = pc2.into_push_pull_output();
    let left_blue_led = pd2.into_push_pull_output();

    LEDS.call_once(|| {
        LEDs::new(
            left_red_led,
            right_red_led,
            left_green_led,
            right_green_led,
            left_blue_led,
        )
    });
}

fn init_usec_timestamp(tim7: TIM7, rcc: &RCC, nvic: &mut NVIC) {
    rcc.apb1enr().modify(|_, w| w.tim7en().set_bit());

    unsafe {
        tim7.psc().write(|w| w.psc().bits(83));
    }
    unsafe {
        tim7.arr().write(|w| w.arr().bits(65535));
    }
    tim7.dier().write(|w| w.uie().set_bit());
    tim7.cr1().modify(|_, w| w.cen().set_bit());

    *TIM7_HANDLE.lock() = Some(tim7);

    unsafe {
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::TIM7,
            config::IRQ_NORMAL_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::TIM7);
    }
}

fn init_motor(
    pa1: Pin<'A', 1, Input>,
    pa15: Pin<'A', 15, Alternate<0, PushPull>>,
    pb9: Pin<'B', 9, Input>,
    pb11: Pin<'B', 11, Input>,
    tim2: TIM2,
    tim4: TIM4,
    clocks: &Clocks,
) {
    let pa15 = pa15.into_input();
    let (_, (motor_2_pwm, motor_0_pwm, _, motor_1_pwm, ..)) = tim2.pwm_hz(328125.Hz(), clocks);
    let motor_2_pwm: stm32f4xx_hal::timer::PwmChannel<TIM2, _> = motor_2_pwm.with(pa15);
    let motor_0_pwm = motor_0_pwm.with(pa1);
    let motor_1_pwm = motor_1_pwm.with(pb11);
    let (_, (_, _, _, motor_3_pwm, ..)) = tim4.pwm_hz(328125.Hz(), &clocks);
    let motor_3_pwm = motor_3_pwm.with(pb9);
    let motor = Motor::new(motor_0_pwm, motor_1_pwm, motor_2_pwm, motor_3_pwm);
    *MOTOR.lock() = Some(motor);
}

#[cfg(not(feature = "no_dbg_print"))]
fn init_debug_usart(
    pc10: Pin<'C', 10, Input>,
    pc11: Pin<'C', 11, Input>,
    usart3: USART3,
    dma1_s3: StreamX<DMA1, 3>,
    clocks: &Clocks,
    nvic: &mut NVIC,
) {
    let usart3_pins = (pc10.into_alternate::<7>(), pc11.into_alternate::<7>());
    let usart3 = usart3
        .serial(
            usart3_pins,
            Config::default().baudrate(115_200.bps()).dma(DmaConfig::Tx),
            &clocks,
        )
        .unwrap();

    let dma_handle = usart3.use_dma_tx(dma1_s3);

    *DEBUG_USART_DMA_HANDLE.lock() = Some(dma_handle);

    unsafe {
        // tx interrupt
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::DMA1_STREAM3,
            config::IRQ_NORMAL_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::DMA1_STREAM3);
    }

    let (debug_prod, debug_cons) = sync::create_channel::<DebugString, 32>();
    DEBUG_PRINT_PRODUCER.call_once(move || debug_prod);
    DEBUG_PRINT_CONSUMER.call_once(move || debug_cons);

    debug::set_debug_ready();
}

fn init_usart(
    pc6: Pin<'C', 6, Input>,
    pc7: Pin<'C', 7, Input>,
    usart6: USART6,
    dma2_s7: StreamX<DMA2, 7>,
    clocks: &Clocks,
    nvic: &mut NVIC,
) {
    // USART6
    let usart6_pins = (pc6.into_alternate::<8>(), pc7.into_alternate::<8>());
    let mut usart6 = usart6
        .serial(
            usart6_pins,
            Config::default()
                .baudrate(1_000_000.bps())
                .dma(DmaConfig::Tx),
            &clocks,
        )
        .unwrap();

    // Mask the interrupts before `.listen()` below, otherwise the interrupt
    // may come before we set `NRF_USART` to `Some`.
    NVIC::mask(stm32f4xx_hal::pac::Interrupt::USART6);
    NVIC::mask(stm32f4xx_hal::pac::Interrupt::DMA2_STREAM7);

    usart6.listen(stm32f4xx_hal::serial::Event::RxNotEmpty);

    let handle = usart6.use_dma_tx(dma2_s7);

    *NRF_USART.lock() = Some(handle);

    unsafe {
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::USART6,
            config::IRQ_HIGH_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::USART6);
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::DMA2_STREAM7,
            config::IRQ_HIGH_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::DMA2_STREAM7);
    }
}

fn init_i2c_dma(
    pa8: Pin<'A', 8, Input>,
    pb6: Pin<'B', 6, Input>,
    pb7: Pin<'B', 7, Input>,
    pc9: Pin<'C', 9, Input>,
    i2c1: I2C1,
    i2c3: I2C3,
    dma1_s0: StreamX<DMA1, 0>,
    dma1_s2: StreamX<DMA1, 2>,
    dma1_s4: StreamX<DMA1, 4>,
    dma1_s6: StreamX<DMA1, 6>,
    clocks: &Clocks,
    nvic: &mut NVIC,
) {
    release_peripheral_after_software_reset(&i2c1, &i2c3);

    // i2c3
    let i2c3_pins = (
        pa8.into_alternate::<4>().set_open_drain(),
        pc9.into_alternate::<4>().set_open_drain(),
    );
    let i2c3 = i2c3.i2c(i2c3_pins, 400.kHz(), &clocks);
    // let dma1_streams = StreamsTuple::new(dma1);
    let i2c3_dma: I2c3DMAHandle = i2c3.use_dma(dma1_s4, dma1_s2);
    *I2C3_DMA_HANDLE.lock() = Some(i2c3_dma);
    unsafe {
        // rx interrupt
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::DMA1_STREAM2,
            config::IRQ_NORMAL_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::DMA1_STREAM2);
        // tx interrupt
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::DMA1_STREAM4,
            config::IRQ_NORMAL_PRIORITY,
        );
        pac::NVIC::unmask(stm32f4xx_hal::pac::Interrupt::DMA1_STREAM4);
    }

    // i2c1
    let i2c1_pins = (
        pb6.into_alternate::<4>().set_open_drain(),
        pb7.into_alternate::<4>().set_open_drain(),
    );
    let i2c1 = i2c1.i2c(i2c1_pins, 400.kHz(), &clocks);
    let i2c1_dma: I2c1DMAHandle = i2c1.use_dma(dma1_s6, dma1_s0);
    *I2C1_DMA_HANDLE.lock() = Some(i2c1_dma);
    unsafe {
        // rx interrupt
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::DMA1_STREAM0,
            config::IRQ_NORMAL_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::DMA1_STREAM0);
        // tx interrupt
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::DMA1_STREAM6,
            config::IRQ_NORMAL_PRIORITY,
        );
        pac::NVIC::unmask(stm32f4xx_hal::pac::Interrupt::DMA1_STREAM6);
    }
}

fn init_spi(
    pa5: Pin<'A', 5, Input>,
    pa6: Pin<'A', 6, Input>,
    pa7: Pin<'A', 7, Input>,
    pb4: Pin<'B', 4, Alternate<0, PushPull>>,
    spi1: SPI1,
    clocks: &Clocks,
) {
    let spi1_pins = (
        pa5.into_alternate::<5>().speed(Speed::VeryHigh),
        pa6.into_alternate::<5>().speed(Speed::VeryHigh),
        pa7.into_alternate::<5>().speed(Speed::VeryHigh),
    );
    let mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };
    *SPI1_CS.lock() = Some(pb4.into_push_pull_output());
    let spi1 = spi1.spi(spi1_pins, mode, 1.MHz(), &clocks);
    *SPI1_HANDLE.lock() = Some(spi1);
}

fn init_sensor() {
    let mut accel_gyro = AccelGyro::new();
    let mut tof = ToF::new();
    let mut flow = Flow::new();
    accel_gyro.init();
    tof.init();
    flow.init();
    *SENSOR_ACCEL_GYRO.lock() = Some(accel_gyro);
    *SENSOR_TOF.lock() = Some(tof);
    *SENSOR_FLOW.lock() = Some(flow);
}

fn init_estimator() {
    let estimator = Estimator::new();
    *ESTIMATOR.lock() = Some(estimator);
}

fn init_stabilizer() {
    let stabilizer = Stabilizer::new();
    *STABILIZER.lock() = Some(Arc::new(stabilizer));
}

fn init_radio_link() {
    let radio_rx_link = RadioRxLink::new();
    *RADIO_RX_LINK.lock() = Some(radio_rx_link);

    let radio_tx_link = RadioTxLink::new();
    *RADIO_TX_LINK.lock() = Some(radio_tx_link);

    let (radio_tx_prod, radio_tx_cons) = sync::create_channel::<SystemPacket, 16>();
    RADIO_TX_PRODUCER.call_once(move || radio_tx_prod);
    RADIO_TX_CONSUMER.call_once(move || radio_tx_cons);
}

#[cfg(feature = "exti1_panic")]
fn init_exti1(nvic: &mut NVIC) {
    unsafe {
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::EXTI1,
            config::IRQ_NORMAL_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::EXTI1);
    }
}

#[cfg(feature = "irq_latency_test")]
type GPIOB5 = Pin<'B', 5, Output>;
#[cfg(feature = "irq_latency_test")]
static PB5: hopter::sync::Spin<Option<GPIOB5>> = hopter::sync::Spin::new(None);
#[cfg(feature = "irq_latency_test")]
type GPIOC12 = Pin<'C', 12, Input>;
#[cfg(feature = "irq_latency_test")]
static PC12: hopter::sync::Spin<Option<GPIOC12>> = hopter::sync::Spin::new(None);

#[cfg(feature = "irq_latency_test")]
fn init_interrupt_test(
    mut pc12: Pin<'C', 12, Input>,
    pb5: Pin<'B', 5, Input>,
    exti: &mut EXTI,
    syscfg: &mut SysCfg,
    nvic: &mut NVIC,
) {
    pc12.make_interrupt_source(syscfg);
    pc12.trigger_on_edge(exti, Edge::Rising);
    pc12.enable_interrupt(exti);

    let mut pb5 = pb5.into_push_pull_output();
    pb5.set_low();

    *PB5.lock() = Some(pb5);
    *PC12.lock() = Some(pc12);

    // INTERRUPT_TEST_GPIO.call_once(|| pb5.into_push_pull_output());
    // INTERRUPT_INPUT_GPIO.call_once(|| pc12);

    unsafe {
        nvic.set_priority(
            stm32f4xx_hal::pac::Interrupt::EXTI15_10,
            config::IRQ_MAX_PRIORITY,
        );
        NVIC::unmask(stm32f4xx_hal::pac::Interrupt::EXTI15_10);
    }
}

fn release_peripheral_after_software_reset(i2c1: &I2C1, i2c3: &I2C3) {
    unsafe {
        use stm32f4xx_hal::rcc::{Enable, Reset};
        // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
        let rcc = &(*RCC::ptr());

        // Enable and reset clock.
        I2C1::enable(rcc);
        I2C1::reset(rcc);
        I2C3::enable(rcc);
        I2C3::reset(rcc);
    }

    let dp = unsafe { pac::Peripherals::steal() };

    i2c1.cr1().modify(|_, w| w.pe().clear_bit());

    dp.GPIOA
        .moder()
        .modify(|_, w| w.moder6().output().moder7().output());
    dp.GPIOA
        .otyper()
        .modify(|_, w| w.ot6().push_pull().ot7().push_pull());

    dp.GPIOB.odr().modify(|_, w| w.odr7().set_bit());
    for _ in 0..10 {
        dp.GPIOB.odr().modify(|_, w| w.odr6().set_bit());
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
        dp.GPIOB.odr().modify(|_, w| w.odr6().clear_bit());
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
    dp.GPIOB.odr().modify(|_, w| w.odr6().clear_bit());

    i2c1.cr1().modify(|_, w| w.swrst().set_bit());
    i2c1.cr1().modify(|_, w| w.swrst().clear_bit());

    while i2c1.cr1().read().swrst().bit_is_set() {}

    i2c3.cr1().modify(|_, w| w.pe().clear_bit());

    dp.GPIOA.moder().modify(|_, w| w.moder8().output());
    dp.GPIOA.otyper().modify(|_, w| w.ot8().push_pull());

    dp.GPIOC.moder().modify(|_, w| w.moder9().output());
    dp.GPIOC.otyper().modify(|_, w| w.ot9().push_pull());

    dp.GPIOC.odr().modify(|_, w| w.odr9().set_bit());

    for _ in 0..10 {
        dp.GPIOA.odr().modify(|_, w| w.odr8().set_bit());
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
        dp.GPIOA.odr().modify(|_, w| w.odr8().clear_bit());
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
    dp.GPIOA.odr().modify(|_, w| w.odr8().set_bit());

    i2c3.cr1().modify(|_, w| w.swrst().set_bit());
    i2c3.cr1().modify(|_, w| w.swrst().clear_bit());

    while i2c3.cr1().read().swrst().bit_is_set() {}
}

#[allow(unused_mut)]
pub fn init_peripherals(mut cp: cortex_m::Peripherals, mut dp: pac::Peripherals) {
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    init_usec_timestamp(dp.TIM7, &dp.RCC, &mut cp.NVIC);
    init_leds(gpioc.pc0, gpioc.pc1, gpioc.pc2, gpioc.pc3, gpiod.pd2);
    let clocks = init_clock(dp.RCC.constrain());
    init_motor(
        gpioa.pa1, gpioa.pa15, gpiob.pb9, gpiob.pb11, dp.TIM2, dp.TIM4, &clocks,
    );

    // Has to be placed before USART initialization, otherwise USART6 interrupt
    // may come before we initialize radio link and cause an unwrap panic.
    init_radio_link();

    let dma1_streams = StreamsTuple::new(dp.DMA1);
    let dma2_streams = StreamsTuple::new(dp.DMA2);

    #[cfg(not(feature = "no_dbg_print"))]
    init_debug_usart(
        gpioc.pc10,
        gpioc.pc11,
        dp.USART3,
        dma1_streams.3,
        &clocks,
        &mut cp.NVIC,
    );
    init_usart(
        gpioc.pc6,
        gpioc.pc7,
        dp.USART6,
        dma2_streams.7,
        &clocks,
        &mut cp.NVIC,
    );
    init_i2c_dma(
        gpioa.pa8,
        gpiob.pb6,
        gpiob.pb7,
        gpioc.pc9,
        dp.I2C1,
        dp.I2C3,
        dma1_streams.0,
        dma1_streams.2,
        dma1_streams.4,
        dma1_streams.6,
        &clocks,
        &mut cp.NVIC,
    );
    init_spi(gpioa.pa5, gpioa.pa6, gpioa.pa7, gpiob.pb4, dp.SPI1, &clocks);
    init_sensor();
    init_estimator();
    init_stabilizer();

    #[cfg(feature = "exti1_panic")]
    init_exti1(&mut cp.NVIC);

    #[cfg(feature = "irq_latency_test")]
    init_interrupt_test(
        gpioc.pc12,
        gpiob.pb5,
        &mut dp.EXTI,
        &mut dp.SYSCFG.constrain(),
        &mut cp.NVIC,
    );

    *SYS_CLOCKS.lock() = Some(clocks);
}

pub fn acquire_accel_gyro(
) -> OwningRefMut<'static, MutexGuard<'static, Option<AccelGyro>>, AccelGyro> {
    acquire_singleton(&SENSOR_ACCEL_GYRO)
}

pub fn acquire_tof() -> OwningRefMut<'static, MutexGuard<'static, Option<ToF>>, ToF> {
    acquire_singleton(&SENSOR_TOF)
}

pub fn acquire_flow() -> OwningRefMut<'static, MutexGuard<'static, Option<Flow>>, Flow> {
    acquire_singleton(&SENSOR_FLOW)
}

pub fn acquire_estimator(
) -> OwningRefMut<'static, MutexGuard<'static, Option<Estimator>>, Estimator> {
    acquire_singleton(&ESTIMATOR)
}

pub fn acquire_stabilizer() -> Arc<Stabilizer> {
    STABILIZER
        .lock()
        .as_mut()
        .map(|stabilizer| Arc::clone(stabilizer))
        .unwrap()
}

pub fn acquire_motor() -> OwningRefMut<'static, MutexGuard<'static, Option<Motor>>, Motor> {
    acquire_singleton(&MOTOR)
}

pub fn acquire_spi1_cs() -> OwningRefMut<'static, MutexGuard<'static, Option<Spi1Cs>>, Spi1Cs> {
    acquire_singleton(&SPI1_CS)
}

pub fn acquire_spi1_handle(
) -> OwningRefMut<'static, MutexGuard<'static, Option<Spi1Handle>>, Spi1Handle> {
    acquire_singleton(&SPI1_HANDLE)
}

pub fn acquire_irqsafe_tim7_handle(
) -> OwningRefMut<'static, MutexIrqSafeGuard<'static, Option<TIM7>, impl MaskableIrq>, TIM7> {
    acquire_irqsafe_singleton(&TIM7_HANDLE)
}

pub fn acquire_irqsafe_i2c1_dma_handle() -> OwningRefMut<
    'static,
    MutexIrqSafeGuard<'static, Option<I2c1DMAHandle>, impl MaskableIrq>,
    I2c1DMAHandle,
> {
    acquire_irqsafe_singleton(&I2C1_DMA_HANDLE)
}

pub fn acquire_irqsafe_i2c3_dma_handle() -> OwningRefMut<
    'static,
    MutexIrqSafeGuard<'static, Option<I2c3DMAHandle>, impl MaskableIrq>,
    I2c3DMAHandle,
> {
    acquire_irqsafe_singleton(&I2C3_DMA_HANDLE)
}

pub fn acquire_irqsafe_radio_rx_link() -> OwningRefMut<
    'static,
    MutexIrqSafeGuard<'static, Option<RadioRxLink>, impl MaskableIrq>,
    RadioRxLink,
> {
    acquire_irqsafe_singleton(&RADIO_RX_LINK)
}

pub fn acquire_radio_tx_link(
) -> OwningRefMut<'static, MutexGuard<'static, Option<RadioTxLink>>, RadioTxLink> {
    acquire_singleton(&RADIO_TX_LINK)
}

pub fn acquire_irqsafe_nrf_usart() -> OwningRefMut<
    'static,
    MutexIrqSafeGuard<'static, Option<NRFSerialDMAHandle>, impl MaskableIrq>,
    NRFSerialDMAHandle,
> {
    acquire_irqsafe_singleton(&NRF_USART)
}

fn acquire_singleton<T>(
    singleton: &'static Mutex<Option<T>>,
) -> OwningRefMut<MutexGuard<'static, Option<T>>, T> {
    let guard = singleton.lock();
    let or = OwningRefMut::new(guard);
    or.map_mut(|guard| guard.as_mut().unwrap())
}

pub fn force_acquire_motor() -> OwningRefMut<'static, MutexGuard<'static, Option<Motor>>, Motor> {
    let guard = loop {
        if let Some(guard) = MOTOR.try_lock() {
            break guard;
        }
        unsafe {
            MOTOR.force_unlock();
        }
    };

    let or = OwningRefMut::new(guard);
    or.map_mut(|guard| guard.as_mut().unwrap())
}

fn acquire_irqsafe_singleton<T, I: MaskableIrq>(
    singleton: &'static MutexIrqSafe<Option<T>, I>,
) -> OwningRefMut<MutexIrqSafeGuard<'static, Option<T>, I>, T> {
    let guard = singleton.lock();
    let or = OwningRefMut::new(guard);
    or.map_mut(|guard| guard.as_mut().unwrap())
}

pub fn borrow_i2c1_sem() -> &'static Semaphore {
    &SEM_I2C1
}

pub fn borrow_i2c3_sem() -> &'static Semaphore {
    &SEM_I2C3
}

pub fn borrow_stabilizer_sem() -> &'static Semaphore {
    &SEM_STABILIZER
}

pub fn borrow_system_start_wait_sem() -> &'static Semaphore {
    &SEM_SYSTEM_START_WAIT
}

pub fn borrow_nrf_usart_sem() -> &'static Semaphore {
    &SEM_NRF_USART
}

pub fn system_wait_start() {
    let sem = borrow_system_start_wait_sem();
    sem.down();
}

pub fn clone_radio_tx_producer() -> Producer<SystemPacket, 16> {
    RADIO_TX_PRODUCER.wait().clone()
}

pub fn clone_radio_tx_consumer() -> Consumer<SystemPacket, 16> {
    RADIO_TX_CONSUMER.wait().clone()
}

#[allow(unused)]
pub fn borrow_irqsafe_leds() -> &'static LEDs {
    LEDS.wait()
}

cfg_if! { if #[cfg(not(feature = "no_dbg_print"))] {

pub fn acquire_irqsafe_debug_usart_dma_handle() -> OwningRefMut<
    'static,
    MutexIrqSafeGuard<'static, Option<DebugSerialDMAHandle>, impl MaskableIrq>,
    DebugSerialDMAHandle,
> {
    acquire_irqsafe_singleton(&DEBUG_USART_DMA_HANDLE)
}

pub fn clone_debug_print_producer() -> Producer<DebugString, 32> {
    DEBUG_PRINT_PRODUCER.wait().clone()
}

#[allow(unused)]
pub fn clone_debug_print_consumer() -> Consumer<DebugString, 32> {
    DEBUG_PRINT_CONSUMER.wait().clone()
}

pub fn borrow_debug_usart_sem() -> &'static Semaphore {
    &SEM_DEBUG_USART
}

}} // end if #[cfg(not(feature = "no_dbg_print"))]
