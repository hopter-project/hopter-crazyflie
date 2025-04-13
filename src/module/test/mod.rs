#[cfg(feature = "irq_latency_test")]
use hopter::interrupt::declare::handler;

#[cfg(feature = "irq_latency_test")]
#[handler(EXTI15_10)]
fn exti15_10_handler() {
    use stm32f4xx_hal::gpio::{ExtiPin, Input, Output, Pin};

    let mut pb5 = super::PB5.lock();
    let pb5 = pb5.as_mut().unwrap();

    pb5.set_high();

    for _ in 0..50 {
        cortex_m::asm::nop();
    }
    pb5.set_low();

    for _ in 0..100 {
        cortex_m::asm::nop();
    }

    let mut pc12 = super::PC12.lock();
    let pc12 = pc12.as_mut().unwrap();
    pc12.clear_interrupt_pending_bit();
}
