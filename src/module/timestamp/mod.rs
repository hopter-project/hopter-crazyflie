use core::sync::atomic::{AtomicU32, Ordering};
use hopter::interrupt::declare::{handler, irq};

/// The higher half of the counter of the high precision lock-free system timestamp.
static TIMESTAMP_HIGH: AtomicU32 = AtomicU32::new(0);

/// Increase the higher half of the counter.
pub fn incr_timestamp_high() {
    TIMESTAMP_HIGH.fetch_add(1, Ordering::SeqCst);
}

/// Get the microsecond precison system timestamp.
pub fn usec_timestamp() -> u64 {
    // Get the higher half of the counter.
    let prev_high = TIMESTAMP_HIGH.load(Ordering::SeqCst);

    // Get the lower half of the counter.
    let cnt = {
        let tim7 = super::acquire_irqsafe_tim7_handle();
        tim7.cnt.read().cnt().bits()
    };

    // Get the higher half of the counter again.
    let cur_high = TIMESTAMP_HIGH.load(Ordering::SeqCst);

    // If the higher half was not incremented during our read of the lower half,
    // combine the higher and lower half and return the result.
    if prev_high == cur_high {
        (prev_high as u64) << 16 | (cnt as u64)

    // Otherwise, the higher half was just incremented. The lower half we just
    // read possibly just wrapped round to a very small value. We should read
    // the lower half again and combine it with the new higher half. We do not
    // expect to see the higher half get incremented again very soon.
    } else {
        let cnt = super::acquire_irqsafe_tim7_handle().cnt.read().cnt().bits();
        (cur_high as u64) << 16 | (cnt as u64)
    }
}

irq!(Tim7Irq, stm32f4xx_hal::pac::Interrupt::TIM7);

/// Handle TIM7 interrupt.
#[handler(TIM7)]
fn tim7_handler() {
    // Increase the high half of the system timestamp.
    incr_timestamp_high();

    // Acknowledge the interrupt.
    let tim7 = crate::module::acquire_irqsafe_tim7_handle();
    tim7.sr.modify(|_, w| w.uif().clear_bit());
}
