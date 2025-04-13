use crate::module::{self, timestamp};
use alloc::sync::Arc;
use hopter::{
    debug::{
        cpu_load::{LoadInspector, MicrosecPrecision},
        segmented_stack,
    },
    task,
    time::IntervalBarrier,
};

pub fn build_led_task() {
    task::build()
        .set_id(super::TASK_ID_LED)
        .set_priority(super::TASK_PRIO_LED)
        .set_entry(led_task)
        .set_stack_init_size(super::TASK_STK_SIZE_LED)
        .spawn_restartable()
        .unwrap();
}

fn led_task() {
    let mut barrier = IntervalBarrier::new(500).unwrap();
    loop {
        barrier.wait();
        let leds = module::borrow_irqsafe_leds();
        leds.left_green_toggle();
        leds.right_green_toggle();
    }
}
