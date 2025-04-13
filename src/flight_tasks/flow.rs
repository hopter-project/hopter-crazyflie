use crate::module::{self, flow::FlowData};
use hopter::{sync::Producer, task, time::IntervalBarrier};

pub fn build_flow_task(prod_estimator: Producer<FlowData, 2>) {
    task::build()
        .set_id(super::TASK_ID_FLOW)
        .set_priority(super::TASK_PRIO_FLOW)
        .set_entry(|| flow_task(prod_estimator))
        .set_stack_init_size(super::TASK_STK_SIZE_FLOW)
        .spawn_restartable()
        .unwrap();
}

fn flow_task(prod_estimator: Producer<FlowData, 2>) {
    let mut barrier = IntervalBarrier::new(flow_task_get_interval()).unwrap();

    loop {
        barrier.wait();

        super::flow_report_alive();

        #[cfg(feature = "flow_panic")]
        {
            FLOW_PANIC_REQUIRED.store(false, Ordering::Relaxed);
            let prev_cnt = FLOW_PANIC_COUNTER.fetch_add(1, Ordering::Relaxed);
            if (prev_cnt + 1) % 50 == 0 {
                FLOW_PANIC_REQUIRED.store(true, Ordering::Relaxed);
            }
        }

        let mut flow = module::acquire_flow();
        if let Some(flow_data) = flow.task_loop() {
            let _ = prod_estimator.try_produce_allow_isr(flow_data);
        }
    }
}

fn flow_task_get_interval() -> u32 {
    let flow = module::acquire_flow();
    flow.get_interval_ms()
}

#[cfg(feature = "flow_panic")]
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

#[cfg(feature = "flow_panic")]
pub static FLOW_PANIC_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "flow_panic")]
pub static FLOW_PANIC_REQUIRED: AtomicBool = AtomicBool::new(false);
