use crate::module::{self, tof::ToFData};
use hopter::{sync::Producer, task, time::IntervalBarrier};

pub fn build_tof_task(prod_estimator: Producer<ToFData, 2>) {
    task::build()
        .set_id(super::TASK_ID_TOF)
        .set_priority(super::TASK_PRIO_TOF)
        .set_entry(|| tof_task(prod_estimator))
        .set_stack_init_size(super::TASK_STK_SIZE_TOF)
        .spawn_restartable()
        .unwrap();
}

fn tof_task(prod_estimator: Producer<ToFData, 2>) {
    let mut barrier = IntervalBarrier::new(tof_task_get_interval()).unwrap();

    loop {
        barrier.wait();

        super::tof_report_alive();

        #[cfg(feature = "tof_panic")]
        {
            TOF_PANIC_REQUIRED.store(false, Ordering::Relaxed);
            let prev_cnt = TOF_PANIC_COUNTER.fetch_add(1, Ordering::Relaxed);
            if (prev_cnt + 1) % 25 == 0 {
                TOF_PANIC_REQUIRED.store(true, Ordering::Relaxed);
            }
        }

        let mut tof = module::acquire_tof();
        if let Some(tof_data) = tof.task_loop() {
            let _ = prod_estimator.try_produce_allow_isr(tof_data);
        }
    }
}

fn tof_task_get_interval() -> u32 {
    let tof = module::acquire_tof();
    tof.get_interval_ms()
}

#[cfg(feature = "tof_panic")]
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

#[cfg(feature = "tof_panic")]
pub static TOF_PANIC_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "tof_panic")]
pub static TOF_PANIC_REQUIRED: AtomicBool = AtomicBool::new(false);
