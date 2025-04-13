use crate::module::{
    self,
    controller::types::{SensorT, StateT},
    flow::FlowData,
    timestamp,
    tof::ToFData,
};
use core::sync::atomic::{AtomicBool, Ordering};
use hopter::{
    sync::{Consumer, Producer},
    task,
    time::IntervalBarrier,
};

pub fn build_estimator_kalman_task(
    cons_accel_gyro: Consumer<SensorT, 2>,
    cons_tof: Consumer<ToFData, 2>,
    cons_flow: Consumer<FlowData, 2>,
    prod_estimator: Producer<StateT, 2>,
) {
    task::build()
        .set_id(super::TASK_ID_ESTIMATOR)
        .set_priority(super::TASK_PRIO_ESTIMATOR)
        .set_entry(|| estimator_kalman_task(cons_accel_gyro, cons_tof, cons_flow, prod_estimator))
        .set_stack_init_size(super::TASK_STK_SIZE_ESTIMATOR)
        .spawn_restartable()
        .unwrap();
}

fn estimator_kalman_task(
    cons_accel_gyro: Consumer<SensorT, 2>,
    cons_tof: Consumer<ToFData, 2>,
    cons_flow: Consumer<FlowData, 2>,
    prod_estimator: Producer<StateT, 2>,
) {
    dbg_println!("Estimator Kalman [START]");

    let mut barrier = IntervalBarrier::new(estimator_kalman_init_and_get_interval()).unwrap();
    let mut count = 0;

    loop {
        barrier.wait();

        super::estimator_report_alive();

        #[cfg(feature = "estimator_panic")]
        {
            let prev_cnt = ESTIMATOR_PANIC_COUNTER.fetch_add(1, Ordering::Relaxed);
            if (prev_cnt + 1) % 1000 == 0 {
                ESTIMATOR_PANIC_REQUIRED.store(true, Ordering::Relaxed);
            }
        }

        let mut estimator = module::acquire_estimator();

        // receive sensor data from other tasks
        while let Some(data) = cons_accel_gyro.try_consume_allow_isr() {
            estimator.update_accumulated_data(data);
        }

        while let Some(data) = cons_tof.try_consume_allow_isr() {
            estimator.update_tof(data);
        }

        while let Some(data) = cons_flow.try_consume_allow_isr() {
            estimator.update_flow(data);
        }

        let mut drone_state = estimator.task_loop(count);
        drone_state.timestamp = timestamp::usec_timestamp();
        // send state to stabilizer tasks
        let _ = prod_estimator.try_produce_allow_isr(drone_state);

        count += 1;
    }
}

fn estimator_kalman_init_and_get_interval() -> u32 {
    static HAS_STARTED: AtomicBool = AtomicBool::new(false);
    if !HAS_STARTED.load(Ordering::SeqCst) {
        module::system_wait_start();
        HAS_STARTED.store(true, Ordering::SeqCst)
    }

    let estimator = module::acquire_estimator();
    estimator.get_interval_ms()
}

#[cfg(feature = "estimator_panic")]
use core::sync::atomic::AtomicUsize;

#[cfg(feature = "estimator_panic")]
pub static ESTIMATOR_PANIC_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "estimator_panic")]
pub static ESTIMATOR_PANIC_REQUIRED: AtomicBool = AtomicBool::new(false);
