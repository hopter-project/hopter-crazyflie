use crate::module::{
    self,
    controller::types::{SensorT, SetPointT, StateT},
    timestamp,
};
use core::sync::atomic::{AtomicBool, Ordering};
use hopter::{sync::Consumer, task};

pub fn build_stabilizer_task(
    cons_accel_gyro: Consumer<SensorT, 2>,
    cons_estimator: Consumer<StateT, 2>,
    cons_setpoint: Consumer<SetPointT, 2>,
) {
    task::build()
        .set_id(super::TASK_ID_STABILIZER)
        .set_priority(super::TASK_PRIO_STABILIZER)
        .set_entry(|| stabilizer_task(cons_accel_gyro, cons_estimator, cons_setpoint))
        .set_stack_init_size(super::TASK_STK_SIZE_STABILIZER)
        .set_stack_limit(4096)
        .spawn_restartable()
        .unwrap();
}

fn stabilizer_task(
    cons_accel_gyro: Consumer<SensorT, 2>,
    cons_estimator: Consumer<StateT, 2>,
    cons_setpoint: Consumer<SetPointT, 2>,
) {
    #[cfg(feature = "stabilizer_panic")]
    {
        use crate::flight_tasks::STABILIZER_PANIC_END_TS;
        use crate::flight_tasks::STABILIZER_PANIC_RESPOND;
        use core::sync::atomic::Ordering;
        if STABILIZER_PANIC_RESPOND.load(Ordering::Relaxed) {
            STABILIZER_PANIC_RESPOND.store(false, Ordering::Relaxed);
            STABILIZER_PANIC_END_TS.store(
                crate::module::timestamp::usec_timestamp() as usize,
                Ordering::Relaxed,
            );
        }
    }

    #[cfg(feature = "stabilizer_overflow")]
    {
        use crate::flight_tasks::STABILIZER_PANIC_END_TS;
        use crate::flight_tasks::STABILIZER_PANIC_RESPOND;
        use core::sync::atomic::Ordering;
        if STABILIZER_PANIC_RESPOND.load(Ordering::Relaxed) {
            STABILIZER_PANIC_RESPOND.store(false, Ordering::Relaxed);
            STABILIZER_PANIC_END_TS.store(
                crate::module::timestamp::usec_timestamp() as usize,
                Ordering::Relaxed,
            );
        }
    }

    static HAS_STARTED: AtomicBool = AtomicBool::new(false);
    if !HAS_STARTED.load(Ordering::SeqCst) {
        module::system_wait_start();
        HAS_STARTED.store(true, Ordering::SeqCst)
    }

    dbg_println!("Stabilizer [START]");

    let mut latest_sensor_data;
    let mut latest_state_data = StateT::default();
    let mut latest_setpoint_data = SetPointT::default();
    let mut last_setpoint_update = 0;
    let mut count = 0;

    loop {
        module::borrow_stabilizer_sem().down();

        super::stabilizer_report_alive();

        #[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
        {
            let prev_cnt = STABILIZER_PANIC_COUNTER.fetch_add(1, Ordering::Relaxed);
            if (prev_cnt + 1) == 8325 {
                STABILIZER_PANIC_REQUIRED.store(true, Ordering::Relaxed);
            }
        }

        let stabilizer = module::acquire_stabilizer();

        // this should not fail
        latest_sensor_data = cons_accel_gyro.try_consume_allow_isr().unwrap();

        // this may fail, but it's ok with the previous state
        while let Some(state) = cons_estimator.try_consume_allow_isr() {
            latest_state_data = state;
        }

        let now = timestamp::usec_timestamp();

        if let Some(setpoint) = cons_setpoint.try_consume_allow_isr() {
            latest_setpoint_data = setpoint;
            last_setpoint_update = now;
        } else if now - last_setpoint_update > 300000 {
            // if no setpoint update for 300ms, stop
            latest_setpoint_data = SetPointT::default();
            last_setpoint_update = now;
        }

        stabilizer.task_loop(
            latest_sensor_data,
            latest_state_data,
            latest_setpoint_data,
            count,
        );

        count += 1;
    }
}

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
use core::sync::atomic::AtomicUsize;

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
pub static STABILIZER_PANIC_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
pub static STABILIZER_PANIC_REQUIRED: AtomicBool = AtomicBool::new(false);

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
pub static STABILIZER_PANIC_RESPOND: AtomicBool = AtomicBool::new(false);

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
#[no_mangle]
pub static STABILIZER_PANIC_BEGIN_TS: AtomicUsize = AtomicUsize::new(0);

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
#[no_mangle]
pub static STABILIZER_PANIC_END_TS: AtomicUsize = AtomicUsize::new(0);

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
#[no_mangle]
pub static STABILIZER_PANIC_MID_TS: AtomicUsize = AtomicUsize::new(0);

#[cfg(any(feature = "stabilizer_panic", feature = "stabilizer_overflow"))]
#[no_mangle]
pub extern "C" fn record_mid() {
    STABILIZER_PANIC_MID_TS.store(timestamp::usec_timestamp() as usize, Ordering::Relaxed);
}
