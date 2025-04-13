use crate::module::{self, controller::types::SensorT};
use hopter::{sync::Producer, task, time::IntervalBarrier};

pub fn build_sensor_accel_gyro_task(
    prod_stabilizer: Producer<SensorT, 2>,
    prod_estimator: Producer<SensorT, 2>,
) {
    task::build()
        .set_id(super::TASK_ID_ACCEL_GYRO)
        .set_priority(super::TASK_PRIO_ACCEL_GYRO)
        .set_entry(move || sensor_accel_gyro(prod_stabilizer, prod_estimator))
        .set_stack_init_size(super::TASK_STK_SIZE_ACCEL_GYRO)
        .spawn_restartable()
        .unwrap();
}

fn sensor_accel_gyro(prod_stabilizer: Producer<SensorT, 2>, prod_estimator: Producer<SensorT, 2>) {
    let mut barrier = IntervalBarrier::new(sensor_accel_gyro_get_interval()).unwrap();
    loop {
        barrier.wait();

        super::accel_gyro_report_alive();

        #[cfg(feature = "accel_gyro_panic")]
        {
            ACCEL_GYRO_PANIC_REQUIRED.store(false, Ordering::Relaxed);
            let prev_cnt = ACCEL_GYRO_PANIC_COUNTER.fetch_add(1, Ordering::Relaxed);
            if (prev_cnt + 1) % 1000 == 0 {
                ACCEL_GYRO_PANIC_REQUIRED.store(true, Ordering::Relaxed);
            }
        }

        let mut accel_gyro = module::acquire_accel_gyro();
        let sensor_data = accel_gyro.task_loop();
        // send sensor data to other tasks
        let _ = prod_stabilizer.try_produce_allow_isr(sensor_data);
        let _ = prod_estimator.try_produce_allow_isr(sensor_data);
        {
            let sem = module::borrow_stabilizer_sem();
            let _ = sem.try_up_allow_isr();
        }
    }
}

fn sensor_accel_gyro_get_interval() -> u32 {
    let accel_gyro = module::acquire_accel_gyro();
    accel_gyro.get_interval_ms()
}

#[cfg(feature = "accel_gyro_panic")]
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

#[cfg(feature = "accel_gyro_panic")]
pub static ACCEL_GYRO_PANIC_COUNTER: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "accel_gyro_panic")]
pub static ACCEL_GYRO_PANIC_REQUIRED: AtomicBool = AtomicBool::new(false);
