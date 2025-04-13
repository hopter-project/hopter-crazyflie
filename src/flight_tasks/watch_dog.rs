use crate::module;
use core::sync::atomic::{AtomicBool, Ordering};
use hopter::{task, time};

static TOF_ALIVE: AtomicBool = AtomicBool::new(false);
static FLOW_ALIVE: AtomicBool = AtomicBool::new(false);
static ACCEL_GYRO_ALIVE: AtomicBool = AtomicBool::new(false);
static CRTP_RX_ALIVE: AtomicBool = AtomicBool::new(false);
static STABILIZER_ALIVE: AtomicBool = AtomicBool::new(false);
static ESTIMATOR_ALIVE: AtomicBool = AtomicBool::new(false);
static WATCH_DOG_ENABLE: AtomicBool = AtomicBool::new(false);

pub(super) fn tof_report_alive() {
    TOF_ALIVE.store(true, Ordering::SeqCst);
}

pub(super) fn flow_report_alive() {
    FLOW_ALIVE.store(true, Ordering::SeqCst);
}

pub(super) fn accel_gyro_report_alive() {
    ACCEL_GYRO_ALIVE.store(true, Ordering::SeqCst);
}

pub(super) fn crtp_rx_report_alive() {
    CRTP_RX_ALIVE.store(true, Ordering::SeqCst);
}

pub(super) fn stabilizer_report_alive() {
    STABILIZER_ALIVE.store(true, Ordering::SeqCst);
}

pub(super) fn estimator_report_alive() {
    ESTIMATOR_ALIVE.store(true, Ordering::SeqCst);
}

pub(super) fn enable_watch_dog() {
    WATCH_DOG_ENABLE.store(true, Ordering::SeqCst);
}

pub fn build_watch_dog_task() {
    task::build()
        .set_id(super::TASK_ID_WATCH_DOG)
        .set_entry(watch_dog)
        .set_stack_init_size(super::TASK_STK_SIZE_WATCH_DOG)
        .set_priority(super::TASK_PRIO_WATCH_DOG)
        .spawn_restartable()
        .unwrap();
}

fn watch_dog() {
    loop {
        time::sleep_ms(1000).unwrap();
        watch_dog_task_inner();
    }
}

#[inline(never)]
fn watch_dog_task_inner() {
    if WATCH_DOG_ENABLE.load(Ordering::SeqCst) {
        if !TOF_ALIVE.load(Ordering::SeqCst) {
            dbg_println_blocking!("tof dead");
            emergency_motor_halt();
        }
        if !FLOW_ALIVE.load(Ordering::SeqCst) {
            dbg_println_blocking!("flow dead");
            emergency_motor_halt();
        }
        if !CRTP_RX_ALIVE.load(Ordering::SeqCst) {
            dbg_println_blocking!("crtp rx dead");
            emergency_motor_halt();
        }
        if !ACCEL_GYRO_ALIVE.load(Ordering::SeqCst) {
            dbg_println_blocking!("accel_gyro dead");
            emergency_motor_halt();
        }
        if !STABILIZER_ALIVE.load(Ordering::SeqCst) {
            dbg_println_blocking!("stabilizer dead");
            emergency_motor_halt();
        }
        if !ESTIMATOR_ALIVE.load(Ordering::SeqCst) {
            dbg_println_blocking!("estimator dead");
            emergency_motor_halt();
        }
        TOF_ALIVE.store(false, Ordering::SeqCst);
        FLOW_ALIVE.store(false, Ordering::SeqCst);
        CRTP_RX_ALIVE.store(false, Ordering::SeqCst);
        ACCEL_GYRO_ALIVE.store(false, Ordering::SeqCst);
        STABILIZER_ALIVE.store(false, Ordering::SeqCst);
        ESTIMATOR_ALIVE.store(false, Ordering::SeqCst);
    }

    fn emergency_motor_halt() {
        let mut motor = module::force_acquire_motor();
        motor.stop();
        loop {}
    }
}
