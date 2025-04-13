use crate::module::controller::types::SensorT;
use core::sync::atomic::{AtomicU32, Ordering};

pub struct Supervisor {
    tumble_count: AtomicU32,
}

impl Supervisor {
    pub fn new() -> Self {
        Self {
            tumble_count: AtomicU32::new(0),
        }
    }

    pub fn check_tumble(&self, sensor: &SensorT) -> bool {
        if sensor.accel.z < -0.5 {
            if self.tumble_count.load(Ordering::Relaxed) > 30 {
                return true;
            } else {
                self.tumble_count.fetch_add(1, Ordering::Relaxed);
            }
        } else {
            self.tumble_count.store(0, Ordering::Relaxed);
        }
        false
    }
}
