use super::{
    controller::{
        types::{SensorT, SetPointT, StateT},
        Controller,
    },
    estimator::supervisor::Supervisor,
};
use hopter::sync::Mutex;
pub struct Stabilizer {
    pub controller: Controller,
    pub supervisor: Supervisor,
}

static EMERGENCY_STOP: Mutex<bool> = Mutex::new(false);

impl Stabilizer {
    pub fn new() -> Self {
        Self {
            controller: Controller::new(),
            supervisor: Supervisor::new(),
        }
    }

    pub fn task_loop(
        &self,
        sensor_data: SensorT,
        state_data: StateT,
        setpoint_data: SetPointT,
        count: u64,
    ) {
        let control = self
            .controller
            .update(&setpoint_data, &sensor_data, &state_data, count);

        if self.supervisor.check_tumble(&sensor_data) || *EMERGENCY_STOP.lock() {
            super::acquire_motor().stop();
            *EMERGENCY_STOP.lock() = false;
        } else {
            super::acquire_motor().power_distribution(control);
        }
    }
}
