use super::timestamp;
use crate::driver::pmw3901::{PMW3901Dev, PMW3901MotionData};

pub struct Flow {
    rate: u32,
    dev: PMW3901Dev,
    last_read: f32,
    is_valid: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct FlowData {
    pub delta_x: f32,
    pub delta_y: f32,
    pub std_dev_x: f32,
    pub std_dev_y: f32,
    pub dt: f32,
}

impl Flow {
    pub fn new() -> Self {
        Self {
            rate: 50,
            dev: PMW3901Dev {},
            last_read: 0.0,
            is_valid: false,
        }
    }

    pub fn init(&mut self) -> i8 {
        let rslt = self.dev.init();
        self.is_valid = rslt == 0;
        rslt
    }

    pub fn get_interval_ms(&self) -> u32 {
        1000 / self.rate
    }

    pub fn task_loop(&mut self) -> Option<FlowData> {
        if !self.is_valid {
            return None;
        }
        let mut motion_data = PMW3901MotionData::default();
        self.dev.pmw3901_get_motion_data(&mut motion_data);

        const DELTA_LIMIT: i16 = 150;
        if motion_data.delta_x.abs() < DELTA_LIMIT
            && motion_data.delta_y.abs() < DELTA_LIMIT
            && motion_data.motion == 0xB0
        {
            let now = timestamp::usec_timestamp() as f32 / 1000000.0;
            let dt = now - self.last_read;
            self.last_read = now;
            Some(FlowData {
                delta_x: -motion_data.delta_y as f32,
                delta_y: -motion_data.delta_x as f32,
                std_dev_x: 1.0,
                std_dev_y: 1.0,
                dt,
            })
        } else {
            None
        }
    }
}
