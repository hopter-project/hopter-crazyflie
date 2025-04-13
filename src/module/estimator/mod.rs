use self::kalman::{KalmanCoreData, GRAVITY};

use super::{
    controller::types::{AccelerationT, AttitudeRateT, SensorT, StateT},
    flow::FlowData,
    timestamp,
    tof::ToFData,
};

pub mod kalman;
pub mod kalman_update;
pub mod supervisor;

pub struct Estimator {
    rate: u32,
    kalman: KalmanCoreData,
    latest_accel: AccelerationT,
    latest_gyro: AttitudeRateT,
    last_prediction: f32,
    last_process_noise: f32,
    accumulated_accel: AccelerationT,
    accumulated_accel_count: u32,
    accumulated_gyro: AttitudeRateT,
    accumulated_gyro_count: u32,
    update: bool,
}

impl Estimator {
    pub fn new() -> Self {
        Self {
            rate: 1000,
            kalman: KalmanCoreData::new(),
            latest_accel: AccelerationT::default(),
            latest_gyro: AttitudeRateT::default(),
            last_prediction: 0.0,
            last_process_noise: 0.0,
            accumulated_accel: AccelerationT::default(),
            accumulated_accel_count: 0,
            accumulated_gyro: AttitudeRateT::default(),
            accumulated_gyro_count: 0,
            update: false,
        }
    }

    pub fn reset(&mut self) {
        self.kalman = KalmanCoreData::new();
        self.clear_accumulated_data();
    }

    pub fn update_accumulated_data(&mut self, sensor: SensorT) {
        self.latest_accel = sensor.accel;
        self.latest_gyro = sensor.gyro;
        self.accumulated_accel.x += sensor.accel.x;
        self.accumulated_accel.y += sensor.accel.y;
        self.accumulated_accel.z += sensor.accel.z;
        self.accumulated_accel_count += 1;

        self.accumulated_gyro.roll += sensor.gyro.roll;
        self.accumulated_gyro.pitch += sensor.gyro.pitch;
        self.accumulated_gyro.yaw += sensor.gyro.yaw;
        self.accumulated_gyro_count += 1;
    }

    pub fn update_tof(&mut self, tof: ToFData) {
        self.kalman.update_with_tof(&tof);
        self.update = true;
    }

    pub fn update_flow(&mut self, flow: FlowData) {
        self.kalman.update_with_flow(&flow, &self.latest_gyro);
        self.update = true;
    }

    fn clear_accumulated_data(&mut self) {
        self.accumulated_accel = AccelerationT::default();
        self.accumulated_accel_count = 0;
        self.accumulated_gyro = AttitudeRateT::default();
        self.accumulated_gyro_count = 0;
    }

    pub fn task_loop(&mut self, count: u64) -> StateT {
        // update kalman filter prediction at 100hz

        if count % 10 == 0 {
            if self.accumulated_accel_count > 0 && self.accumulated_gyro_count > 0 {
                self.accumulated_gyro.roll /= self.accumulated_gyro_count as f32;
                self.accumulated_gyro.pitch /= self.accumulated_gyro_count as f32;
                self.accumulated_gyro.yaw /= self.accumulated_gyro_count as f32;

                self.accumulated_accel.x /= self.accumulated_accel_count as f32;
                self.accumulated_accel.y /= self.accumulated_accel_count as f32;
                self.accumulated_accel.z /= self.accumulated_accel_count as f32;

                let is_flying: bool;
                {
                    is_flying = super::acquire_motor().is_flying;
                }

                let accel_average = AccelerationT {
                    x: self.accumulated_accel.x * GRAVITY,
                    y: self.accumulated_accel.y * GRAVITY,
                    z: self.accumulated_accel.z * GRAVITY,
                };

                let gyro_average = AttitudeRateT {
                    roll: self.accumulated_gyro.roll.to_radians(),
                    pitch: self.accumulated_gyro.pitch.to_radians(),
                    yaw: self.accumulated_gyro.yaw.to_radians(),
                };

                let now = timestamp::usec_timestamp() as f32 / 1_000_000_f32;

                self.kalman.predict(
                    accel_average,
                    gyro_average,
                    now - self.last_prediction,
                    is_flying,
                );
                self.clear_accumulated_data();
                self.last_prediction = now;
                self.update = true;
            }
        }

        // add process noise every loop
        let now = timestamp::usec_timestamp() as f32 / 1_000_000_f32;
        self.kalman.add_process_noise(now - self.last_process_noise);
        self.last_process_noise = now;

        if self.update {
            // if an update has been made, the state is finalized
            self.kalman.finalize();
            if !self.kalman.check_bound() {
                self.reset();
                dbg_println!("state out of bound, reset kalman");
            }
            self.update = false;
        }

        self.kalman.externalize_state(self.latest_accel)
    }

    pub fn get_interval_ms(&self) -> u32 {
        1000 / self.rate
    }
}
