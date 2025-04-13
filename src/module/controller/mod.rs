pub mod pid;
pub mod types;

use core::sync::atomic::Ordering;

use self::types::{AttitudeAccelT, AttitudeRateT};

use super::motor::{MOTOR_THRUST_BASE, MOTOR_THRUST_MAX, MOTOR_THRUST_MIN, MOTOR_THRUST_SCALE};
use atomic_float::AtomicF32;
use libm;
use pid::PID;
use types::{
    AttitudeRateTAtomic, AttitudeT, AttitudeTAtomic, ControlT, ControlTAtomic, SensorT,
    SetPointMode, SetPointT, StateT,
};

const ATTITUDE_UPDATE_RATE: f32 = 500.0;
const POSITION_UPDATE_RATE: f32 = 100.0;

#[allow(unused)]
pub enum ChannelA {
    Roll,
    Pitch,
    Yaw,
}

/* Attitude controller */
#[derive(Debug)]
pub struct ControllerA {
    roll_val: PID,
    roll_rate: PID,
    pitch_val: PID,
    pitch_rate: PID,
    yaw_val: PID,
    yaw_rate: PID,
}

impl ControllerA {
    pub fn new() -> Self {
        Self {
            roll_val: PID::new(6.0, 3.0, 0.0, ATTITUDE_UPDATE_RATE, Some(20.0), None),
            roll_rate: PID::new(250.0, 500.0, 2.5, ATTITUDE_UPDATE_RATE, Some(33.0), None),
            pitch_val: PID::new(6.0, 3.0, 0.0, ATTITUDE_UPDATE_RATE, Some(20.0), None),
            pitch_rate: PID::new(250.0, 500.0, 2.5, ATTITUDE_UPDATE_RATE, Some(33.0), None),
            yaw_val: PID::new(6.0, 1.0, 0.35, ATTITUDE_UPDATE_RATE, Some(360.0), None),
            yaw_rate: PID::new(120.0, 16.7, 0.0, ATTITUDE_UPDATE_RATE, Some(33.0), None),
        }
    }

    fn update_rate(
        &self,
        rate_measure: &AttitudeT,
        rate_target: &AttitudeRateTAtomic,
    ) -> AttitudeAccelT {
        let roll_accel = self
            .roll_rate
            .update(rate_target.roll.load(Ordering::Relaxed) - rate_measure.roll);
        let pitch_accel = self
            .pitch_rate
            .update(rate_target.pitch.load(Ordering::Relaxed) - rate_measure.pitch);
        let yaw_accel = self
            .yaw_rate
            .update(rate_target.yaw.load(Ordering::Relaxed) - rate_measure.yaw);
        AttitudeAccelT {
            roll: roll_accel,
            pitch: pitch_accel,
            yaw: yaw_accel,
        }
    }

    fn update_val(&self, val_measure: &AttitudeT, val_target: &AttitudeTAtomic) -> AttitudeRateT {
        let roll_rate = self
            .roll_val
            .update(val_target.roll.load(Ordering::Relaxed) - val_measure.roll);
        let pitch_rate = self
            .pitch_val
            .update(val_target.pitch.load(Ordering::Relaxed) - val_measure.pitch);
        /* angle range: -180.0 ~ 180.0 */
        let yaw_rate = self.yaw_val.update(canonicalize_angle(
            val_target.yaw.load(Ordering::Relaxed) - val_measure.yaw,
        ));
        AttitudeRateT {
            roll: roll_rate,
            pitch: pitch_rate,
            yaw: yaw_rate,
        }
    }

    pub fn update(
        &self,
        set_point: &SetPointT,
        sensor: &SensorT,
        state: &StateT,
        val_target: &AttitudeTAtomic,
        rate_target: &AttitudeRateTAtomic,
        thrust: &AtomicF32,
    ) -> ControlT {
        let mut yaw = val_target.yaw.load(Ordering::Relaxed);
        if set_point.mode_yaw == SetPointMode::Velocity {
            yaw += set_point.attitude_rate.yaw * (1.0 / ATTITUDE_UPDATE_RATE);
        } else if set_point.mode_yaw == SetPointMode::Abs {
            yaw = set_point.attitude.yaw;
        }
        yaw = canonicalize_angle(yaw);
        val_target.yaw.store(yaw, Ordering::Relaxed);

        if set_point.mode_z == SetPointMode::Disable {
            thrust.store(set_point.thrust, Ordering::Relaxed);
        }

        if set_point.mode_x == SetPointMode::Disable || set_point.mode_y == SetPointMode::Disable {
            val_target
                .roll
                .store(set_point.attitude.roll, Ordering::Relaxed);
            val_target
                .pitch
                .store(set_point.attitude.pitch, Ordering::Relaxed);
        }

        rate_target.store(
            self.update_val(&state.attitude, val_target),
            Ordering::Relaxed,
        );

        if set_point.mode_roll == SetPointMode::Velocity {
            rate_target
                .roll
                .store(set_point.attitude_rate.roll, Ordering::Relaxed);
            self.roll_val.reset();
        }

        if set_point.mode_pitch == SetPointMode::Velocity {
            rate_target
                .pitch
                .store(set_point.attitude_rate.pitch, Ordering::Relaxed);
            self.pitch_val.reset();
        }

        let gyro = AttitudeT {
            roll: sensor.gyro.roll,
            pitch: -sensor.gyro.pitch,
            yaw: sensor.gyro.yaw,
        };

        ControlT {
            attitude: self.update_rate(&gyro, rate_target),
            thrust: thrust.load(Ordering::Relaxed),
        }
    }

    #[allow(unused)]
    pub fn reset_rate(&self, ch: ChannelA) {
        match ch {
            ChannelA::Roll => self.roll_rate.reset(),
            ChannelA::Pitch => self.pitch_rate.reset(),
            ChannelA::Yaw => self.yaw_rate.reset(),
        }
    }

    pub fn reset_all(&self) {
        self.roll_val.reset();
        self.pitch_val.reset();
        self.yaw_val.reset();
        self.roll_rate.reset();
        self.pitch_rate.reset();
        self.yaw_rate.reset();
    }
}

/* Position controller */
#[derive(Debug)]
pub struct ControllerP {
    x_val: PID,
    x_rate: PID,
    y_val: PID,
    y_rate: PID,
    z_val: PID,
    z_rate: PID,
}

impl ControllerP {
    pub fn new() -> Self {
        Self {
            x_val: PID::new(2.0, 0.0, 0.0, POSITION_UPDATE_RATE, Some(1.0), None),
            x_rate: PID::new(12.0, 1.0, 0.3, POSITION_UPDATE_RATE, Some(25.0), Some(20.0)),
            y_val: PID::new(2.0, 0.0, 0.0, POSITION_UPDATE_RATE, Some(1.0), None),
            y_rate: PID::new(12.0, 1.0, 0.3, POSITION_UPDATE_RATE, Some(25.0), Some(20.0)),
            z_val: PID::new(2.0, 0.5, 0.0, POSITION_UPDATE_RATE, Some(1.0), None),
            z_rate: PID::new(
                25.0,
                15.0,
                0.3,
                POSITION_UPDATE_RATE,
                Some(20.0),
                Some(i16::MAX as f32 / MOTOR_THRUST_SCALE),
            ),
        }
    }

    pub fn reset_all(&self) {
        self.x_val.reset();
        self.y_val.reset();
        self.z_val.reset();
        self.x_rate.reset();
        self.y_rate.reset();
        self.z_rate.reset();
    }

    pub fn update(
        &self,
        set_point: &SetPointT,
        state: &StateT,
        val_target: &AttitudeTAtomic,
        thrust: &AtomicF32,
    ) {
        let cos_yaw = libm::cosf(state.attitude.yaw.to_radians());
        let sin_yaw = libm::sinf(state.attitude.yaw.to_radians());

        let v_x: f32;
        let v_y: f32;
        let v_z: f32;

        if set_point.mode_x == SetPointMode::Abs {
            v_x = self.x_val.update(set_point.position.x - state.position.x);
        } else if set_point.body_velocity {
            v_x = set_point.velocity.x * cos_yaw - set_point.velocity.y * sin_yaw;
        } else {
            v_x = set_point.velocity.x;
        }

        if set_point.mode_y == SetPointMode::Abs {
            v_y = self.y_val.update(set_point.position.y - state.position.y);
        } else if set_point.body_velocity {
            v_y = set_point.velocity.x * sin_yaw + set_point.velocity.y * cos_yaw;
        } else {
            v_y = set_point.velocity.y;
        }

        if set_point.mode_z == SetPointMode::Abs {
            v_z = self.z_val.update(set_point.position.z - state.position.z);
        } else {
            v_z = set_point.velocity.z;
        }

        let accel_x = self.x_rate.update(v_x - state.velocity.x);
        let accel_y = self.y_rate.update(v_y - state.velocity.y);
        let accel_z = self.z_rate.update(v_z - state.velocity.z);

        // convert acceleration in x,y to roll,pitch
        val_target
            .roll
            .store(accel_x * sin_yaw - accel_y * cos_yaw, Ordering::Relaxed);
        val_target
            .pitch
            .store(-accel_x * cos_yaw - accel_y * sin_yaw, Ordering::Relaxed);
        thrust.store(
            (accel_z * MOTOR_THRUST_SCALE + MOTOR_THRUST_BASE)
                .clamp(MOTOR_THRUST_MIN, MOTOR_THRUST_MAX),
            Ordering::Relaxed,
        );
    }
}

#[derive(Debug)]
pub struct Controller {
    pub a: ControllerA,
    pub p: ControllerP,
    pub control: ControlTAtomic,
    pub val_target: AttitudeTAtomic,
    pub rate_target: AttitudeRateTAtomic,
    pub thrust: AtomicF32,
}

impl Controller {
    pub fn new() -> Self {
        Self {
            a: ControllerA::new(),
            p: ControllerP::new(),
            control: ControlTAtomic::default(),
            val_target: AttitudeTAtomic::default(),
            rate_target: AttitudeRateTAtomic::default(),
            thrust: AtomicF32::new(0.0),
        }
    }

    fn should_update(count: u64, freq: f32) -> bool {
        count % (1000.0 / freq) as u64 == 0
    }

    pub fn update(
        &self,
        set_point: &SetPointT,
        sensor: &SensorT,
        state: &StateT,
        count: u64,
    ) -> ControlT {
        if Self::should_update(count, POSITION_UPDATE_RATE) {
            self.p
                .update(set_point, state, &self.val_target, &self.thrust);
        }

        if Self::should_update(count, ATTITUDE_UPDATE_RATE) {
            self.control.store(
                self.a.update(
                    set_point,
                    sensor,
                    state,
                    &self.val_target,
                    &self.rate_target,
                    &self.thrust,
                ),
                Ordering::Relaxed,
            );
        }

        if self.control.thrust.load(Ordering::Relaxed) < MOTOR_THRUST_MIN {
            self.a.reset_all();
            self.p.reset_all();
            // avoid drone from rotating when flying after a previous landing
            self.val_target
                .yaw
                .store(state.attitude.yaw, Ordering::Relaxed);
            self.control.store(ControlT::default(), Ordering::Relaxed);
        }
        return self.control.load(Ordering::Relaxed);
    }
}

pub fn canonicalize_angle(angle: f32) -> f32 {
    if angle > 180.0 {
        angle - 360.0
    } else if angle < -180.0 {
        angle + 360.0
    } else {
        angle
    }
}
