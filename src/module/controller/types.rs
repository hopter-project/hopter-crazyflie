use core::sync::atomic::Ordering;

use crate::driver::bmi08x::bmi08x_sensor_data;
use atomic_float::AtomicF32;

#[derive(Copy, Clone, Default, Debug)]
pub struct AttitudeT {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl From<bmi08x_sensor_data> for AttitudeT {
    fn from(v: bmi08x_sensor_data) -> Self {
        Self {
            roll: v.x as f32,
            pitch: v.y as f32,
            yaw: v.z as f32,
        }
    }
}

#[derive(Default, Debug)]
pub struct AttitudeTAtomic {
    pub roll: AtomicF32,
    pub pitch: AtomicF32,
    pub yaw: AtomicF32,
}

impl AttitudeTAtomic {
    pub fn load(&self, ordering: Ordering) -> AttitudeT {
        AttitudeT {
            roll: self.roll.load(ordering),
            pitch: self.pitch.load(ordering),
            yaw: self.yaw.load(ordering),
        }
    }

    pub fn store(&self, val: AttitudeT, ordering: Ordering) {
        self.roll.store(val.roll, ordering);
        self.pitch.store(val.pitch, ordering);
        self.yaw.store(val.yaw, ordering);
    }
}

pub type AttitudeRateT = AttitudeT;
pub type AttitudeAccelT = AttitudeT;

pub type AttitudeRateTAtomic = AttitudeTAtomic;

#[derive(Copy, Clone, Default, Debug)]
pub struct PositionT {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl From<bmi08x_sensor_data> for PositionT {
    fn from(v: bmi08x_sensor_data) -> Self {
        Self {
            x: v.x as f32,
            y: v.y as f32,
            z: v.z as f32,
        }
    }
}

pub type VelocityT = PositionT;
pub type AccelerationT = PositionT;

#[derive(Copy, Clone, Default, Debug)]
pub struct StateT {
    pub timestamp: u64,
    pub attitude: AttitudeT,
    pub position: PositionT,
    pub velocity: VelocityT,
    pub accel: AccelerationT,
}

#[allow(unused)]
#[derive(Copy, Clone, Default, Debug)]
pub struct SensorT {
    pub timestamp: u64,
    pub gyro: AttitudeT,
    pub accel: AccelerationT,
}

#[derive(Copy, Clone, Default, Debug)]
pub struct ControlT {
    pub attitude: AttitudeT,
    pub thrust: f32,
}

#[derive(Default, Debug)]
pub struct ControlTAtomic {
    pub attitude: AttitudeTAtomic,
    pub thrust: AtomicF32,
}

impl ControlTAtomic {
    pub fn load(&self, ordering: Ordering) -> ControlT {
        ControlT {
            attitude: self.attitude.load(ordering),
            thrust: self.thrust.load(ordering),
        }
    }

    pub fn store(&self, val: ControlT, ordering: Ordering) {
        self.attitude.store(val.attitude, ordering);
        self.thrust.store(val.thrust, ordering);
    }
}

#[derive(Copy, Clone, Default, Debug, PartialEq)]
pub enum SetPointMode {
    #[default]
    Disable = 0,
    Abs,
    Velocity,
}

#[allow(unused)]
#[derive(Copy, Clone, Default, Debug)]
pub struct SetPointT {
    pub timestamp: u64,
    pub attitude: AttitudeT,
    pub attitude_rate: AttitudeRateT,
    pub position: PositionT,
    pub velocity: VelocityT,
    pub thrust: f32,
    pub body_velocity: bool,

    pub mode_x: SetPointMode,
    pub mode_y: SetPointMode,
    pub mode_z: SetPointMode,
    pub mode_roll: SetPointMode,
    pub mode_pitch: SetPointMode,
    pub mode_yaw: SetPointMode,
}

impl SetPointT {
    pub fn default_hover_packet() -> Self {
        Self {
            mode_x: SetPointMode::Velocity,
            mode_y: SetPointMode::Velocity,
            mode_z: SetPointMode::Abs,
            mode_roll: SetPointMode::Disable,
            mode_pitch: SetPointMode::Disable,
            mode_yaw: SetPointMode::Velocity,
            body_velocity: true,
            ..Default::default()
        }
    }
}
