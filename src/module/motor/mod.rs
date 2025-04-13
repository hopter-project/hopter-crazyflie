use super::controller::types::ControlT;
use stm32f4xx_hal::{
    pac::{TIM2, TIM4},
    timer::pwm::PwmChannel,
};

pub const MOTOR_VALUE_BIT: u8 = 8; // max 256
pub const MOTOR_THRUST_SCALE: f32 = 1000.0;
pub const MOTOR_THRUST_BASE: f32 = 36000.0;
pub const MOTOR_THRUST_MAX: f32 = 60000.0;
pub const MOTOR_THRUST_MIN: f32 = 1000.0;

/*
 * ch0: PA1 TIM2 CH2
 * ch1: PB11 TIM2 CH4
 * ch2: PA15 TIM2 CH1
 * ch3: PB9 TIM4 CH4
 */
pub struct Motor {
    ch0: PwmChannel<TIM2, 1>,
    ch1: PwmChannel<TIM2, 3>,
    ch2: PwmChannel<TIM2, 0>,
    ch3: PwmChannel<TIM4, 3>,
    pub is_flying: bool,
}

impl Motor {
    pub fn new(
        ch0: PwmChannel<TIM2, 1>,
        ch1: PwmChannel<TIM2, 3>,
        ch2: PwmChannel<TIM2, 0>,
        ch3: PwmChannel<TIM4, 3>,
    ) -> Self {
        let mut motor = Self {
            ch0,
            ch1,
            ch2,
            ch3,
            is_flying: false,
        };
        motor.init();
        motor
    }

    pub fn init(&mut self) {
        self.ch0.enable();
        self.ch1.enable();
        self.ch2.enable();
        self.ch3.enable();

        // TODO: add beep
    }

    pub fn set_ratio(&mut self, id: u8, thrust: u16) {
        let duty: u16 = (thrust) >> (16 - MOTOR_VALUE_BIT) & ((1 << MOTOR_VALUE_BIT) - 1);

        match id {
            0 => self.ch0.set_duty(duty),
            1 => self.ch1.set_duty(duty),
            2 => self.ch2.set_duty(duty),
            3 => self.ch3.set_duty(duty),
            _ => {
                panic!("Invalid motor id");
            }
        }
    }

    pub fn power_distribution(&mut self, control: ControlT) {
        let t = control.thrust;
        let r = control.attitude.roll / 2.0;
        let p = control.attitude.pitch / 2.0;
        let y = control.attitude.yaw;
        self.set_ratio(0, clamp_f32_to_u16(t - r + p - y));
        self.set_ratio(1, clamp_f32_to_u16(t - r - p + y));
        self.set_ratio(2, clamp_f32_to_u16(t + r - p - y));
        self.set_ratio(3, clamp_f32_to_u16(t + r + p + y));
        if t > MOTOR_THRUST_MIN {
            self.is_flying = true;
        } else {
            self.is_flying = false;
        }
    }

    pub fn stop(&mut self) {
        self.set_ratio(0, 0);
        self.set_ratio(1, 0);
        self.set_ratio(2, 0);
        self.set_ratio(3, 0);
    }
}

fn clamp_f32_to_u16(val: f32) -> u16 {
    libm::roundf(val).clamp(0.0, u16::MAX as f32) as u16
}
