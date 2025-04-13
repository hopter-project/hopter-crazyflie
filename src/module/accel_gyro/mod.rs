use super::{
    controller::types::{AccelerationT, AttitudeRateT, SensorT},
    filter::Lpf2d,
};
use crate::driver::{bmi08x::*, serial::i2c};
use alloc::vec;
use hopter::interrupt::declare::{handler, irq};
use stm32f4xx_hal::prelude::*;

const ACCEL_SCALE_NUMBER: usize = 256;
const GYRO_BIAS_NUMBER: usize = 512;
const GYRO_BIAS_VARIANCE_THRESHOLD: f32 = 100.0;

pub struct AccelGyro {
    rate: u32,
    dev: bmi08x_dev,
    accel_filter: [Lpf2d; 3],
    gyro_filter: [Lpf2d; 3],
    accel_scale_buffer: Option<vec::Vec<bmi08x_sensor_data>>,
    gyro_bias_buffer: Option<vec::Vec<bmi08x_sensor_data>>,
    accel_scale_value: f32,
    gyro_bias_value: [f32; 3],
    calibration_done: bool,
}

impl AccelGyro {
    pub fn new() -> Self {
        Self {
            rate: 1000,
            dev: bmi08x_dev {
                accel_chip_id: 0,
                gyro_chip_id: 0,
                intf_ptr_accel: BMI08X_ACCEL_I2C_ADDR_PRIMARY as u32,
                intf_ptr_gyro: BMI08X_GYRO_I2C_ADDR_SECONDARY as u32,
                intf: bmi08x_intf::BMI08X_I2C_INTF,
                variant: bmi08x_variant::BMI088_VARIANT,
                dummy: 0,
                accel_cfg: bmi08x_cfg {
                    power: BMI08X_ACCEL_PM_ACTIVE,
                    range: BMI088_ACCEL_RANGE_24G,
                    bw: BMI08X_ACCEL_BW_OSR4,
                    odr: BMI08X_ACCEL_ODR_1600_HZ,
                },
                gyro_cfg: bmi08x_cfg {
                    power: BMI08X_GYRO_PM_NORMAL,
                    range: BMI08X_GYRO_RANGE_2000_DPS,
                    bw: BMI08X_GYRO_BW_116_ODR_1000_HZ,
                    odr: BMI08X_GYRO_BW_116_ODR_1000_HZ,
                },
                read_write_len: 32,
                intf_rslt: BMI08X_OK,
            },
            accel_filter: [Lpf2d::new(1000.0, 30.0); 3],
            gyro_filter: [Lpf2d::new(1000.0, 80.0); 3],
            accel_scale_buffer: Some(vec::Vec::new()),
            gyro_bias_buffer: Some(vec::Vec::new()),
            accel_scale_value: 1.0,
            gyro_bias_value: [0.0; 3],
            calibration_done: false,
        }
    }

    pub fn init(&mut self) -> i8 {
        let mut rslt = self.dev.bmi08x_init();
        if rslt == BMI08X_OK {
            dbg_println!("BMI088 init [OK]");
            rslt = self.dev.bmi08g_set_int_config(bmi08x_gyro_int_channel_cfg {
                int_channel: bmi08x_gyro_int_channel::BMI08X_INT_CHANNEL_3,
                int_type: bmi08x_gyro_int_types::BMI08X_GYRO_INT_DATA_RDY,
                int_pin_cfg: bmi08x_int_pin_cfg {
                    lvl: BMI08X_INT_ACTIVE_HIGH,
                    output_mode: BMI08X_INT_MODE_PUSH_PULL,
                    enable_int_pin: BMI08X_ENABLE,
                },
            });
            if rslt == BMI08X_OK {
                dbg_println!("BMI088 gyro int config [OK]");
            } else {
                dbg_println!("BMI088 gyro int config [FAILED]");
            }
        } else {
            rslt = -1;
            dbg_println!("BMI088 init [FAILED]");
        }
        return rslt;
    }

    pub fn get_interval_ms(&self) -> u32 {
        1000 / self.rate
    }

    pub fn get_accel(&mut self, accel_data: &mut bmi08x_sensor_data) -> i8 {
        self.dev.bmi08a_get_data(accel_data)
    }

    pub fn get_gyro(&mut self, gyro_data: &mut bmi08x_sensor_data) -> i8 {
        self.dev.bmi08g_get_data(gyro_data)
    }

    pub fn accel_to_gravity(accel: f32) -> f32 {
        accel * 24.0 / 32768.0
    }

    pub fn gyro_to_degree(gyro: f32) -> f32 {
        gyro * 2000.0 / 32768.0
    }

    pub fn task_loop(&mut self) -> SensorT {
        // get data from accel and gyro
        let mut accel_sensor_data = bmi08x_sensor_data::default();
        let mut gyro_sensor_data = bmi08x_sensor_data::default();
        self.get_accel(&mut accel_sensor_data);
        self.get_gyro(&mut gyro_sensor_data);

        // accel scale calibration
        if let Some(accel_scale) = &mut self.accel_scale_buffer {
            if accel_scale.len() < ACCEL_SCALE_NUMBER {
                accel_scale.push(accel_sensor_data);
            } else if accel_scale.len() == ACCEL_SCALE_NUMBER {
                let mut accel_scale_sum = 0_f32;
                for accel in accel_scale {
                    let fx = accel.x as f32;
                    let fy = accel.y as f32;
                    let fz = accel.z as f32;
                    accel_scale_sum += libm::sqrtf(fx * fx + fy * fy + fz * fz);
                }

                self.accel_scale_value =
                    Self::accel_to_gravity(accel_scale_sum / ACCEL_SCALE_NUMBER as f32);
                self.accel_scale_buffer = None;
                dbg_println!("accel scale calibration [OK]");
            }
        }

        let mut accel_data = AccelerationT {
            x: Self::accel_to_gravity(accel_sensor_data.x as f32) / self.accel_scale_value,
            y: Self::accel_to_gravity(accel_sensor_data.y as f32) / self.accel_scale_value,
            z: Self::accel_to_gravity(accel_sensor_data.z as f32) / self.accel_scale_value,
        };
        accel_data.x = self.accel_filter[0].update(accel_data.x);
        accel_data.y = self.accel_filter[1].update(accel_data.y);
        accel_data.z = self.accel_filter[2].update(accel_data.z);

        // gyro bias calibration
        if let Some(gyro_bias) = &mut self.gyro_bias_buffer {
            if gyro_bias.len() < GYRO_BIAS_NUMBER {
                gyro_bias.push(gyro_sensor_data);
            } else if gyro_bias.len() == GYRO_BIAS_NUMBER {
                let mut gyro_bias_sum = [0_f32; 3];
                let mut gyro_bias_sum_sq = [0_f32; 3];
                for gyro in gyro_bias {
                    gyro_bias_sum[0] += gyro.x as f32;
                    gyro_bias_sum[1] += gyro.y as f32;
                    gyro_bias_sum[2] += gyro.z as f32;
                    gyro_bias_sum_sq[0] += gyro.x as f32 * gyro.x as f32;
                    gyro_bias_sum_sq[1] += gyro.y as f32 * gyro.y as f32;
                    gyro_bias_sum_sq[2] += gyro.z as f32 * gyro.z as f32;
                }

                // mean
                gyro_bias_sum[0] /= GYRO_BIAS_NUMBER as f32;
                gyro_bias_sum[1] /= GYRO_BIAS_NUMBER as f32;
                gyro_bias_sum[2] /= GYRO_BIAS_NUMBER as f32;
                // variance
                let gyro_bias_var = [
                    gyro_bias_sum_sq[0] / GYRO_BIAS_NUMBER as f32
                        - gyro_bias_sum[0] * gyro_bias_sum[0],
                    gyro_bias_sum_sq[1] / GYRO_BIAS_NUMBER as f32
                        - gyro_bias_sum[1] * gyro_bias_sum[1],
                    gyro_bias_sum_sq[2] / GYRO_BIAS_NUMBER as f32
                        - gyro_bias_sum[2] * gyro_bias_sum[2],
                ];

                // if variance is too large, discard this data
                if gyro_bias_var[0] > GYRO_BIAS_VARIANCE_THRESHOLD
                    || gyro_bias_var[1] > GYRO_BIAS_VARIANCE_THRESHOLD
                    || gyro_bias_var[2] > GYRO_BIAS_VARIANCE_THRESHOLD
                {
                    self.gyro_bias_buffer = Some(vec::Vec::new());
                } else {
                    self.gyro_bias_value = [
                        Self::gyro_to_degree(gyro_bias_sum[0]),
                        Self::gyro_to_degree(gyro_bias_sum[1]),
                        Self::gyro_to_degree(gyro_bias_sum[2]),
                    ];
                    self.gyro_bias_buffer = None;
                    dbg_println!("gyro bias calibration [OK]");
                }
            }
        }

        if self.gyro_bias_buffer.is_none()
            && self.accel_scale_buffer.is_none()
            && self.calibration_done == false
        {
            self.calibration_done = true;
            {
                let sem = super::borrow_system_start_wait_sem();
                sem.up();
                sem.up();
                sem.up();
                sem.up();
            }
        }

        // gyro data, the minus sign is because the sensor is mounted upside down
        let mut gyro_data = AttitudeRateT {
            roll: Self::gyro_to_degree(gyro_sensor_data.x as f32 - self.gyro_bias_value[0]),
            pitch: Self::gyro_to_degree(gyro_sensor_data.y as f32 - self.gyro_bias_value[1]),
            yaw: Self::gyro_to_degree(gyro_sensor_data.z as f32 - self.gyro_bias_value[2]),
        };
        gyro_data.roll = self.gyro_filter[0].update(gyro_data.roll);
        gyro_data.pitch = self.gyro_filter[1].update(gyro_data.pitch);
        gyro_data.yaw = self.gyro_filter[2].update(gyro_data.yaw);

        SensorT {
            timestamp: 0,
            accel: accel_data,
            gyro: gyro_data,
        }
    }
}

pub fn i2c3_read(reg_addr: u8, reg_data: &mut [u8], intf: u32) -> i8 {
    i2c::i2c_dma_read(
        super::acquire_irqsafe_i2c3_dma_handle,
        super::borrow_i2c3_sem,
        &[reg_addr],
        reg_data,
        intf,
    )
}

pub fn i2c3_write(reg_addr: u8, reg_data: &[u8], intf: u32) -> i8 {
    i2c::i2c_write(
        super::acquire_irqsafe_i2c3_dma_handle,
        &[reg_addr],
        reg_data,
        intf,
    )
}

irq!(Dma1Stream2Irq, stm32f4xx_hal::pac::Interrupt::DMA1_STREAM2);
irq!(Dma1Stream4Irq, stm32f4xx_hal::pac::Interrupt::DMA1_STREAM4);
pub type I2C3DmaIrq = (Dma1Stream2Irq, Dma1Stream4Irq);

/// Handle DMA interrupt. Increase the semaphore count so that the task waiting
/// for DMA read/write to finish can resume.
#[handler(DMA1_STREAM2)]
fn dma1_stream2_handler() {
    let sem = super::borrow_i2c3_sem();
    {
        let mut handle = super::acquire_irqsafe_i2c3_dma_handle();
        handle.handle_dma_interrupt();
    }

    let _ = sem.try_up_allow_isr();
}

/// Handle DMA interrupt for the write part of a DMA write-read transaction,
/// so that the read part can proceed.
#[handler(DMA1_STREAM4)]
fn dma1_stream4_handler() {
    {
        let mut handle = super::acquire_irqsafe_i2c3_dma_handle();
        handle.handle_dma_interrupt();
    }
}
