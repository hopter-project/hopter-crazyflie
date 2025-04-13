/*
 * Time of Flight (ToF) sensor: vl53l1, measures the drone height
 */

use crate::driver::{
    serial::i2c,
    vl53l1::{
        vl53l1::*,
        vl53l1_def::{VL53L1_DevData_t, VL53L1_RangingMeasurementData_t},
    },
};
use hopter::interrupt::declare::{handler, irq};
use stm32f4xx_hal::prelude::*;

pub struct ToF {
    rate: u32,
    dev: VL53L1_Dev_t,
    is_valid: bool,
}

#[allow(unused)]
const RANGE_OUTLIER_LIMIT: f32 = 5.0;

#[derive(Debug, Clone, Copy)]
pub struct ToFData {
    pub distance: f32,
    pub std_dev: f32,
}

impl ToF {
    pub fn new() -> Self {
        Self {
            rate: 25,
            dev: VL53L1_Dev_t {
                Data: VL53L1_DevData_t::default(),
                i2c_slave_address: 0x29,
                comms_type: 0,
                comms_speed_khz: 0,
                new_data_ready_poll_duration_ms: 0,
            },
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

    pub fn task_loop(&mut self) -> Option<ToFData> {
        if !self.is_valid {
            return None;
        }

        let mut data = VL53L1_RangingMeasurementData_t::default();
        self.dev.VL53L1_WaitMeasurementDataReady();
        self.dev.VL53L1_GetRangingMeasurementData(&mut data);
        self.dev.VL53L1_ClearInterruptAndStartMeasurement();
        let distance = data.RangeMilliMeter as f32 / 1000.0;
        Some(ToFData {
            distance,
            std_dev: 0.0025 * (1.0 + libm::expf(2.92135 * (distance - 2.5))),
        })
    }
}

pub fn i2c1_read(reg_addr: u16, reg_data: &mut [u8], intf: u32) -> i8 {
    i2c::i2c_dma_read(
        super::acquire_irqsafe_i2c1_dma_handle,
        super::borrow_i2c1_sem,
        &[(reg_addr >> 8) as u8, (reg_addr & 0xFF) as u8],
        reg_data,
        intf,
    )
}

pub fn i2c1_write(reg_addr: u16, reg_data: &[u8], intf: u32) -> i8 {
    i2c::i2c_write(
        super::acquire_irqsafe_i2c1_dma_handle,
        &[(reg_addr >> 8) as u8, (reg_addr & 0xFF) as u8],
        reg_data,
        intf,
    )
}

irq!(Dma1Stream0Irq, stm32f4xx_hal::pac::Interrupt::DMA1_STREAM0);
irq!(Dma1Stream6Irq, stm32f4xx_hal::pac::Interrupt::DMA1_STREAM6);
pub type I2C1DmaIrq = (Dma1Stream0Irq, Dma1Stream6Irq);

/// Handle DMA interrupt. Increase the semaphore count so that the task waiting
/// for DMA read/write to finish can resume.
#[handler(DMA1_STREAM0)]
fn dma1_stream0_handler() {
    let sem = super::borrow_i2c1_sem();
    {
        let mut handle = super::acquire_irqsafe_i2c1_dma_handle();
        handle.handle_dma_interrupt();
    }

    let _ = sem.try_up_allow_isr();
}

/// Handle DMA interrupt for the write part of a DMA write-read transaction,
/// so that the read part can proceed.
#[handler(DMA1_STREAM6)]
fn dma1_stream6_handler() {
    {
        let mut handle = super::acquire_irqsafe_i2c1_dma_handle();
        handle.handle_dma_interrupt();
    }
}
