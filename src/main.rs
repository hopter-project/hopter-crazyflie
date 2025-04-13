#![no_std]
#![no_main]
#![feature(asm_const)]
#![feature(never_type)]
#![feature(naked_functions)]
#![feature(alloc_error_handler)]
#![feature(negative_impls)]
#![feature(generic_arg_infer)]
#![allow(internal_features)]
#![feature(lang_items)]
#![feature(core_intrinsics)]
#![feature(new_uninit)]
#![feature(raw_ref_op)]

extern crate alloc;
#[macro_use]
mod debug;
mod driver;
mod flight_tasks;
mod math;
mod module;

use hopter::{sync, task::main};
use module::{
    controller::types::{SensorT, SetPointT, StateT},
    crtp::SystemPacket,
    flow::FlowData,
    tof::ToFData,
};
use stm32f4xx_hal::pac;

#[main]
fn main(cp: cortex_m::Peripherals) {
    let dp = pac::Peripherals::take().unwrap();
    module::init_peripherals(cp, dp);

    let (prod_stabilizer_accel_gyro, cons_stabilizer_accel_gyro) =
        sync::create_channel::<SensorT, 2>();
    let (prod_estimator_accel_gyro, cons_estimator_accel_gyro) =
        sync::create_channel::<SensorT, 2>();
    let (prod_estimator_tof, cons_tof) = sync::create_channel::<ToFData, 2>();
    let (prod_estimator_flow, cons_flow) = sync::create_channel::<FlowData, 2>();
    let (prod_estimator_state, cons_state) = sync::create_channel::<StateT, 2>();
    let (prod_stabilizer_setpoint, cons_setpoint) = sync::create_channel::<SetPointT, 2>();
    let (prod_crtprx_system_packet, cons_system_packet) =
        sync::create_channel::<SystemPacket, 16>();

    {
        module::acquire_irqsafe_radio_rx_link().set_sender(prod_crtprx_system_packet);
    }

    flight_tasks::build_led_task();
    flight_tasks::build_sensor_accel_gyro_task(
        prod_stabilizer_accel_gyro,
        prod_estimator_accel_gyro,
    );
    flight_tasks::build_tof_task(prod_estimator_tof);
    flight_tasks::build_flow_task(prod_estimator_flow);
    flight_tasks::build_estimator_kalman_task(
        cons_estimator_accel_gyro,
        cons_tof,
        cons_flow,
        prod_estimator_state,
    );
    flight_tasks::build_crtp_rx_task(cons_system_packet, prod_stabilizer_setpoint);
    flight_tasks::build_stabilizer_task(cons_stabilizer_accel_gyro, cons_state, cons_setpoint);
    flight_tasks::build_watch_dog_task();

    #[cfg(not(feature = "no_dbg_print"))]
    debug::build_debug_task();
}
