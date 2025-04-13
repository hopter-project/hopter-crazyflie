mod accel_gyro;
mod crtp_rx;
mod crtp_tx;
mod estimator;
mod flow;
#[allow(unused)]
mod led;
mod stabilizer;
mod tof;
mod watch_dog;

pub use accel_gyro::*;
pub use crtp_rx::*;
#[allow(unused)]
pub use crtp_tx::*;
pub use estimator::*;
pub use flow::*;
#[allow(unused)]
pub use led::*;
pub use stabilizer::*;
pub use tof::*;
pub use watch_dog::*;

pub const TASK_ID_LED: u8 = 3;
pub const TASK_ID_ACCEL_GYRO: u8 = 4;
pub const TASK_ID_TOF: u8 = 5;
pub const TASK_ID_FLOW: u8 = 6;
pub const TASK_ID_ESTIMATOR: u8 = 7;
pub const TASK_ID_CRTP_RX: u8 = 8;
pub const TASK_ID_STABILIZER: u8 = 9;
#[allow(unused)]
pub const TASK_ID_CRTP_TX: u8 = 10;
pub const TASK_ID_WATCH_DOG: u8 = 11;
#[allow(unused)]
pub const TASK_ID_DEBUG_PRINTER: u8 = 12;

pub const TASK_PRIO_LED: u8 = 12;
pub const TASK_PRIO_ACCEL_GYRO: u8 = 8;
pub const TASK_PRIO_TOF: u8 = 8;
pub const TASK_PRIO_FLOW: u8 = 8;
pub const TASK_PRIO_ESTIMATOR: u8 = 8;
pub const TASK_PRIO_CRTP_RX: u8 = 8;
pub const TASK_PRIO_STABILIZER: u8 = 6;
#[allow(unused)]
pub const TASK_PRIO_CRTP_TX: u8 = 8;
pub const TASK_PRIO_WATCH_DOG: u8 = 4;
#[allow(unused)]
pub const TASK_PRIO_DEBUG_PRINTER: u8 = 11;

pub const TASK_STK_SIZE_ACCEL_GYRO: usize = 3072;
pub const TASK_STK_SIZE_TOF: usize = 3072;
pub const TASK_STK_SIZE_FLOW: usize = 3072;
pub const TASK_STK_SIZE_ESTIMATOR: usize = 3072;
pub const TASK_STK_SIZE_CRTP_RX: usize = 3072;
pub const TASK_STK_SIZE_STABILIZER: usize = 3072;
pub const TASK_STK_SIZE_WATCH_DOG: usize = 3072;
pub const TASK_STK_SIZE_LED: usize = 3072;
