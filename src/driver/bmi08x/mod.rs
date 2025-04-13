#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(unused)]
use alloc::{format, vec};
use stm32f4xx_hal::pac::can1::btr::R;

pub const BMI08X_INTF_RET_SUCCESS: i8 = 0;

/*************************** BMI08 Accelerometer Macros *****************************/

/** Register map */
/* Accel registers */

/* Accel Chip Id register */
pub const BMI08X_REG_ACCEL_CHIP_ID: u8 = 0x00;

/* Accel Error condition register */
pub const BMI08X_REG_ACCEL_ERR: u8 = 0x02;

/* Accel Status flag register */
pub const BMI08X_REG_ACCEL_STATUS: u8 = 0x03;

/* Accel X LSB data register */
pub const BMI08X_REG_ACCEL_X_LSB: u8 = 0x12;

/* Accel X MSB data register */
pub const BMI08X_REG_ACCEL_X_MSB: u8 = 0x13;

/* Accel Y LSB data register */
pub const BMI08X_REG_ACCEL_Y_LSB: u8 = 0x14;

/* Accel Y MSB data register */
pub const BMI08X_REG_ACCEL_Y_MSB: u8 = 0x15;

/* Accel Z LSB data register */
pub const BMI08X_REG_ACCEL_Z_LSB: u8 = 0x16;

/* Accel Z MSB data register */
pub const BMI08X_REG_ACCEL_Z_MSB: u8 = 0x17;

/* Sensor time byte 0 register */
pub const BMI08X_REG_ACCEL_SENSORTIME_0: u8 = 0x18;

/* Sensor time byte 1 register */
pub const BMI08X_REG_ACCEL_SENSORTIME_1: u8 = 0x19;

/* Sensor time byte 2 register */
pub const BMI08X_REG_ACCEL_SENSORTIME_2: u8 = 0x1A;

/* Accel Interrupt status0 register */
pub const BMI08X_REG_ACCEL_INT_STAT_0: u8 = 0x1C;

/* Accel Interrupt status1 register */
pub const BMI08X_REG_ACCEL_INT_STAT_1: u8 = 0x1D;

/* Accel general purpose register 0*/
pub const BMI08X_REG_ACCEL_GP_0: u8 = 0x1E;

/* Sensor temperature MSB data register */
pub const BMI08X_REG_TEMP_MSB: u8 = 0x22;

/* Sensor temperature LSB data register */
pub const BMI08X_REG_TEMP_LSB: u8 = 0x23;

/* Accel general purpose register 4*/
pub const BMI08X_REG_ACCEL_GP_4: u8 = 0x27;

/* Accel Internal status register */
pub const BMI08X_REG_ACCEL_INTERNAL_STAT: u8 = 0x2A;

/* Accel configuration register */
pub const BMI08X_REG_ACCEL_CONF: u8 = 0x40;

/* Accel range setting register */
pub const BMI08X_REG_ACCEL_RANGE: u8 = 0x41;

/* Accel Interrupt pin 1 configuration register */
pub const BMI08X_REG_ACCEL_INT1_IO_CONF: u8 = 0x53;

/* Accel Interrupt pin 2 configuration register */
pub const BMI08X_REG_ACCEL_INT2_IO_CONF: u8 = 0x54;

/* Accel Interrupt latch configuration register */
pub const BMI08X_REG_ACCEL_INT_LATCH_CONF: u8 = 0x55;

/* Accel Interrupt pin1 mapping register */
pub const BMI08X_REG_ACCEL_INT1_MAP: u8 = 0x56;

/* Accel Interrupt pin2 mapping register */
pub const BMI08X_REG_ACCEL_INT2_MAP: u8 = 0x57;

/* Accel Interrupt map register */
pub const BMI08X_REG_ACCEL_INT1_INT2_MAP_DATA: u8 = 0x58;

/* Accel Init control register */
pub const BMI08X_REG_ACCEL_INIT_CTRL: u8 = 0x59;

/* Accel Self test register */
pub const BMI08X_REG_ACCEL_SELF_TEST: u8 = 0x6D;

/* Accel Power mode configuration register */
pub const BMI08X_REG_ACCEL_PWR_CONF: u8 = 0x7C;

/* Accel Power control (switch on or off ; register */
pub const BMI08X_REG_ACCEL_PWR_CTRL: u8 = 0x7D;

/* Accel Soft reset register */
pub const BMI08X_REG_ACCEL_SOFTRESET: u8 = 0x7E;

/* Feature Config related Registers */
pub const BMI08X_REG_ACCEL_RESERVED_5B: u8 = 0x5B;
pub const BMI08X_REG_ACCEL_RESERVED_5C: u8 = 0x5C;
pub const BMI08X_REG_ACCEL_FEATURE_CFG: u8 = 0x5E;

/* BMI085 Accel unique chip identifier */
pub const BMI085_ACCEL_CHIP_ID: u8 = 0x1F;

/* BMI088 Accel unique chip identifier */
pub const BMI088_ACCEL_CHIP_ID: u8 = 0x1E;

/* Accel I2C slave address */
pub const BMI08X_ACCEL_I2C_ADDR_PRIMARY: u8 = 0x18;
pub const BMI08X_ACCEL_I2C_ADDR_SECONDARY: u8 = 0x19;

/* Interrupt masks */
pub const BMI08X_ACCEL_DATA_READY_INT: u8 = 0x80;
pub const BMI08X_ACCEL_FIFO_WM_INT: u8 = 0x02;
pub const BMI08X_ACCEL_FIFO_FULL_INT: u8 = 0x01;

pub const BMI08X_GYRO_DATA_READY_INT: u8 = 0x80;
pub const BMI08X_GYRO_FIFO_WM_INT: u8 = 0x10;
pub const BMI08X_GYRO_FIFO_FULL_INT: u8 = 0x10;

/* Accel Bandwidth */
pub const BMI08X_ACCEL_BW_OSR4: u8 = 0x08;
pub const BMI08X_ACCEL_BW_OSR2: u8 = 0x09;
pub const BMI08X_ACCEL_BW_NORMAL: u8 = 0x0A;

/* BMI085 Accel Range */
pub const BMI085_ACCEL_RANGE_2G: u8 = 0x00;
pub const BMI085_ACCEL_RANGE_4G: u8 = 0x01;
pub const BMI085_ACCEL_RANGE_8G: u8 = 0x02;
pub const BMI085_ACCEL_RANGE_16G: u8 = 0x03;

/*  BMI088 Accel Range */
pub const BMI088_ACCEL_RANGE_3G: u8 = 0x00;
pub const BMI088_ACCEL_RANGE_6G: u8 = 0x01;
pub const BMI088_ACCEL_RANGE_12G: u8 = 0x02;
pub const BMI088_ACCEL_RANGE_24G: u8 = 0x03;

/* Accel Output data rate */
pub const BMI08X_ACCEL_ODR_12_5_HZ: u8 = 0x05;
pub const BMI08X_ACCEL_ODR_25_HZ: u8 = 0x06;
pub const BMI08X_ACCEL_ODR_50_HZ: u8 = 0x07;
pub const BMI08X_ACCEL_ODR_100_HZ: u8 = 0x08;
pub const BMI08X_ACCEL_ODR_200_HZ: u8 = 0x09;
pub const BMI08X_ACCEL_ODR_400_HZ: u8 = 0x0A;
pub const BMI08X_ACCEL_ODR_800_HZ: u8 = 0x0B;
pub const BMI08X_ACCEL_ODR_1600_HZ: u8 = 0x0C;

/* Accel Self test */
pub const BMI08X_ACCEL_SWITCH_OFF_SELF_TEST: u8 = 0x00;
pub const BMI08X_ACCEL_POSITIVE_SELF_TEST: u8 = 0x0D;
pub const BMI08X_ACCEL_NEGATIVE_SELF_TEST: u8 = 0x09;

/* Accel Power mode */
pub const BMI08X_ACCEL_PM_ACTIVE: u8 = 0x00;
pub const BMI08X_ACCEL_PM_SUSPEND: u8 = 0x03;

/* Accel Power control settings */
pub const BMI08X_ACCEL_POWER_DISABLE: u8 = 0x00;
pub const BMI08X_ACCEL_POWER_ENABLE: u8 = 0x04;

/* Accel internal interrupt pin mapping */
pub const BMI08X_ACCEL_INTA_DISABLE: u8 = 0x00;
pub const BMI08X_ACCEL_INTA_ENABLE: u8 = 0x01;
pub const BMI08X_ACCEL_INTB_DISABLE: u8 = 0x00;
pub const BMI08X_ACCEL_INTB_ENABLE: u8 = 0x02;
pub const BMI08X_ACCEL_INTC_DISABLE: u8 = 0x00;
pub const BMI08X_ACCEL_INTC_ENABLE: u8 = 0x04;

/* Accel Soft reset delay */
pub const BMI08X_ACCEL_SOFTRESET_DELAY_MS: u8 = 1;

/* Mask definitions for ACCEL_ERR_REG register */
pub const BMI08X_FATAL_ERR_MASK: u8 = 0x01;
pub const BMI08X_ERR_CODE_MASK: u8 = 0x1C;

/* Position definitions for ACCEL_ERR_REG register */
pub const BMI08X_CMD_ERR_POS: u8 = 1;
pub const BMI08X_ERR_CODE_POS: u8 = 2;

/* Mask definition for ACCEL_STATUS_REG register */
pub const BMI08X_ACCEL_STATUS_MASK: u8 = 0x80;

/* Position definitions for ACCEL_STATUS_REG  */
pub const BMI08X_ACCEL_STATUS_POS: u8 = 7;

/* Mask definitions for odr, bandwidth and range */
pub const BMI08X_ACCEL_ODR_MASK: u8 = 0x0F;
pub const BMI08X_ACCEL_BW_MASK: u8 = 0xF0;
pub const BMI08X_ACCEL_RANGE_MASK: u8 = 0x03;

/* Position definitions for odr, bandwidth and range */
pub const BMI08X_ACCEL_BW_POS: u8 = 4;

/* Mask definitions for INT1_IO_CONF register */
pub const BMI08X_ACCEL_INT_EDGE_MASK: u8 = 0x01;
pub const BMI08X_ACCEL_INT_LVL_MASK: u8 = 0x02;
pub const BMI08X_ACCEL_INT_OD_MASK: u8 = 0x04;
pub const BMI08X_ACCEL_INT_IO_MASK: u8 = 0x08;
pub const BMI08X_ACCEL_INT_IN_MASK: u8 = 0x10;

/* Position definitions for INT1_IO_CONF register */
pub const BMI08X_ACCEL_INT_EDGE_POS: u8 = 0;
pub const BMI08X_ACCEL_INT_LVL_POS: u8 = 1;
pub const BMI08X_ACCEL_INT_OD_POS: u8 = 2;
pub const BMI08X_ACCEL_INT_IO_POS: u8 = 3;
pub const BMI08X_ACCEL_INT_IN_POS: u8 = 4;

/* Mask definitions for INT1/INT2 mapping register */
pub const BMI08X_ACCEL_MAP_INTA_MASK: u8 = 0x01;

/* Mask definitions for INT1/INT2 mapping register */
pub const BMI08X_ACCEL_MAP_INTA_POS: u8 = 0x00;

/* Mask definitions for INT1_INT2_MAP_DATA register */
pub const BMI08X_ACCEL_INT1_DRDY_MASK: u8 = 0x04;
pub const BMI08X_ACCEL_INT2_DRDY_MASK: u8 = 0x40;

/* Position definitions for INT1_INT2_MAP_DATA register */
pub const BMI08X_ACCEL_INT1_DRDY_POS: u8 = 2;
pub const BMI08X_ACCEL_INT2_DRDY_POS: u8 = 6;

/* Asic Initialization value */
pub const BMI08X_ASIC_INITIALIZED: u8 = 0x01;

/*************************** BMI08 Gyroscope Macros *****************************/
/** Register map */
/* Gyro registers */

/* Gyro Chip Id register */
pub const BMI08X_REG_GYRO_CHIP_ID: u8 = 0x00;

/* Gyro X LSB data register */
pub const BMI08X_REG_GYRO_X_LSB: u8 = 0x02;

/* Gyro X MSB data register */
pub const BMI08X_REG_GYRO_X_MSB: u8 = 0x03;

/* Gyro Y LSB data register */
pub const BMI08X_REG_GYRO_Y_LSB: u8 = 0x04;

/* Gyro Y MSB data register */
pub const BMI08X_REG_GYRO_Y_MSB: u8 = 0x05;

/* Gyro Z LSB data register */
pub const BMI08X_REG_GYRO_Z_LSB: u8 = 0x06;

/* Gyro Z MSB data register */
pub const BMI08X_REG_GYRO_Z_MSB: u8 = 0x07;

/* Gyro Interrupt status register */
pub const BMI08X_REG_GYRO_INT_STAT_1: u8 = 0x0A;

/* Gyro FIFO status register */
pub const BMI08X_REG_GYRO_FIFO_STATUS: u8 = 0x0E;

/* Gyro Range register */
pub const BMI08X_REG_GYRO_RANGE: u8 = 0x0F;

/* Gyro Bandwidth register */
pub const BMI08X_REG_GYRO_BANDWIDTH: u8 = 0x10;

/* Gyro Power register */
pub const BMI08X_REG_GYRO_LPM1: u8 = 0x11;

/* Gyro Soft reset register */
pub const BMI08X_REG_GYRO_SOFTRESET: u8 = 0x14;

/* Gyro Interrupt control register */
pub const BMI08X_REG_GYRO_INT_CTRL: u8 = 0x15;

/* Gyro Interrupt Pin configuration register */
pub const BMI08X_REG_GYRO_INT3_INT4_IO_CONF: u8 = 0x16;

/* Gyro Interrupt Map register */
pub const BMI08X_REG_GYRO_INT3_INT4_IO_MAP: u8 = 0x18;

/* Gyro FIFO watermark enable register */
pub const BMI08X_REG_GYRO_FIFO_WM_ENABLE: u8 = 0x1E;

/* Gyro Self test register */
pub const BMI08X_REG_GYRO_SELF_TEST: u8 = 0x3C;

/* Gyro Fifo Config 0 register */
pub const BMI08X_REG_GYRO_FIFO_CONFIG0: u8 = 0x3D;

/* Gyro Fifo Config 1 register */
pub const BMI08X_REG_GYRO_FIFO_CONFIG1: u8 = 0x3E;

/* Gyro Fifo Data register */
pub const BMI08X_REG_GYRO_FIFO_DATA: u8 = 0x3F;

/* Gyro unique chip identifier */
pub const BMI08X_GYRO_CHIP_ID: u8 = 0x0F;

/* Gyro I2C slave address */
pub const BMI08X_GYRO_I2C_ADDR_PRIMARY: u8 = 0x68;
pub const BMI08X_GYRO_I2C_ADDR_SECONDARY: u8 = 0x69;

/* Gyro Range */
pub const BMI08X_GYRO_RANGE_2000_DPS: u8 = 0x00;
pub const BMI08X_GYRO_RANGE_1000_DPS: u8 = 0x01;
pub const BMI08X_GYRO_RANGE_500_DPS: u8 = 0x02;
pub const BMI08X_GYRO_RANGE_250_DPS: u8 = 0x03;
pub const BMI08X_GYRO_RANGE_125_DPS: u8 = 0x04;

/* Gyro Output data rate and bandwidth */
pub const BMI08X_GYRO_BW_532_ODR_2000_HZ: u8 = 0x00;
pub const BMI08X_GYRO_BW_230_ODR_2000_HZ: u8 = 0x01;
pub const BMI08X_GYRO_BW_116_ODR_1000_HZ: u8 = 0x02;
pub const BMI08X_GYRO_BW_47_ODR_400_HZ: u8 = 0x03;
pub const BMI08X_GYRO_BW_23_ODR_200_HZ: u8 = 0x04;
pub const BMI08X_GYRO_BW_12_ODR_100_HZ: u8 = 0x05;
pub const BMI08X_GYRO_BW_64_ODR_200_HZ: u8 = 0x06;
pub const BMI08X_GYRO_BW_32_ODR_100_HZ: u8 = 0x07;
pub const BMI08X_GYRO_ODR_RESET_VAL: u8 = 0x80;

/* Gyro Power mode */
pub const BMI08X_GYRO_PM_NORMAL: u8 = 0x00;
pub const BMI08X_GYRO_PM_DEEP_SUSPEND: u8 = 0x20;
pub const BMI08X_GYRO_PM_SUSPEND: u8 = 0x80;

/* Gyro data ready interrupt enable value */
pub const BMI08X_GYRO_DRDY_INT_DISABLE_VAL: u8 = 0x00;
pub const BMI08X_GYRO_DRDY_INT_ENABLE_VAL: u8 = 0x80;
pub const BMI08X_GYRO_FIFO_INT_DISABLE_VAL: u8 = 0x00;
pub const BMI08X_GYRO_FIFO_INT_ENABLE_VAL: u8 = 0x40;
pub const BMI08X_GYRO_FIFO_WM_ENABLE_VAL: u8 = 0x80;
pub const BMI08X_GYRO_FIFO_WM_DISABLE_VAL: u8 = 0x00;

/* Gyro data ready map values */
pub const BMI08X_GYRO_MAP_DRDY_TO_INT3: u8 = 0x01;
pub const BMI08X_GYRO_MAP_DRDY_TO_INT4: u8 = 0x80;
pub const BMI08X_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4: u8 = 0x81;
pub const BMI08X_GYRO_MAP_FIFO_INT3: u8 = 0x04;
pub const BMI08X_GYRO_MAP_FIFO_INT4: u8 = 0x20;
pub const BMI08X_GYRO_MAP_FIFO_BOTH_INT3_INT4: u8 = 0x24;

/* Gyro Soft reset delay */
pub const BMI08X_GYRO_SOFTRESET_DELAY: u8 = 30;

/* Gyro power mode config delay */
pub const BMI08X_GYRO_POWER_MODE_CONFIG_DELAY: u8 = 30;

/** Mask definitions for range, bandwidth and power */
pub const BMI08X_GYRO_RANGE_MASK: u8 = 0x07;
pub const BMI08X_GYRO_BW_MASK: u8 = 0x0F;
pub const BMI08X_GYRO_POWER_MASK: u8 = 0xA0;

/** Position definitions for range, bandwidth and power */
pub const BMI08X_GYRO_POWER_POS: u8 = 5;

/* Mask definitions for BMI08X_GYRO_INT_CTRL_REG register */
pub const BMI08X_GYRO_DATA_EN_MASK: u8 = 0x80;

/* Position definitions for BMI08X_GYRO_INT_CTRL_REG register */
pub const BMI08X_GYRO_DATA_EN_POS: u8 = 7;

/* Mask definitions for BMI08X_GYRO_INT3_INT4_IO_CONF_REG register */
pub const BMI08X_GYRO_INT3_LVL_MASK: u8 = 0x01;
pub const BMI08X_GYRO_INT3_OD_MASK: u8 = 0x02;
pub const BMI08X_GYRO_INT4_LVL_MASK: u8 = 0x04;
pub const BMI08X_GYRO_INT4_OD_MASK: u8 = 0x08;

/* Position definitions for BMI08X_GYRO_INT3_INT4_IO_CONF_REG register */
pub const BMI08X_GYRO_INT3_OD_POS: u8 = 1;
pub const BMI08X_GYRO_INT4_LVL_POS: u8 = 2;
pub const BMI08X_GYRO_INT4_OD_POS: u8 = 3;

/* Mask definitions for BMI08X_GYRO_INT_EN_REG register */
pub const BMI08X_GYRO_INT_EN_MASK: u8 = 0x80;

/* Position definitions for BMI08X_GYRO_INT_EN_REG register */
pub const BMI08X_GYRO_INT_EN_POS: u8 = 7;

/* Mask definitions for BMI088_GYRO_INT_MAP_REG register */
pub const BMI08X_GYRO_INT3_MAP_MASK: u8 = 0x01;
pub const BMI08X_GYRO_INT4_MAP_MASK: u8 = 0x80;

/* Position definitions for BMI088_GYRO_INT_MAP_REG register */
pub const BMI08X_GYRO_INT3_MAP_POS: u8 = 0;
pub const BMI08X_GYRO_INT4_MAP_POS: u8 = 7;

/* Mask definitions for BMI088_GYRO_INT_MAP_REG register */
pub const BMI088_GYRO_INT3_MAP_MASK: u8 = 0x01;
pub const BMI088_GYRO_INT4_MAP_MASK: u8 = 0x80;

/* Position definitions for BMI088_GYRO_INT_MAP_REG register */
pub const BMI088_GYRO_INT3_MAP_POS: u8 = 0;
pub const BMI088_GYRO_INT4_MAP_POS: u8 = 7;

/* Mask definitions for GYRO_SELF_TEST register */
pub const BMI08X_GYRO_SELF_TEST_EN_MASK: u8 = 0x01;
pub const BMI08X_GYRO_SELF_TEST_RDY_MASK: u8 = 0x02;
pub const BMI08X_GYRO_SELF_TEST_RESULT_MASK: u8 = 0x04;
pub const BMI08X_GYRO_SELF_TEST_FUNCTION_MASK: u8 = 0x08;

/* Position definitions for GYRO_SELF_TEST register */
pub const BMI08X_GYRO_SELF_TEST_RDY_POS: u8 = 1;
pub const BMI08X_GYRO_SELF_TEST_RESULT_POS: u8 = 2;
pub const BMI08X_GYRO_SELF_TEST_FUNCTION_POS: u8 = 3;

/* Gyro Fifo configurations */
pub const BMI08X_GYRO_FIFO_OVERRUN_MASK: u8 = 0x80;
pub const BMI08X_GYRO_FIFO_OVERRUN_POS: u8 = 0x07;
pub const BMI08X_GYRO_FIFO_MODE_MASK: u8 = 0xC0;
pub const BMI08X_GYRO_FIFO_MODE_POS: u8 = 0x06;
pub const BMI08X_GYRO_FIFO_TAG_MASK: u8 = 0x80;
pub const BMI08X_GYRO_FIFO_TAG_POS: u8 = 0x07;
pub const BMI08X_GYRO_FIFO_DATA_SELECT_MASK: u8 = 0x03;
pub const BMI08X_GYRO_FIFO_FRAME_COUNT_MASK: u8 = 0x7F;
pub const BMI08X_GYRO_FIFO_WM_LEVEL_MASK: u8 = 0x7F;

/* Gyro Fifo interrupt map */
pub const BMI08X_GYRO_FIFO_INT3_MASK: u8 = 0x04;
pub const BMI08X_GYRO_FIFO_INT3_POS: u8 = 0x02;
pub const BMI08X_GYRO_FIFO_INT4_MASK: u8 = 0x20;
pub const BMI08X_GYRO_FIFO_INT4_POS: u8 = 0x05;

/* Gyro FIFO definitions */
pub const BMI08X_GYRO_FIFO_TAG_ENABLED: u8 = 0x01;
pub const BMI08X_GYRO_FIFO_TAG_DISABLED: u8 = 0x00;
pub const BMI08X_GYRO_FIFO_MODE_BYPASS: u8 = 0x00;
pub const BMI08X_GYRO_FIFO_MODE: u8 = 0x01;
pub const BMI08X_GYRO_FIFO_MODE_STREAM: u8 = 0x02;
pub const BMI08X_GYRO_FIFO_XYZ_AXIS_ENABLED: u8 = 0x00;
pub const BMI08X_GYRO_FIFO_X_AXIS_ENABLED: u8 = 0x01;
pub const BMI08X_GYRO_FIFO_Y_AXIS_ENABLED: u8 = 0x02;
pub const BMI08X_GYRO_FIFO_Z_AXIS_ENABLED: u8 = 0x03;
pub const BMI08X_GYRO_FIFO_XYZ_AXIS_FRAME_SIZE: u8 = 0x06;
pub const BMI08X_GYRO_FIFO_SINGLE_AXIS_FRAME_SIZE: u8 = 0x02;
pub const BMI08X_GYRO_FIFO_1KB_BUFFER: u16 = 1024;

/*************************** Common Macros for both Accel and Gyro *****************************/
/* SPI read/write mask to configure address */
pub const BMI08X_SPI_RD_MASK: u8 = 0x80;
pub const BMI08X_SPI_WR_MASK: u8 = 0x7F;

/* API success code */
pub const BMI08X_OK: i8 = 0;

/* API error codes */
pub const BMI08X_E_NULL_PTR: i8 = -1;
pub const BMI08X_E_COM_FAIL: i8 = -2;
pub const BMI08X_E_DEV_NOT_FOUND: i8 = -3;
pub const BMI08X_E_OUT_OF_RANGE: i8 = -4;
pub const BMI08X_E_INVALID_INPUT: i8 = -5;
pub const BMI08X_E_CONFIG_STREAM_ERROR: i8 = -6;
pub const BMI08X_E_RD_WR_LENGTH_INVALID: i8 = -7;
pub const BMI08X_E_INVALID_CONFIG: i8 = -8;
pub const BMI08X_E_FEATURE_NOT_SUPPORTED: i8 = -9;

/* API warning codes */
pub const BMI08X_W_SELF_TEST_FAIL: i8 = 1;

/* Soft reset Value */
pub const BMI08X_SOFT_RESET_CMD: u8 = 0xB6;

/* Enable/disable macros */
pub const BMI08X_DISABLE: u8 = 0;
pub const BMI08X_ENABLE: u8 = 1;

/*  To define warnings for FIFO activity */
pub const BMI08X_W_FIFO_EMPTY: i8 = 1;
pub const BMI08X_W_PARTIAL_READ: i8 = 2;

/*  Maximum length to read */
pub const BMI08X_MAX_LEN: u8 = 128;

/*  Sensortime resolution in seconds */
pub const BMI08X_SENSORTIME_RESOLUTION: f32 = 0.0000390625;

/*  Constant values macros */
pub const BMI08X_SENSOR_DATA_SYNC_TIME_MS: u8 = 1;
pub const BMI08X_DELAY_BETWEEN_WRITES_MS: u8 = 1;
pub const BMI08X_SELF_TEST_DELAY_MS: u8 = 3;
pub const BMI08X_POWER_CONFIG_DELAY: u8 = 5;
pub const BMI08X_SENSOR_SETTLE_TIME_MS: u8 = 30;
pub const BMI08X_SELF_TEST_DATA_READ_MS: u8 = 50;
pub const BMI08X_ASIC_INIT_TIME_MS: u8 = 150;

pub const BMI08X_CONFIG_STREAM_SIZE: u16 = 6144;

/* Sensor time array parameter definitions */
pub const BMI08X_SENSOR_TIME_MSB_BYTE: u8 = 2;
pub const BMI08X_SENSOR_TIME_XLSB_BYTE: u8 = 1;
pub const BMI08X_SENSOR_TIME_LSB_BYTE: u8 = 0;

/*   int pin active state */
pub const BMI08X_INT_ACTIVE_LOW: u8 = 0;
pub const BMI08X_INT_ACTIVE_HIGH: u8 = 1;

/*   interrupt pin output definition  */
pub const BMI08X_INT_MODE_PUSH_PULL: u8 = 0;
pub const BMI08X_INT_MODE_OPEN_DRAIN: u8 = 1;

/* Sensor bit resolution */
pub const BMI08X_16_BIT_RESOLUTION: u8 = 16;

/*********************************BMI08X FIFO Macros**********************************/
/** Register map */
/* @name FIFO Header Mask definitions */
pub const BMI08X_FIFO_HEADER_ACC_FRM: u8 = 0x84;
pub const BMI08X_FIFO_HEADER_ALL_FRM: u8 = 0x9C;
pub const BMI08X_FIFO_HEADER_SENS_TIME_FRM: u8 = 0x44;
pub const BMI08X_FIFO_HEADER_SKIP_FRM: u8 = 0x40;
pub const BMI08X_FIFO_HEADER_INPUT_CFG_FRM: u8 = 0x48;
pub const BMI08X_FIFO_HEAD_OVER_READ_MSB: u8 = 0x80;
pub const BMI08X_FIFO_SAMPLE_DROP_FRM: u8 = 0x50;

/* Accel registers */
pub const BMI08X_FIFO_LENGTH_0_ADDR: u8 = 0x24;
pub const BMI08X_FIFO_LENGTH_1_ADDR: u8 = 0x25;
pub const BMI08X_FIFO_DATA_ADDR: u8 = 0x26;
pub const BMI08X_FIFO_DOWNS_ADDR: u8 = 0x45;
pub const BMI08X_FIFO_WTM_0_ADDR: u8 = 0x46;
pub const BMI08X_FIFO_WTM_1_ADDR: u8 = 0x47;
pub const BMI08X_FIFO_CONFIG_0_ADDR: u8 = 0x48;
pub const BMI08X_FIFO_CONFIG_1_ADDR: u8 = 0x49;

/* @name FIFO sensor data lengths */
pub const BMI08X_FIFO_ACCEL_LENGTH: u8 = 6;
pub const BMI08X_FIFO_WTM_LENGTH: u8 = 2;
pub const BMI08X_FIFO_LENGTH_MSB_BYTE: u8 = 1;
pub const BMI08X_FIFO_DATA_LENGTH: u8 = 2;
pub const BMI08X_FIFO_CONFIG_LENGTH: u8 = 2;
pub const BMI08X_SENSOR_TIME_LENGTH: u8 = 3;
pub const BMI08X_FIFO_SKIP_FRM_LENGTH: u8 = 1;
pub const BMI08X_FIFO_INPUT_CFG_LENGTH: u8 = 1;

/* @name FIFO byte counter mask definition */
pub const BMI08X_FIFO_BYTE_COUNTER_MSB_MASK: u8 = 0x3F;

/* @name FIFO frame masks */
pub const BMI08X_FIFO_LSB_CONFIG_CHECK: u8 = 0x00;
pub const BMI08X_FIFO_MSB_CONFIG_CHECK: u8 = 0x80;
pub const BMI08X_FIFO_INTR_MASK: u8 = 0x5C;

/*name FIFO config modes */
pub const BMI08X_ACC_STREAM_MODE: u8 = 0x00;
pub const BMI08X_ACC_FIFO_MODE: u8 = 0x01;

/*name Mask definitions for FIFO configuration modes */
pub const BMI08X_ACC_FIFO_MODE_CONFIG_MASK: u8 = 0x01;

/* @name Mask definitions for FIFO_CONFIG_1 register */
pub const BMI08X_ACCEL_EN_MASK: u8 = 0x40;
pub const BMI08X_ACCEL_INT1_EN_MASK: u8 = 0x08;
pub const BMI08X_ACCEL_INT2_EN_MASK: u8 = 0x04;

/*name Position definitions for FIFO_CONFIG_1 register */
pub const BMI08X_ACCEL_EN_POS: u8 = 6;
pub const BMI08X_ACCEL_INT1_EN_POS: u8 = 3;
pub const BMI08X_ACCEL_INT2_EN_POS: u8 = 2;

/* @name Position definitions for FIFO_DOWNS register */
pub const BMI08X_ACC_FIFO_DOWNS_MASK: u8 = 0xF0;

/* @name FIFO down sampling bit positions */
pub const BMI08X_ACC_FIFO_DOWNS_POS: u8 = 0x04;

/* @name FIFO down sampling user macros */
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_0: u8 = 0;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_1: u8 = 1;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_2: u8 = 2;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_3: u8 = 3;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_4: u8 = 4;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_5: u8 = 5;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_6: u8 = 6;
pub const BMI08X_ACC_FIFO_DOWN_SAMPLE_7: u8 = 7;

/* @name Mask definitions for INT1_INT2_MAP_DATA register */
pub const BMI08X_ACCEL_INT2_FWM_MASK: u8 = 0x20;
pub const BMI08X_ACCEL_INT2_FFULL_MASK: u8 = 0x10;
pub const BMI08X_ACCEL_INT1_FWM_MASK: u8 = 0x02;
pub const BMI08X_ACCEL_INT1_FFULL_MASK: u8 = 0x01;

/* @name Positions definitions for INT1_INT2_MAP_DATA register */
pub const BMI08X_ACCEL_INT1_FWM_POS: u8 = 1;
pub const BMI08X_ACCEL_INT2_FFULL_POS: u8 = 4;
pub const BMI08X_ACCEL_INT2_FWM_POS: u8 = 5;

/* Utility Macros  */
pub const BMI08X_SET_LOW_BYTE: u16 = 0x00FF;
pub const BMI08X_SET_HIGH_BYTE: u16 = 0xFF00;
pub const BMI08X_SET_LOW_NIBBLE: u8 = 0x0F;

fn BMI08X_SET_BITS_POS_0(reg_var: u8, bit_mask: u8, val: u8) -> u8 {
    (reg_var & !bit_mask) | (val & bit_mask)
}

fn BMI08X_SET_BITS(reg_var: u8, bit_pose: u8, bit_mask: u8, val: u8) -> u8 {
    (reg_var & !bit_mask) | ((val << bit_pose) & bit_mask)
}

fn BMI08X_MS_TO_US(ms: u8) -> u32 {
    ms as u32 * 1000
}

type bmi08x_read_fptr_t = fn(u8, &mut [u8], u32) -> i8;
type bmi08x_write_fptr_t = fn(u8, &[u8], u32) -> i8;
type bmi08x_delay_ms_fptr_t = fn(u32);

/*
 * @brief Interface selection enums
 */
#[derive(PartialEq)]
pub enum bmi08x_intf {
    /* I2C interface */
    BMI08X_I2C_INTF,
    /* SPI interface */
    BMI08X_SPI_INTF,
}

/* Enum to define BMA4 variants */
#[derive(PartialEq)]
pub enum bmi08x_variant {
    BMI085_VARIANT = 0,
    BMI088_VARIANT = 1,
}

/*
 *  @brief Enum to select accelerometer Interrupt pins
 */
#[derive(PartialEq)]
pub enum bmi08x_accel_int_channel {
    /* interrupt Channel 1 for accel sensor */
    BMI08X_INT_CHANNEL_1,
    /* interrupt Channel 2 for accel sensor */
    BMI08X_INT_CHANNEL_2,
}

/*
 *  @brief Enum to select gyroscope Interrupt pins
 */
#[derive(PartialEq)]
pub enum bmi08x_gyro_int_channel {
    /* interrupt Channel 3 for gyro sensor */
    BMI08X_INT_CHANNEL_3,
    /* interrupt Channel 4 for gyro sensor */
    BMI08X_INT_CHANNEL_4,
}

/*
 *  @brief Enum to select accelerometer interrupts
 */
#[derive(PartialEq)]
pub enum bmi08x_accel_int_types {
    BMI08X_ACCEL_INT_DATA_RDY,
    /* Accel data ready interrupt */
    BMI08X_ACCEL_INT_SYNC_DATA_RDY,
    /* Accel synchronized data ready interrupt */
    BMI08X_ACCEL_SYNC_INPUT,
    /* Accel FIFO watermark interrupt */
    BMI08X_ACCEL_INT_FIFO_WM,
    /* Accel FIFO full interrupt */
    BMI08X_ACCEL_INT_FIFO_FULL,
}

/*
 *  @brief Enum to select gyroscope interrupts
 */
#[derive(PartialEq)]
pub enum bmi08x_gyro_int_types {
    /* Gyro data ready interrupt */
    BMI08X_GYRO_INT_DATA_RDY,
    /* Gyro FIFO watermark interrupt */
    BMI08X_GYRO_INT_FIFO_WM,
    /* Gyro FIFO full interrupt */
    BMI08X_GYRO_INT_FIFO_FULL,
}

/*
 *  @brief Sensor configuration structure
 */
pub struct bmi08x_cfg {
    /* power mode */
    pub power: u8,
    /* range */
    pub range: u8,
    /* bandwidth */
    pub bw: u8,
    /* output data rate */
    pub odr: u8,
}

/*
 *  @brief Interrupt channel structure for gyro
 */
pub struct bmi08x_gyro_int_channel_cfg {
    /* Gyro Interrupt channel */
    pub int_channel: bmi08x_gyro_int_channel,

    /* Select Gyro Interrupt type */
    pub int_type: bmi08x_gyro_int_types,

    /* Structure to configure gyro interrupt pins */
    pub int_pin_cfg: bmi08x_int_pin_cfg,
}

/*
 *  @brief Interrupt pin configuration structure
 */
pub struct bmi08x_int_pin_cfg {
    /* interrupt pin level configuration
     * Assignable macros :
     * - BMI08X_INT_ACTIVE_LOW
     * - BMI08X_INT_ACTIVE_HIGH
     */
    pub lvl: u8,

    /* interrupt pin mode configuration
     * Assignable macros :
     * - BMI08X_INT_MODE_PUSH_PULL
     * - BMI08X_INT_MODE_OPEN_DRAIN
     */
    pub output_mode: u8,

    /* Enable interrupt pin
     * Assignable Macros :
     * - BMI08X_ENABLE
     * - BMI08X_DISABLE
     */
    pub enable_int_pin: u8,
}

/*
 *  @brief Sensor XYZ data structure
 */
#[derive(Default, Copy, Clone)]
pub struct bmi08x_sensor_data {
    /* X-axis sensor data */
    pub x: i16,

    /* Y-axis sensor data */
    pub y: i16,

    /* Z-axis sensor data */
    pub z: i16,
}

pub struct bmi08x_dev {
    /* accel chip id */
    pub accel_chip_id: u8,
    /* gyro chip id */
    pub gyro_chip_id: u8,
    /* interface function pointer used to enable the device address for I2C and chip selection for SPI */
    pub intf_ptr_accel: u32,
    /* interface function pointer used to enable the device address for I2C and chip selection for SPI */
    pub intf_ptr_gyro: u32,
    /* Interface Selection
     * For SPI, interface = BMI08X_SPI_INTF
     * For I2C, interface = BMI08X_I2C_INTF
     **/
    pub intf: bmi08x_intf,
    /* Define the BMI08X variant BMI085 or BMI088 */
    pub variant: bmi08x_variant,
    /* decide SPI or I2C read mechanism */
    pub dummy: u8,
    pub accel_cfg: bmi08x_cfg,
    pub gyro_cfg: bmi08x_cfg,
    /* maximum read/write length (maximum supported length is 32) */
    pub read_write_len: u8,
    /* result of read/write */
    pub intf_rslt: i8,
}

impl bmi08x_dev {
    pub fn bmi08x_init(&mut self) -> i8 {
        self.bmi08a_init()
            | self.bmi08g_init()
            | self.bmi08a_set_power_mode()
            | self.bmi08a_set_meas_conf()
            | self.bmi08g_set_power_mode()
            | self.bmi08g_set_meas_conf()
    }

    fn bmi08a_init(&mut self) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut chip_id = [0_u8; 1];
        if self.intf == bmi08x_intf::BMI08X_SPI_INTF {
            self.dummy = BMI08X_ENABLE;
            rslt = get_regs(
                BMI08X_REG_ACCEL_CHIP_ID,
                &mut chip_id,
                self,
                self.intf_ptr_accel,
            );
        } else {
            self.dummy = BMI08X_DISABLE;
        }

        if rslt == BMI08X_OK {
            rslt = get_regs(
                BMI08X_REG_ACCEL_CHIP_ID,
                &mut chip_id,
                self,
                self.intf_ptr_accel,
            );

            if rslt == BMI08X_OK {
                if self.variant == bmi08x_variant::BMI085_VARIANT
                    && chip_id[0] == BMI085_ACCEL_CHIP_ID
                    || self.variant == bmi08x_variant::BMI088_VARIANT
                        && chip_id[0] == BMI088_ACCEL_CHIP_ID
                {
                    self.accel_chip_id = chip_id[0];
                } else {
                    rslt = BMI08X_E_DEV_NOT_FOUND;
                }
            }
        }
        return rslt;
    }

    fn bmi08g_init(&mut self) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut chip_id = [0_u8; 1];
        rslt = get_regs(
            BMI08X_REG_GYRO_CHIP_ID,
            &mut chip_id,
            self,
            self.intf_ptr_gyro,
        );
        if rslt == BMI08X_OK {
            if chip_id[0] == BMI08X_GYRO_CHIP_ID {
                self.gyro_chip_id = chip_id[0];
            } else {
                rslt = BMI08X_E_DEV_NOT_FOUND;
            }
        }
        return rslt;
    }

    fn bmi08a_set_power_mode(&mut self) -> i8 {
        let power_mode = self.accel_cfg.power;
        let enable: u8;
        match power_mode {
            BMI08X_ACCEL_PM_ACTIVE => {
                enable = BMI08X_ACCEL_POWER_ENABLE;
            }
            BMI08X_ACCEL_PM_SUSPEND => {
                enable = BMI08X_ACCEL_POWER_DISABLE;
            }
            _ => {
                return BMI08X_E_INVALID_INPUT;
            }
        }
        let mut rslt = set_regs(
            BMI08X_REG_ACCEL_PWR_CONF,
            &[power_mode],
            self,
            self.intf_ptr_accel,
        );
        if rslt == BMI08X_OK {
            hopter::time::sleep_ms(BMI08X_POWER_CONFIG_DELAY.into());
            rslt = set_regs(
                BMI08X_REG_ACCEL_PWR_CTRL,
                &[enable],
                self,
                self.intf_ptr_accel,
            );
            if rslt == BMI08X_OK {
                hopter::time::sleep_ms(BMI08X_POWER_CONFIG_DELAY.into());
            }
        }
        return rslt;
    }

    fn bmi08a_set_meas_conf(&mut self) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut data = [0_u8; 2];
        let odr = self.accel_cfg.odr;
        let bw = self.accel_cfg.bw;
        let range = self.accel_cfg.range;
        let mut is_odr_invalid = false;
        let mut is_bw_invalid = false;
        let mut is_range_invalid = false;
        if odr < BMI08X_ACCEL_ODR_12_5_HZ || odr > BMI08X_ACCEL_ODR_1600_HZ {
            is_odr_invalid = true;
        }

        if bw > BMI08X_ACCEL_BW_NORMAL {
            is_bw_invalid = true;
        }

        if self.variant == bmi08x_variant::BMI085_VARIANT && range > BMI085_ACCEL_RANGE_16G
            || self.variant == bmi08x_variant::BMI088_VARIANT && range > BMI088_ACCEL_RANGE_24G
        {
            is_range_invalid = true;
        }

        if !is_odr_invalid && !is_bw_invalid && !is_range_invalid {
            rslt = get_regs(BMI08X_REG_ACCEL_CONF, &mut data, self, self.intf_ptr_accel);
            hopter::time::sleep_ms(1);

            if rslt == BMI08X_OK {
                // TODO:
                data[0] = BMI08X_SET_BITS_POS_0(data[0], BMI08X_ACCEL_ODR_MASK, odr);
                data[0] = BMI08X_SET_BITS(data[0], BMI08X_ACCEL_BW_POS, BMI08X_ACCEL_BW_MASK, bw);
                data[1] = BMI08X_SET_BITS_POS_0(data[1], BMI08X_ACCEL_RANGE_MASK, range);
                rslt = set_regs(BMI08X_REG_ACCEL_CONF, &data, self, self.intf_ptr_accel);
            }
        } else {
            rslt = BMI08X_E_INVALID_CONFIG;
        }
        return rslt;
    }

    fn bmi08g_set_power_mode(&mut self) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut data = [0_u8; 1];
        let power_mode = self.gyro_cfg.power;

        rslt = get_regs(BMI08X_REG_GYRO_LPM1, &mut data, self, self.intf_ptr_gyro);

        if rslt == BMI08X_OK {
            if power_mode == BMI08X_GYRO_PM_SUSPEND && data[0] == BMI08X_GYRO_PM_DEEP_SUSPEND
                || power_mode == BMI08X_GYRO_PM_DEEP_SUSPEND && data[0] == BMI08X_GYRO_PM_SUSPEND
            {
                rslt = BMI08X_E_INVALID_INPUT;
            } else {
                rslt = set_regs(
                    BMI08X_REG_GYRO_LPM1,
                    &[power_mode],
                    self,
                    self.intf_ptr_gyro,
                );
                if rslt == BMI08X_OK {
                    hopter::time::sleep_ms(BMI08X_GYRO_POWER_MODE_CONFIG_DELAY.into());
                }
            }
        }
        return rslt;
    }

    fn bmi08g_set_meas_conf(&mut self) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut data = [0_u8; 1];
        let odr = self.gyro_cfg.odr;
        let range = self.gyro_cfg.range;
        let mut is_odr_invalid = false;
        let mut is_range_invalid = false;

        if odr > BMI08X_GYRO_BW_32_ODR_100_HZ {
            is_odr_invalid = true;
        }

        if range > BMI08X_GYRO_RANGE_125_DPS {
            is_range_invalid = true;
        }

        if !is_odr_invalid && !is_range_invalid {
            /* Read range value from the range register */
            rslt = get_regs(
                BMI08X_REG_GYRO_BANDWIDTH,
                &mut data,
                self,
                self.intf_ptr_gyro,
            );
            if rslt == BMI08X_OK {
                data[0] = BMI08X_SET_BITS_POS_0(data[0], BMI08X_GYRO_BW_MASK, odr);
                /* Write odr value to odr register */
                rslt = set_regs(BMI08X_REG_GYRO_BANDWIDTH, &data, self, self.intf_ptr_gyro);
                if rslt == BMI08X_OK {
                    rslt = get_regs(BMI08X_REG_GYRO_RANGE, &mut data, self, self.intf_ptr_gyro);
                    if rslt == BMI08X_OK {
                        data[0] = BMI08X_SET_BITS_POS_0(data[0], BMI08X_GYRO_RANGE_MASK, range);
                        rslt = set_regs(BMI08X_REG_GYRO_RANGE, &data, self, self.intf_ptr_gyro);
                    }
                }
            }
        } else {
            rslt = BMI08X_E_INVALID_CONFIG;
        }
        return rslt;
    }

    pub fn bmi08g_set_int_config(&mut self, int_config: bmi08x_gyro_int_channel_cfg) -> i8 {
        let mut rslt = BMI08X_OK;
        match int_config.int_type {
            bmi08x_gyro_int_types::BMI08X_GYRO_INT_DATA_RDY => {
                rslt = self.set_gyro_data_ready_int(int_config);
            }
            bmi08x_gyro_int_types::BMI08X_GYRO_INT_FIFO_WM
            | bmi08x_gyro_int_types::BMI08X_GYRO_INT_FIFO_FULL => {
                // rslt =
            }
        }
        return rslt;
    }

    fn set_gyro_data_ready_int(&mut self, int_config: bmi08x_gyro_int_channel_cfg) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut data = [0_u8; 1];
        let mut enable = 0_u8;
        rslt = get_regs(
            BMI08X_REG_GYRO_INT3_INT4_IO_MAP,
            &mut data,
            self,
            self.intf_ptr_gyro,
        );

        if rslt == BMI08X_OK {
            let conf = int_config.int_pin_cfg.enable_int_pin;
            match int_config.int_channel {
                bmi08x_gyro_int_channel::BMI08X_INT_CHANNEL_3 => {
                    data[0] = BMI08X_SET_BITS_POS_0(data[0], BMI08X_GYRO_INT3_MAP_MASK, conf);
                }
                bmi08x_gyro_int_channel::BMI08X_INT_CHANNEL_4 => {
                    data[0] = BMI08X_SET_BITS(
                        data[0],
                        BMI08X_GYRO_INT4_MAP_POS,
                        BMI08X_GYRO_INT4_MAP_MASK,
                        conf,
                    );
                }
            }

            if data[0] & BMI08X_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4 > 0 {
                enable = BMI08X_GYRO_DRDY_INT_ENABLE_VAL;
            } else {
                enable = BMI08X_GYRO_DRDY_INT_DISABLE_VAL;
            }

            rslt = set_regs(
                BMI08X_REG_GYRO_INT3_INT4_IO_MAP,
                &data,
                self,
                self.intf_ptr_gyro,
            );
            if rslt == BMI08X_OK {
                rslt = self.set_int_pin_config(int_config);
                if rslt == BMI08X_OK {
                    rslt = set_regs(
                        BMI08X_REG_GYRO_INT_CTRL,
                        &[enable],
                        self,
                        self.intf_ptr_accel,
                    );
                }
            }
        }
        return rslt;
    }

    fn set_int_pin_config(&mut self, int_config: bmi08x_gyro_int_channel_cfg) -> i8 {
        let mut rslt = BMI08X_OK;
        let mut data = [0_u8; 1];
        rslt = get_regs(
            BMI08X_REG_GYRO_INT3_INT4_IO_CONF,
            &mut data,
            self,
            self.intf_ptr_gyro,
        );
        if rslt == BMI08X_OK {
            match int_config.int_channel {
                bmi08x_gyro_int_channel::BMI08X_INT_CHANNEL_3 => {
                    data[0] = BMI08X_SET_BITS_POS_0(
                        data[0],
                        BMI08X_GYRO_INT3_LVL_MASK,
                        int_config.int_pin_cfg.lvl,
                    );
                    data[0] = BMI08X_SET_BITS(
                        data[0],
                        BMI08X_GYRO_INT3_OD_POS,
                        BMI08X_GYRO_INT3_OD_MASK,
                        int_config.int_pin_cfg.output_mode,
                    );
                }
                bmi08x_gyro_int_channel::BMI08X_INT_CHANNEL_4 => {
                    data[0] = BMI08X_SET_BITS_POS_0(
                        data[0],
                        BMI08X_GYRO_INT4_LVL_MASK,
                        int_config.int_pin_cfg.lvl,
                    );
                    data[0] = BMI08X_SET_BITS(
                        data[0],
                        BMI08X_GYRO_INT4_OD_POS,
                        BMI08X_GYRO_INT4_OD_MASK,
                        int_config.int_pin_cfg.output_mode,
                    );
                }
            }
            rslt = set_regs(
                BMI08X_REG_GYRO_INT3_INT4_IO_CONF,
                &data,
                self,
                self.intf_ptr_gyro,
            );
        }
        return rslt;
    }

    pub fn bmi08a_get_data(&mut self, accel_data: &mut bmi08x_sensor_data) -> i8 {
        let mut data = [0_u8; 6];
        let rslt = get_regs(BMI08X_REG_ACCEL_X_LSB, &mut data, self, self.intf_ptr_accel);
        if rslt == BMI08X_OK {
            accel_data.x = ((data[1] as i16) << 8) | data[0] as i16;
            accel_data.y = ((data[3] as i16) << 8) | data[2] as i16;
            accel_data.z = ((data[5] as i16) << 8) | data[4] as i16;
        }
        return rslt;
    }

    pub fn bmi08g_get_data(&mut self, gyro_data: &mut bmi08x_sensor_data) -> i8 {
        let mut data = [0_u8; 6];
        let rslt = get_regs(BMI08X_REG_GYRO_X_LSB, &mut data, self, self.intf_ptr_gyro);
        if rslt == BMI08X_OK {
            gyro_data.x = ((data[1] as i16) << 8) | data[0] as i16;
            gyro_data.y = ((data[3] as i16) << 8) | data[2] as i16;
            gyro_data.z = ((data[5] as i16) << 8) | data[4] as i16;
        }
        return rslt;
    }
}

pub fn get_regs(reg_addr: u8, reg_data: &mut [u8], dev: &mut bmi08x_dev, intf: u32) -> i8 {
    let mut rslt = BMI08X_OK;
    let mut reg_addr = reg_addr;
    let len = reg_data.len();
    let mut temp_buff = vec![0 as u8; len + dev.dummy as usize];

    if dev.intf == bmi08x_intf::BMI08X_SPI_INTF {
        reg_addr = reg_addr | BMI08X_SPI_RD_MASK;
    }

    dev.intf_rslt = crate::module::accel_gyro::i2c3_read(reg_addr, &mut temp_buff, intf);

    if dev.intf_rslt == BMI08X_INTF_RET_SUCCESS {
        for index in 0..len {
            reg_data[index] = temp_buff[index + dev.dummy as usize];
        }
    } else {
        rslt = BMI08X_E_COM_FAIL;
    }

    return rslt;
}

pub fn set_regs(reg_addr: u8, reg_data: &[u8], dev: &mut bmi08x_dev, intf: u32) -> i8 {
    let mut rslt = BMI08X_OK;
    let mut reg_addr = reg_addr;
    if dev.intf == bmi08x_intf::BMI08X_SPI_INTF {
        reg_addr = reg_addr & BMI08X_SPI_WR_MASK;
    }

    dev.intf_rslt = crate::module::accel_gyro::i2c3_write(reg_addr, reg_data, intf);
    if dev.intf_rslt != BMI08X_INTF_RET_SUCCESS {
        rslt = BMI08X_E_COM_FAIL;
    }
    return rslt;
}
