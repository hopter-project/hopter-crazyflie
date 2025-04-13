/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved

This file is part of VL53L1 and is dual licensed,
either GPL-2.0+
or 'BSD 3-clause "New" or "Revised" License' , at your option.
******************************************************************************
*/

/* indexes for the bare driver tuning setting API function */
#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
pub enum VL53L1_Tuning_t {
    VL53L1_TUNING_VERSION = 0,
    VL53L1_TUNING_PROXY_MIN,
    VL53L1_TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
    VL53L1_TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
    VL53L1_TUNING_MIN_AMBIENT_DMAX_VALID,
    VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
    VL53L1_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
    VL53L1_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
    VL53L1_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
    VL53L1_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
    VL53L1_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR,
    VL53L1_TUNING_PHASECAL_PATCH_POWER,
    VL53L1_TUNING_MAX_TUNABLE_KEY,
}

/* default values for the tuning settings parameters */
pub const TUNING_VERSION: i32 = 0x0007;

pub const TUNING_PROXY_MIN: i32 = 30; /* min distance in mm */
pub const TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM: i32 = 600;
/* Target distance in mm for single target Xtalk */
pub const TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER: i32 = 50;
/* Number of sample used for single target Xtalk */
pub const TUNING_MIN_AMBIENT_DMAX_VALID: i32 = 8;
/* Minimum ambient level to state the Dmax returned by the device is valid */
pub const TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER: i32 = 50;
/* Maximum loops to perform simple offset calibration */
pub const TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM: i32 = 600;
/* Target distance in mm for target Xtalk from Bins method*/
pub const TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT: i32 = 1;
/* Number of loops done during the simple offset calibration*/
pub const TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT: i32 = 9;
/* zero distance offset calibration non linear compensation default value */

/* The following settings are related to the fix for ticket EwokP #558410 */
pub const TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN: i32 = 24;
/* Acceptance margin for the xtalk_shape bin_data sum computation */
pub const TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET: i32 = 50;
/* Recovery value for Xtalk compensation plane offset in kcps */
/* 50 stands for ~0.10 kcps cover glass in 7.9 format */
/* End of settings related to the fix for ticket EwokP #558410 */
pub const TUNING_PHASECAL_PATCH_POWER: i32 = 0;
/* PhaseCal duration patch tuning param
* 0 default duration 15ms, 1 leads to 60ms, 2 for 240ms and 3 for 3580ms
*/
