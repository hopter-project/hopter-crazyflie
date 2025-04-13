/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved

This file is part of VL53L1 and is dual licensed,
either GPL-2.0+
or 'BSD 3-clause "New" or "Revised" License' , at your option.
******************************************************************************
*/
#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::vl53l1_ll_def::*;

/**
 * @file vl53l1_def.h
 *
 * @brief Type definitions for VL53L1 API.
 *
 */

/** @defgroup VL53L1_globaldefine_group VL53L1 Defines
 *  @brief    VL53L1 Defines
*/

/** VL53L1 IMPLEMENTATION major version */
pub const VL53L1_IMPLEMENTATION_VER_MAJOR: u8 = 2;
/** VL53L1 IMPLEMENTATION minor version */
pub const VL53L1_IMPLEMENTATION_VER_MINOR: u8 = 4;
/** VL53L1 IMPLEMENTATION sub version */
pub const VL53L1_IMPLEMENTATION_VER_SUB: u8 = 5;
/** VL53L1 IMPLEMENTATION sub version */
pub const VL53L1_IMPLEMENTATION_VER_REVISION: u32 = 2548;

/****************************************
 * PRIVATE define do not edit
****************************************/

/** @brief Defines the parameters of the Get Version Functions
 */
#[derive(Default)]
pub struct VL53L1_Version_t {
    pub revision: u32,
    /* revision number */
    pub major: u8,
    /* major number */
    pub minor: u8,
    /* minor number */
    pub build: u8,
    /* build number */
}

pub const VL53L1_DEVINFO_STRLEN: usize = 32;

/** @brief Defines the parameters of the Get Device Info Functions
 */
#[derive(Default)]
pub struct VL53L1_DeviceInfo_t {
    pub Name: [char; VL53L1_DEVINFO_STRLEN],
    /* Full Name of the Device e.g. VL53L1 cut1.1 */
    pub Type: [char; VL53L1_DEVINFO_STRLEN],
    /* Type of the Device e.g VL53L1 */
    pub ProductId: [char; VL53L1_DEVINFO_STRLEN],
    /* Product Identifier String
     * @warning Not yet implemented
     */
    pub ProductType: u8,
    /* Product Type, VL53L1 = 0xCC, VL53L3 = 0xAA
     * Stands as module_type in the datasheet
     */
    pub ProductRevisionMajor: u8,
    /* Product revision major */
    pub ProductRevisionMinor: u8,
    /* Product revision minor */
}

/** @defgroup VL53L1_define_PresetModes_group Defines Preset modes
 *  Defines all possible preset modes for the device
*/
pub type VL53L1_PresetModes = u8;

pub const VL53L1_PRESETMODE_AUTONOMOUS: VL53L1_PresetModes = 3;
pub const VL53L1_PRESETMODE_LITE_RANGING: VL53L1_PresetModes = 4;
pub const VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS: VL53L1_PresetModes = 8;
/* ... Modes to be added depending on device */

/** @defgroup VL53L1_define_DistanceModes_group Defines Distance modes
 *  Defines all possible Distance modes for the device
*/
pub type VL53L1_DistanceModes = u8;

pub const VL53L1_DISTANCEMODE_SHORT: VL53L1_DistanceModes = 1;
pub const VL53L1_DISTANCEMODE_MEDIUM: VL53L1_DistanceModes = 2;
pub const VL53L1_DISTANCEMODE_LONG: VL53L1_DistanceModes = 3;

/** @defgroup VL53L1_define_XtalkCal_group Defines Xtalk Calibration modes
 *  Defines all possible Offset Calibration modes for the device
*/
pub type VL53L1_XtalkCalibrationModes = u8;

pub const VL53L1_XTALKCALIBRATIONMODE_NO_TARGET: VL53L1_OffsetCalibrationModes = 0;
/* To perform Xtalk calibration with no target below 80 cm */
pub const VL53L1_XTALKCALIBRATIONMODE_SINGLE_TARGET: VL53L1_OffsetCalibrationModes = 1;
/* To perform Xtalk calibration with one target */
pub const VL53L1_XTALKCALIBRATIONMODE_FULL_ROI: VL53L1_OffsetCalibrationModes = 2;
/* To perform Xtalk calibration based on histogram with full ROI */

/** @defgroup VL53L1_define_OffsetCal_group Defines Offset Calibration modes
 *  Defines all possible Offset Calibration modes for the device
*/
pub type VL53L1_OffsetCalibrationModes = u8;

pub const VL53L1_OFFSETCALIBRATIONMODE_STANDARD: VL53L1_OffsetCalibrationModes = 1;
pub const VL53L1_OFFSETCALIBRATIONMODE_PRERANGE_ONLY: VL53L1_OffsetCalibrationModes = 2;

/** @defgroup VL53L1_CheckEnable_group Check Enable list
 *  @brief Check Enable code
*
*  Define used to specify the LimitCheckId.
*  Use @a VL53L1_GetLimitCheckInfo() to get the string.
*/

pub const VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE: u16 = 0;
pub const VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE: u16 = 1;
pub const VL53L1_CHECKENABLE_NUMBER_OF_CHECKS: usize = 2;

/** @defgroup VL53L1_ThresholdMode_gropup Detection Functionality
 *  @brief Defines the different functionalities for the detection feature
*/
pub type VL53L1_ThresholdMode = u8;

pub const VL53L1_THRESHOLD_CROSSED_LOW: VL53L1_ThresholdMode = 0;
/* Trigger interrupt if value < thresh_low */
pub const VL53L1_THRESHOLD_CROSSED_HIGH: VL53L1_ThresholdMode = 1;
/* Trigger interrupt if value > thresh_high */
pub const VL53L1_THRESHOLD_OUT_OF_WINDOW: VL53L1_ThresholdMode = 2;
/* Trigger interrupt if value < thresh_low OR value > thresh_high */
pub const VL53L1_THRESHOLD_IN_WINDOW: VL53L1_ThresholdMode = 3;
/* Trigger interrupt if value > thresh_low AND value < thresh_high */

/** @brief Defines parameters for Distance detection Thresholds configuration
 */
#[derive(Default)]
pub struct VL53L1_DistanceThreshold_t {
    pub CrossMode: VL53L1_ThresholdMode,
    pub High: u16,
    /* Distance threshold high limit in mm */
    pub Low: u16,
    /* Distance threshold low limit  in mm */
}

/** @brief Defines parameters for Signal rate detection Thresholds configuration
 */
#[derive(Default)]
pub struct VL53L1_RateThreshold_t {
    pub CrossMode: VL53L1_ThresholdMode,
    pub High: u32,
    /* Signal rate threshold high limit */
    pub Low: u32,
    /* Signal rate threshold low limit */
}

/** @defgroup VL53L1_DetectionMode_group Gpio Functionality
 *  @brief Defines conditions leading to device's IT on GPIO
*/
pub type VL53L1_DetectionMode = u8;

pub const VL53L1_DETECTION_NORMAL_RUN: VL53L1_DetectionMode = 0;
/* Trigger interrupt on new measurement regardless of threshold
 * just like after a VL53L1_SetPresetMode() call
 */
pub const VL53L1_DETECTION_DISTANCE_ONLY: VL53L1_DetectionMode = 1;
/* Trigger interrupt if "threshold event" occurs on distance */
pub const VL53L1_DETECTION_RATE_ONLY: VL53L1_DetectionMode = 2;
/* Trigger interrupt if "threshold event" occurs on signal rate */
pub const VL53L1_DETECTION_DISTANCE_AND_RATE: VL53L1_DetectionMode = 3;
/* Trigger interrupt if "threshold event" occurs on distance AND rate
 */
pub const VL53L1_DETECTION_DISTANCE_OR_RATE: VL53L1_DetectionMode = 4;
/* Trigger interrupt if "threshold event" occurs on distance OR rate
 */

/** @brief Defines parameters for User/object Detection configuration
 */
#[derive(Default)]
pub struct VL53L1_DetectionConfig_t {
    pub DetectionMode: VL53L1_DetectionMode,
    /* See #VL53L1_DetectionMode*/
    pub IntrNoTarget: u8,
    /* 1 to trigger IT in case of no target found */
    pub Distance: VL53L1_DistanceThreshold_t,
    /* limits in mm */
    pub Rate: VL53L1_RateThreshold_t,
    /* limits in FixPoint1616_t */
}

/** @brief Defines all parameters for the device
 */
#[derive(Default)]
pub struct VL53L1_DeviceParameters_t {
    pub PresetMode: VL53L1_PresetModes,
    /* Defines the operating mode to be used for the next measure */
    pub DistanceMode: VL53L1_DistanceModes,
    /* Defines the operating mode to be used for the next measure */
    pub MeasurementTimingBudgetMicroSeconds: u32,
    /* Defines the allowed total time for a single measurement */
    pub LimitChecksEnable: [u8; VL53L1_CHECKENABLE_NUMBER_OF_CHECKS],
    /* This Array store all the Limit Check enable for this device. */
    pub LimitChecksStatus: [u8; VL53L1_CHECKENABLE_NUMBER_OF_CHECKS],
    /* This Array stores all the Status of the check linked to last
     * measurement.
     */
    pub LimitChecksValue: [u32; VL53L1_CHECKENABLE_NUMBER_OF_CHECKS],
    /* This Array stores all the Limit Check value for this device */
    pub LimitChecksCurrent: [u32; VL53L1_CHECKENABLE_NUMBER_OF_CHECKS],
    /* This Array stores all the Limit Check current value from latest
     * ranging
     */
}

/** @defgroup VL53L1_define_State_group Defines the current status of the device
 *  Defines the current status of the device
*/

pub type VL53L1_State = u8;

pub const VL53L1_STATE_POWERDOWN: VL53L1_State = 0;
/* Device is in HW reset  */
pub const VL53L1_STATE_WAIT_STATICINIT: VL53L1_State = 1;
/* Device is initialized and wait for static initialization  */
pub const VL53L1_STATE_STANDBY: VL53L1_State = 2;
/* Device is in Low power Standby mode   */
pub const VL53L1_STATE_IDLE: VL53L1_State = 3;
/* Device has been initialized and ready to do measurements  */
pub const VL53L1_STATE_RUNNING: VL53L1_State = 4;
/* Device is performing measurement */
pub const VL53L1_STATE_RESET: VL53L1_State = 5;
/* Soft reset has been run on Device */
pub const VL53L1_STATE_UNKNOWN: VL53L1_State = 98;
/* Device is in unknown state and need to be rebooted  */
pub const VL53L1_STATE_ERROR: VL53L1_State = 99;
/* Device is in error state and need to be rebooted  */

/**
 * @struct VL53L1_RangingMeasurementData_t
* @brief Single Range measurement data.
*/
#[derive(Default)]
pub struct VL53L1_RangingMeasurementData_t {
    pub TimeStamp: u32,
    /* 32-bit time stamp.
     * @warning Not yet implemented
     */
    pub StreamCount: u8,
    /* 8-bit Stream Count. */
    pub RangeQualityLevel: u8,
    /* indicate a quality level in percentage from 0 to 100
     * @warning Not yet implemented
     */
    pub SignalRateRtnMegaCps: u32,
    /* Return signal rate (MCPS)\n these is a 16.16 fix point
     *  value, which is effectively a measure of target
     *  reflectance.
     */
    pub AmbientRateRtnMegaCps: u32,
    /* Return ambient rate (MCPS)\n these is a 16.16 fix point
     *  value, which is effectively a measure of the ambient
     *  light.
     */
    pub EffectiveSpadRtnCount: u16,
    /* Return the effective SPAD count for the return signal.
     *  To obtain Real value it should be divided by 256
     */
    pub SigmaMilliMeter: u32,
    /* Return the Sigma value in millimeter */
    pub RangeMilliMeter: i16,
    /* range distance in millimeter. This should be between
     *  RangeMinMilliMeter and RangeMaxMilliMeter
     */
    pub RangeFractionalPart: u8,
    /* Fractional part of range distance. Final value is a
     *  RangeMilliMeter + RangeFractionalPart/256.
     *  @warning Not yet implemented
     */
    pub RangeStatus: u8,
    /* Range Status for the current measurement. This is device
     *  dependent. Value = 0 means value is valid.
     */
}

/** @brief Defines User Zone(ROI) parameters
 *
*/
#[derive(Default)]
pub struct VL53L1_UserRoi_t {
    pub TopLeftX: u8,  /* Top Left x coordinate:  0-15 range */
    pub TopLeftY: u8,  /* Top Left y coordinate:  0-15 range */
    pub BotRightX: u8, /* Bot Right x coordinate: 0-15 range */
    pub BotRightY: u8, /* Bot Right y coordinate: 0-15 range */
}

/** @brief Defines ROI configuration parameters
 *
*  Support up a max of 16 zones, Each Zone has the same size
*
*/

/**
 * @struct VL53L1_CustomerNvmManaged_t
*
*/

#[derive(Default)]
pub struct VL53L1_CustomerNvmManaged_t {
    pub global_config__spad_enables_ref_0: u8,
    pub global_config__spad_enables_ref_1: u8,
    pub global_config__spad_enables_ref_2: u8,
    pub global_config__spad_enables_ref_3: u8,
    pub global_config__spad_enables_ref_4: u8,
    pub global_config__spad_enables_ref_5: u8,
    pub global_config__ref_en_start_select: u8,
    pub ref_spad_man__num_requested_ref_spads: u8,
    pub ref_spad_man__ref_location: u8,
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    pub ref_spad_char__total_rate_target_mcps: u16,
    pub algo__part_to_part_range_offset_mm: i16,
    pub mm_config__inner_offset_mm: i16,
    pub mm_config__outer_offset_mm: i16,
}

/**
 * @struct  VL53L1_CalibrationData_t
* @brief   Structure for storing the Calibration Data
*
*/

#[derive(Default)]
pub struct VL53L1_CalibrationData_t {
    pub struct_version: u32,
    pub customer: VL53L1_CustomerNvmManaged_t,
    pub add_off_cal_data: VL53L1_additional_offset_cal_data_t,
    pub optical_centre: VL53L1_optical_centre_t,
    pub gain_cal: VL53L1_gain_calibration_data_t,
    pub cal_peak_rate_map: VL53L1_cal_peak_rate_map_t,
}

pub const VL53L1_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION: u32 = 0x20;
/** VL53L1 additional Calibration Data struct version final struct version
 * is given by adding it to  VL53L1_LL_CALIBRATION_DATA_STRUCT_VERSION
*/

pub const VL53L1_CALIBRATION_DATA_STRUCT_VERSION: u32 =
    VL53L1_LL_CALIBRATION_DATA_STRUCT_VERSION + VL53L1_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;
/* VL53L1 Calibration Data struct version */

/**
 * @struct  VL53L1_AdditionalData_t
* @brief   Structure for storing the Additional Data
*
*/
pub type VL53L1_AdditionalData_t = VL53L1_additional_data_t;

pub type VL53L1_SequenceStepId = u8;
pub const VL53L1_SEQUENCESTEP_VHV: VL53L1_SequenceStepId = 0;
/*VHV. */
pub const VL53L1_SEQUENCESTEP_PHASECAL: VL53L1_SequenceStepId = 1;
/*Phase Calibration. */
pub const VL53L1_SEQUENCESTEP_REFPHASE: VL53L1_SequenceStepId = 2;
/*Reference Phase. */
pub const VL53L1_SEQUENCESTEP_DSS1: VL53L1_SequenceStepId = 3;
/*DSS1. */
pub const VL53L1_SEQUENCESTEP_DSS2: VL53L1_SequenceStepId = 4;
/*DSS2. */
pub const VL53L1_SEQUENCESTEP_MM1: VL53L1_SequenceStepId = 5;
/*Mode Mitigation 1. */
pub const VL53L1_SEQUENCESTEP_MM2: VL53L1_SequenceStepId = 6;
/*Mode Mitigation 2. */
pub const VL53L1_SEQUENCESTEP_RANGE: VL53L1_SequenceStepId = 7;
/*Final Range step. */

pub const VL53L1_SEQUENCESTEP_NUMBER_OF_ITEMS: u8 = 8;
/* =umber o; Sequence Step Managed by the API. */

/** @defgroup VL53L1_define_RangeStatus_group Defines the Range Status
 */
pub const VL53L1_RANGESTATUS_RANGE_VALID: u8 = 0;
/*The Range is valid. */
pub const VL53L1_RANGESTATUS_SIGMA_FAIL: u8 = 1;
/*Sigma Fail. */
pub const VL53L1_RANGESTATUS_SIGNAL_FAIL: u8 = 2;
/*Signal fail. */
pub const VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED: u8 = 3;
/*Target is below minimum detection threshold. */
pub const VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL: u8 = 4;
/*Phase out of valid limits -  different to a wrap exit. */
pub const VL53L1_RANGESTATUS_HARDWARE_FAIL: u8 = 5;
/*Hardware fail. */
pub const VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL: u8 = 6;
/*The Range is valid but the wraparound check has not been done. */
pub const VL53L1_RANGESTATUS_WRAP_TARGET_FAIL: u8 = 7;
/*Wrapped target - no matching phase in other VCSEL period timing. */
pub const VL53L1_RANGESTATUS_PROCESSING_FAIL: u8 = 8;
/*Internal algo underflow or overflow in lite ranging. */
pub const VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL: u8 = 9;
/*Specific to lite ranging. */
pub const VL53L1_RANGESTATUS_SYNCRONISATION_INT: u8 = 10;
/*1st interrupt when starting ranging in back to back mode. Ignore data. */
pub const VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE: u8 = 11;
/*All Range ok but object is result of multiple pulses merging together.
 * Used by RQL for merged pulse detection
*/
pub const VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL: u8 = 12;
/*Used  by RQL  as different to phase fail. */
pub const VL53L1_RANGESTATUS_MIN_RANGE_FAIL: u8 = 13;
/*User ROI input is not valid e.g. beyond SPAD Array.*/
pub const VL53L1_RANGESTATUS_RANGE_INVALID: u8 = 14;
/*lld returned valid range but negative value ! */
pub const VL53L1_RANGESTATUS_NONE: u8 = 255;
/*No Update. */

/** @brief  Contains the Internal data of the Bare Driver
 */
#[derive(Default)]
pub struct VL53L1_DevData_t {
    pub LLData: VL53L1_LLDriverData_t,
    /* Low Level Driver data structure */
    pub llresults: VL53L1_LLDriverResults_t,
    /* Low Level Driver data structure */
    pub PalState: VL53L1_State,
    /* Store the pal state */
    pub CurrentParameters: VL53L1_DeviceParameters_t,
    /* Current Device Parameter */
}

/* MACRO Definitions */
/** @defgroup VL53L1_define_GeneralMacro_group General Macro Defines
 *  General Macro Defines
*/

/* Defines */
// #define VL53L1_SETPARAMETERFIELD(Dev, field, value) \
//     (VL53L1DevDataSet(Dev, CurrentParameters.field, value))

// #define VL53L1_GETPARAMETERFIELD(Dev, field, variable) \
//     (variable = VL53L1DevDataGet(Dev, CurrentParameters).field)

// #define VL53L1_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
//     (VL53L1DevDataSet(Dev, CurrentParameters.field[index], value))

// #define VL53L1_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
//     (variable = VL53L1DevDataGet(Dev, CurrentParameters).field[index])

// #define VL53L1_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
//     (VL53L1DevDataSet(Dev, DeviceSpecificParameters.field, value))

// #define VL53L1_GETDEVICESPECIFICPARAMETER(Dev, field) \
//     (VL53L1DevDataGet(Dev, DeviceSpecificParameters).field)

pub fn VL53L1_FIXPOINT1616TOFIXPOINT44(Value: u32) -> u16 {
    ((Value >> 12) & 0xFFFF) as u16
}
pub fn VL53L1_FIXPOINT44TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 12
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT72(Value: u32) -> u16 {
    ((Value >> 14) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT72TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 14
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT97(Value: u32) -> u16 {
    ((Value >> 9) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT97TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 9
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT88(Value: u32) -> u16 {
    ((Value >> 8) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT88TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 8
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT412(Value: u32) -> u16 {
    ((Value >> 4) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT412TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 4
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT313(Value: u32) -> u16 {
    ((Value >> 3) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT313TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 3
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT08(Value: u32) -> u8 {
    ((Value >> 8) & 0x00FF) as u8
}

pub fn VL53L1_FIXPOINT08TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 8
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT53(Value: u32) -> u8 {
    ((Value >> 13) & 0x00FF) as u8
}

pub fn VL53L1_FIXPOINT53TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 13
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT102(Value: u32) -> u16 {
    ((Value >> 14) & 0x0FFF) as u16
}

pub fn VL53L1_FIXPOINT102TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 14
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT142(Value: u32) -> u16 {
    ((Value >> 14) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT142TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 14
}

pub fn VL53L1_FIXPOINT1616TOFIXPOINT160(Value: u32) -> u16 {
    ((Value >> 16) & 0xFFFF) as u16
}

pub fn VL53L1_FIXPOINT160TOFIXPOINT1616(Value: u16) -> u32 {
    (Value as u32) << 16
}

pub fn VL53L1_MAKEUINT16(lsb: u8, msb: u8) -> u16 {
    ((msb as u16) << 8) + lsb as u16
}
