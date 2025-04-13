/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/
#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::vl53l1_error_codes::VL53L1_Error;
use super::vl53l1_ll_device::*;
use super::vl53l1_register_structs::*;

/*
 * @file vl53l1_ll_def.h
 *
 * @brief Type definitions for VL53L1 LL Driver.
 *
 */

/* @defgroup VL53L1_globalLLDriverDefine_group VL53L1 Defines
 *  @brief    VL53L1 LL Driver Defines
*/

/** VL53L1 Low Level Driver IMPLEMENTATION major version */
pub const VL53L1_LL_API_IMPLEMENTATION_VER_MAJOR: u8 = 1;
/** VL53L1 Low Level DriverI IMPLEMENTATION minor version */
pub const VL53L1_LL_API_IMPLEMENTATION_VER_MINOR: u8 = 2;
/** VL53L1 Low Level DriverI IMPLEMENTATION sub version */
pub const VL53L1_LL_API_IMPLEMENTATION_VER_SUB: u8 = 13;
/** VL53L1 Low Level Driver IMPLEMENTATION sub version */
pub const VL53L1_LL_API_IMPLEMENTATION_VER_REVISION: u32 = 2190;

pub const VL53L1_LL_API_IMPLEMENTATION_VER_STRING: &str = "1.2.13.2190";

/** VL53L1_FIRMWARE min and max compatible revisions */
pub const VL53L1_FIRMWARE_VER_MINIMUM: u32 = 398;
pub const VL53L1_FIRMWARE_VER_MAXIMUM: u32 = 400;

/****************************************
 * PRIVATE define do not edit
****************************************/

pub const VL53L1_LL_CALIBRATION_DATA_STRUCT_VERSION: u32 = 0xECAB0102;
/** VL53L1 Calibration Data struct version */

/* Start Patch_ZoneCalDataStructVersion_11854 */

pub const VL53L1_LL_ZONE_CALIBRATION_DATA_STRUCT_VERSION: u32 = 0xECAE0101;
/** VL53L1 Zone Calibration Data struct version */

/* End Patch_ZoneCalDataStructVersion_11854 */

pub const VL53L1_MAX_OFFSET_RANGE_RESULTS: usize = 3;
/* Sets the maximum number of offset range results
 required for the offset calibration.
Order is RANGE, MM1, MM2  */

pub const VL53L1_NVM_MAX_FMT_RANGE_DATA: usize = 4;
/* The number of FMT range data points stored in NVM */

pub const VL53L1_NVM_PEAK_RATE_MAP_SAMPLES: usize = 25;
/* The number of samples in the NVM peak rate signal map */
pub const VL53L1_NVM_PEAK_RATE_MAP_WIDTH: usize = 5;
/* Array width of NVM peak rate signal map */
pub const VL53L1_NVM_PEAK_RATE_MAP_HEIGHT: usize = 5;
/* Array height the NVM peak rate signal map */

/** @defgroup VL53L1_defineExtraError_group Error and Warning code returned by API
 *  The following DEFINE are used to identify the PAL ERROR
*/

pub const VL53L1_ERROR_DEVICE_FIRMWARE_TOO_OLD: VL53L1_Error = -80;
/* Device Firmware too old .. */
pub const VL53L1_ERROR_DEVICE_FIRMWARE_TOO_NEW: VL53L1_Error = -85;
/* Device Firmware too new .. */
pub const VL53L1_ERROR_UNIT_TEST_FAIL: VL53L1_Error = -90;
/* Unit Test Fail */
pub const VL53L1_ERROR_FILE_READ_FAIL: VL53L1_Error = -95;
/* File Read  Fail */
pub const VL53L1_ERROR_FILE_WRITE_FAIL: VL53L1_Error = -96;
/* File Write Fail */
/* Tells requested functionality has not been implemented yet or
 * not compatible with the device */

/** @brief Defines the parameters of the LL driver Get Version Functions
 */
#[derive(Default)]
pub struct VL53L1_ll_version_t {
    pub ll_revision: u32, /* revision number */
    pub ll_major: u8,     /* major number */
    pub ll_minor: u8,     /* minor number */
    pub ll_build: u8,     /* build number */
}

/** @brief Reference SPAD Characterization (RefSpadChar) Config
 */

#[derive(Default)]
pub struct VL53L1_refspadchar_config_t {
    pub device_test_mode: u8, /* Device test mode */
    pub vcsel_period: u8,     /* VCSEL period (register) value */
    pub timeout_us: u32,      /* timeout in [us] */
    pub target_count_rate_mcps: u16,
    /* Target reference total count rate in [Mcps] - 9.7 format */
    pub min_count_rate_limit_mcps: u16,
    /* Min valid reference rate [Mcps] - 9.7 format */
    pub max_count_rate_limit_mcps: u16,
    /* Max valid reference rate [Mcps] - 9.7 format */
}

/** @brief SPAD Self Check (SSC) Config data structure
 */

#[derive(Default)]
pub struct VL53L1_ssc_config_t {
    pub array_select: VL53L1_DeviceSscArray,
    /* SPAD Array select
     * 0 - store RTN array count rates \n
     * 1 - store REF array count rates */
    pub vcsel_period: u8,
    /* VCSEL period (register) value */
    pub vcsel_start: u8,
    /* VCSEL start register value */
    pub vcsel_width: u8,
    /* VCSEL ssc_timeout_us width register value e.g. 2 */
    pub timeout_us: u32,
    /* requested Ranging Timeout in [us] e.g 100000us */
    pub rate_limit_mcps: u16,
    /* Rate limit for checks either 1.15 or
     *    9.7 dependent on test_mode
     */
}

/** @brief Xtalk Extraction and Paramter Config
 */
#[derive(Default)]
pub struct VL53L1_xtalk_config_t {
    pub algo__crosstalk_compensation_plane_offset_kcps: u32,
    /* Private crosstalk_compensation_plane_offset_kcps (fixed point 9.9) */
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    /* Private crosstalk_compensation_x_plane_gradient_kcps (fixed point 5.11) */
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    /* Private crosstalk_compensation_y_plane_gradient_kcps (fixed point 5.11) */
    pub nvm_default__crosstalk_compensation_plane_offset_kcps: u32,
    /* NVm stored crosstalk_compensation_plane_offset_kcps (fixed point 9.9) */
    pub nvm_default__crosstalk_compensation_x_plane_gradient_kcps: i16,
    /* NVM stored crosstalk_compensation_x_plane_gradient_kcps (fixed point 5.11) */
    pub nvm_default__crosstalk_compensation_y_plane_gradient_kcps: i16,
    /* NVM stored crosstalk_compensation_y_plane_gradient_kcps (fixed point 5.11) */
    pub global_crosstalk_compensation_enable: u8,
    /* Enable switch for crosstalk compensation in all modes */
    pub lite_mode_crosstalk_margin_kcps: i16,
    /* Additional xtalk factor rate, added to plane_offset value in both
     * SD mode, applied as a seperate addition at point of
     * application to the device, plane_offset
     * value remains unaltered. (fixed point 7.9)
     */
    pub crosstalk_range_ignore_threshold_mult: u8,
    /* User set multiplier for range ignore threshold setting (fixed point 3.5) */
    pub crosstalk_range_ignore_threshold_rate_mcps: u16,
    /* Generated range ignore threshold rate in Mcps per spad (fixed
     * point 3.13)
     */
}

/** @brief TuningParameter Storage
 *
* - Storage structure for any LLD tuning parms
* which are dynamically altered by low level functions
* mostly when programming directly to the device
*
*- Added as part of Patch_AddingTuningParmStorage_11821
*/

#[derive(Default)]
pub struct VL53L1_tuning_parm_storage_t {
    pub tp_tuning_parm_version: u16,
    /* Programmed Global tuning version num for debug
     */
    pub tp_tuning_parm_key_table_version: u16,
    /* Key Table tuning structure \
     * version
     */
    pub tp_tuning_parm_lld_version: u16,
    /* Programmed LLD version to ensure matching tuning structure \
     * key table
     */
    pub tp_init_phase_rtn_lite_long: u8,
    /* initial phase value for rtn array \
     * in Lite Long Ranging Mode
     */
    pub tp_init_phase_rtn_lite_med: u8,
    /* initial phase value for rtn array \
     * in Lite Medium Ranging Mode
     */
    pub tp_init_phase_rtn_lite_short: u8,
    /* initial phase value for rtn array \
     * in Lite Short Ranging Mode
     */
    pub tp_init_phase_ref_lite_long: u8,
    /* initial phase value for ref array \
     * in Lite Long Ranging Mode
     */
    pub tp_init_phase_ref_lite_med: u8,
    /* initial phase value for ref array \
     * in Lite Medium Ranging Mode
     */
    pub tp_init_phase_ref_lite_short: u8,
    /* initial phase value for ref array \
     * in Lite short Ranging Mode
     */
    pub tp_consistency_lite_phase_tolerance: u8,
    /* Phase tolerance consistency value to be used \
     * in Lite modes
     */
    pub tp_phasecal_target: u8,
    /* Phasecal target value
     */
    pub tp_cal_repeat_rate: u16,
    /* Auto VHV/Calibration repeat rate for \
     * use in Lite mode
     */
    pub tp_lite_min_clip: u8,
    /* Min Clip value in mm applied to device in Lite \
     * modes
     */
    pub tp_lite_long_sigma_thresh_mm: u16,
    /* Sigma threshold limit for Lite Long mode  \
     * in 14.2 format mm
     */
    pub tp_lite_med_sigma_thresh_mm: u16,
    /* Sigma threshold limit for Lite Medium mode  \
     * in 14.2 format mm
     */
    pub tp_lite_short_sigma_thresh_mm: u16,
    /* Sigma threshold limit for Lite Short mode  \
     * in 14.2 format mm
     */
    pub tp_lite_long_min_count_rate_rtn_mcps: u16,
    /* Min count rate level used in lite long mode \
     * in 9.7 Mcps format
     */
    pub tp_lite_med_min_count_rate_rtn_mcps: u16,
    /* Min count rate level used in lite medium mode \
     * in 9.7 Mcps format
     */
    pub tp_lite_short_min_count_rate_rtn_mcps: u16,
    /* Min count rate level used in lite short mode \
     * in 9.7 Mcps format
     */
    pub tp_lite_sigma_est_pulse_width_ns: u8,
    /* Sigma thresholding tunign parm for Lite mode
     */
    pub tp_lite_sigma_est_amb_width_ns: u8,
    /* Sigma thresholding tunign parm for Lite mode
     */
    pub tp_lite_sigma_ref_mm: u8,
    /* Sigma thresholding tunign parm for Lite mode
     */
    pub tp_lite_seed_cfg: u8,
    /* Lite Mode Seed mode switch
     */
    pub tp_timed_seed_cfg: u8,
    /* Timed Mode Seed mode switch
     */
    pub tp_lite_quantifier: u8,
    /* Low level quantifier setting for lite modes
     */
    pub tp_lite_first_order_select: u8,
    /* Low level First order select setting for lite modes
     */
    pub tp_dss_target_lite_mcps: u16,
    /* DSS Target rate in 9.7 format Mcps for lite modes
     */
    pub tp_dss_target_timed_mcps: u16,
    /* DSS Target rate in 9.7 format Mcps for Timed modes
     */
    pub tp_phasecal_timeout_lite_us: u32,
    /* Phasecal timeout in us for lite modes
     */
    pub tp_phasecal_timeout_timed_us: u32,
    /* Phasecal timeout in us for Timed modes
     */
    pub tp_mm_timeout_lite_us: u32,
    /* MM stage timeout in us for Lite modes
     */
    pub tp_mm_timeout_timed_us: u32,
    /* MM stage timeout in us for Timed modes
     */
    pub tp_mm_timeout_lpa_us: u32,
    /* MM stage timeout in us for Low Power Auto modes
     */
    pub tp_range_timeout_lite_us: u32,
    /* Ranging stage timeout in us for Lite modes
     */
    pub tp_range_timeout_timed_us: u32,
    /* Ranging stage timeout in us for Timed modes
     */
    pub tp_range_timeout_lpa_us: u32,
    /* Ranging stage timeout in us for Low Power Auto modes
     */
}

/** @brief Optical Centre data
 *
*/

#[derive(Default)]
pub struct VL53L1_optical_centre_t {
    pub x_centre: u8, /* Optical x centre : 4.4 format */
    pub y_centre: u8, /* Optical y centre : 4.4 format */
}

/** @brief Defines User Zone(ROI) parameters
 *
*/

#[derive(Default)]
pub struct VL53L1_user_zone_t {
    pub x_centre: u8, /* Zone x centre :  0-15 range */
    pub y_centre: u8, /* Zone y centre :  0-15 range */
    pub width: u8,    /* Width  of Zone  0 = 1, 7 = 8, 15 = 16 */
    pub height: u8,   /* Height of Zone  0 = 1, 7 = 8, 15 = 16 */
}

/**
 * @struct VL53L1_GPIO_interrupt_config_t
*
* @brief Structure to configure conditions when GPIO interrupt is trigerred
*
*/

#[derive(Default)]
pub struct VL53L1_GPIO_interrupt_config_t {
    /* Distance interrupt mode */
    pub intr_mode_distance: VL53L1_GPIO_Interrupt_Mode,

    /* Rate interrupt mode */
    pub intr_mode_rate: VL53L1_GPIO_Interrupt_Mode,

    /* trigger interrupt if a new measurement is ready
     * 	__WARNING!__ will override other settings
     */
    pub intr_new_measure_ready: u8,

    /* Trigger interrupt if no target found */
    pub intr_no_target: u8,

    /* If set to 0, interrupts will only be triggered if BOTH rate AND
     * distance thresholds are triggered (combined mode). If set to 1,
     * interrupts will be triggered if EITHER rate OR distance thresholds
     * are triggered (independent mode). */
    pub intr_combined_mode: u8,

    /* -- thresholds -- */
    /* The struct holds a copy of the thresholds but they are written when
    * this structure is set using VL53L1_set_GPIO_interrupt_config/_struct
    * */

    /* Distance threshold high limit (mm) */
    pub threshold_distance_high: u16,

    /* Distance threshold low limit (mm) */
    pub threshold_distance_low: u16,

    /* Rate threshold high limit (9.7 Mcps) */
    pub threshold_rate_high: u16,

    /* Rate threshold low limit (9.7 Mcps) */
    pub threshold_rate_low: u16,
}

/* Start Patch_LowPowerAutoMode */
/**
 * @struct VL53L1_low_power_auto_data_t
*
* @brief Structure to hold state, tuning and output variables for the low
* power auto mode (Presence)
*
*/

#[derive(Default)]
pub struct VL53L1_low_power_auto_data_t {
    /* Tuning variable for the VHV loop bound setting in low power auto
     * mode. This is zero based, so the number of loops in VHV is this + 1.
     * Please note, the first range will run with default VHV settings.
     * Only lower 6 bits are allowed */
    pub vhv_loop_bound: u8,

    /* Indicates if we are or are not in low power auto mode */
    pub is_low_power_auto_mode: u8,

    /* Used to check if we're running the first range or not. Not to be
     * used as a stream count */
    pub low_power_auto_range_count: u8,

    /* saved interrupt config byte to restore */
    pub saved_interrupt_config: u8,

    /* saved vhv config init byte to restore */
    pub saved_vhv_init: u8,

    /* saved vhv config timeout byte to restore */
    pub saved_vhv_timeout: u8,

    /* phase cal resutl from the first range */
    pub first_run_phasecal_result: u8,

    /* DSS. Total rate per spad given from the current range */
    pub dss__total_rate_per_spad_mcps: u32,

    /* DSS. Calculated required SPADs value */
    pub dss__required_spads: u16,
}

/* End Patch_LowPowerAutoMode */

/**
 * @struct  VL53L1_range_data_t
* @brief   Internal data structure for storing post processed ranges
*
*/

#[derive(Default, Copy, Clone)]
pub struct VL53L1_range_data_t {
    /* Info size */
    pub range_id: u8,
    /* Range Result id e.g 0, 1, 2 */
    pub time_stamp: u32,
    /* 32-bit time stamp */
    pub width: u16,
    /* VCSEL pulse width in [PLL clocks] 6.4 format */
    pub woi: u8,
    /* WOI width in [PLL clocks] */
    pub fast_osc_frequency: u16,
    /* Oscillator frequency in 4.12 format */
    pub zero_distance_phase: u16,
    /* Zero Distance phase in  5.11 format */
    pub actual_effective_spads: u16,
    /* effective SPAD count in 8.8 format */
    pub total_periods_elapsed: u32,
    /* Elapsed time in macro periods for readout channel */
    pub peak_duration_us: u32,
    /* Peak VCSEL width time in us */
    pub woi_duration_us: u32,
    /* WOI duration time in us */


    /* Event counts */
    pub ambient_window_events: u32,
    /* Return event count for the ambient window   */
    pub ranging_total_events: u32,
    /* Return ranging event count for the ranging window.
    This includes both VCSEL and ambient contributions */
    pub signal_total_events: i32,
    /* Return event count for the ranging window with ambient
             subtracted, Note it is 32-bit signed register */

    /* Rates */
    pub peak_signal_count_rate_mcps: u16,
    /* Peak signal (VCSEL) Rate in 9.7 format */
    pub avg_signal_count_rate_mcps: u16,
    /* Average signal (VCSEL) Rate in 9.7 format */
    pub ambient_count_rate_mcps: u16,
    /* Ambient Rate in 9.7 format */
    pub total_rate_per_spad_mcps: u16,
    /* Total Rate Per SPAD in 3.13 format */
    pub peak_rate_per_spad_kcps: u32,
    /* Peak Rate Per SPAD in 13.11 format */

    /*  Sigma */
    pub sigma_mm: u16,
    /* Range sigma Estimate [mm]  9.7 format */

    /* Phase */
    pub median_phase: u16,
    /* Median Phase in 5.11 format */

    /* Range */
    pub median_range_mm: i16,
    /* Median Range in [mm] by default there are no fractional bits
             Optionally 1 or 2 fractional can be enabled via the
            VL53L1_SYSTEM__FRACTIONAL_ENABLE register */

    /* Range status */
    pub range_status: u8,
}

/**
 * @struct  VL53L1_range_results_t
* @brief   Structure for storing the set of range results
*
*/

#[derive(Default, Copy, Clone)]
pub struct VL53L1_range_results_t {
    pub cfg_device_state: VL53L1_DeviceState,
    /* Configuration Device State */
    pub rd_device_state: VL53L1_DeviceState,
    /* Read Device State */
    pub stream_count: u8,
    /* 8-bit stream count */
    pub device_status: u8,
    /*  Global device status for result set */
    pub data: [VL53L1_range_data_t; 2],
    /* Range data each target distance */
}

/**
 * @struct  VL53L1_offset_range_data_t
* @brief   Structure for storing the set of range results
*          required for the mm1 and mm2 offset calibration
*          functions
*
*/

#[derive(Default)]
pub struct VL53L1_offset_range_data_t {
    pub preset_mode: u8,
    /*  Preset Mode use for range  */
    pub dss_config__roi_mode_control: u8,
    /* Dynamic SPAD selection mode */
    pub dss_config__manual_effective_spads_select: u16,
    /*  Requested number of manual effective SPAD's */
    pub no_of_samples: u8,
    /*  Number of ranges  */
    pub effective_spads: u32,
    /* Average effective SPAD's 8.8 format */
    pub peak_rate_mcps: u32,
    /* Average peak rate Mcps 9.7 format  */
    pub sigma_mm: u32,
    /* Average sigma in [mm] 14.2 format */
    pub median_range_mm: i32,
    /* Avg of median range over all ranges \
    note value is signed */
    pub range_mm_offset: i32,
    /* The calculated range offset value */
}

/**
 * @struct  VL53L1_offset_range_results_t
* @brief   Structure for storing the set of range results
*          required for the offset calibration functions
*
*/

#[derive(Default)]
pub struct VL53L1_offset_range_results_t {
    pub cal_distance_mm: i16,
    pub cal_status: VL53L1_Error,
    pub cal_report: u8,
    pub max_results: u8,
    pub active_results: u8,
    pub data: [VL53L1_offset_range_data_t; VL53L1_MAX_OFFSET_RANGE_RESULTS],
}

/**
 *  @struct  VL53L1_additional_offset_cal_data_t
*  @brief   Additional Offset Calibration Data
*
*  Additional offset calibration data. Contains the rate
*  and effective SPAD counts for the MM inner and outer
*  calibration steps.
*/

#[derive(Default)]
pub struct VL53L1_additional_offset_cal_data_t {
    pub result__mm_inner_actual_effective_spads: u16,
    /* MM Inner actual effective SPADs, 8.8 format */
    pub result__mm_outer_actual_effective_spads: u16,
    /* MM Outer actual effective SPADs, 8.8 format */
    pub result__mm_inner_peak_signal_count_rtn_mcps: u16,
    /* Mean value of MM Inner return peak rate in [Mcps], 9.7 format */
    pub result__mm_outer_peak_signal_count_rtn_mcps: u16,
    /* Mean value of MM Outer return peak rate in [Mcps], 9.7 format */
}

/**
 * @struct  VL53L1_cal_peak_rate_map_t
* @brief   Structure for storing the calibration peak rate map
*          Used by DMAX to understand the spatial roll off
*          in the signal rate map towards the corner of the
*          SPAD array.
*/

#[derive(Default)]
pub struct VL53L1_cal_peak_rate_map_t {
    pub cal_distance_mm: i16,
    /* calibration distance in [mm], 14.2 format */
    pub max_samples: u16,
    /* Array size for rate map i.e. max number samples */
    pub width: u16,
    /* Array width */
    pub height: u16,
    /* Array height */
    pub peak_rate_mcps: [u16; VL53L1_NVM_PEAK_RATE_MAP_SAMPLES],
    /* Array of rate map samples */
}

/**
 * @struct VL53L1_gain_calibration_data_t
*
* @brief Gain calibration data
*
*/

#[derive(Default)]
pub struct VL53L1_gain_calibration_data_t {
    pub standard_ranging_gain_factor: u16, /* Standard ranging gain correction factor 1.11 format */
}

/**
 * @struct VL53L1_ll_driver_state_t
*
* @brief  Contains the driver state information
*
*/

#[derive(Default)]
pub struct VL53L1_ll_driver_state_t {
    pub cfg_device_state: VL53L1_DeviceState,
    /* Configuration Device State */
    pub cfg_stream_count: u8,
    /* configuration stream count, becomes expected
    stream count for zone */
    pub cfg_gph_id: u8,
    /* Config Grouped Parameter Hold ID */
    pub cfg_timing_status: u8,
    /* Timing A or B flag 0 = A, 1 = B */
    pub rd_device_state: VL53L1_DeviceState,
    /* Read Device State */
    pub rd_stream_count: u8,
    /* rd stream count, used to check actual stream count */
    pub rd_gph_id: u8,
    /* Read Grouped Parameter Hold ID */
    pub rd_timing_status: u8,
    /* Timing A or B flag 0 = A, 1 = B */
}

/** @brief Run Offset Cal Function (offsetcal) Config
 */

#[derive(Default)]
pub struct VL53L1_offsetcal_config_t {
    pub dss_config__target_total_rate_mcps: u16,
    /* DSS Target rate in MCPS (9.7 format) used \
     * during run_offset_calibration() */
    pub phasecal_config_timeout_us: u32,
    /* Phasecal timeout in us \
     * used during run_offset_calibration() */
    pub range_config_timeout_us: u32,
    /* Range timeout in us used during \
     * run_offset_calibration() */
    pub mm_config_timeout_us: u32,
    /* MM timeout in us used during \
     * run_offset_calibration() \
     * Added as part of Patch_AddedOffsetCalMMTuningParm_11791 */
    pub pre_num_of_samples: u8,
    /* Number of Ranging samples used during \
     * run_offset_calibration() */
    pub mm1_num_of_samples: u8,
    /* Number of MM1 samples used during \
     * run_offset_calibration() */
    pub mm2_num_of_samples: u8,
    /* Number of MM2 samples used during \
     * run_offset_calibration() */
}

/**
 * @struct VL53L1_LLDriverData_t
*
* @brief VL53L1 LL Driver ST private data structure \n
*
*/
#[derive(Default)]
pub struct VL53L1_LLDriverData_t {
    pub wait_method: u8,
    pub preset_mode: VL53L1_DevicePresetModes,
    pub measurement_mode: VL53L1_DeviceMeasurementModes,
    pub offset_calibration_mode: VL53L1_OffsetCalibrationMode,
    pub offset_correction_mode: VL53L1_OffsetCorrectionMode,
    pub phasecal_config_timeout_us: u32,
    pub mm_config_timeout_us: u32,
    pub range_config_timeout_us: u32,
    pub inter_measurement_period_ms: u32,
    pub dss_config__target_total_rate_mcps: u16,
    pub fw_ready_poll_duration_ms: u32,
    pub fw_ready: u8,
    pub debug_mode: u8,
    pub version: VL53L1_ll_version_t,
    pub ll_state: VL53L1_ll_driver_state_t,
    pub gpio_interrupt_config: VL53L1_GPIO_interrupt_config_t,
    pub customer: VL53L1_customer_nvm_managed_t,
    pub cal_peak_rate_map: VL53L1_cal_peak_rate_map_t,
    pub add_off_cal_data: VL53L1_additional_offset_cal_data_t,
    pub gain_cal: VL53L1_gain_calibration_data_t,
    pub mm_roi: VL53L1_user_zone_t,
    pub optical_centre: VL53L1_optical_centre_t,
    pub tuning_parms: VL53L1_tuning_parm_storage_t,
    pub rtn_good_spads: [u8; VL53L1_RTN_SPAD_BUFFER_SIZE],
    pub refspadchar: VL53L1_refspadchar_config_t,
    pub ssc_cfg: VL53L1_ssc_config_t,
    pub xtalk_cfg: VL53L1_xtalk_config_t,
    pub offsetcal_cfg: VL53L1_offsetcal_config_t,
    pub stat_nvm: VL53L1_static_nvm_managed_t,
    pub stat_cfg: VL53L1_static_config_t,
    pub gen_cfg: VL53L1_general_config_t,
    pub tim_cfg: VL53L1_timing_config_t,
    pub dyn_cfg: VL53L1_dynamic_config_t,
    pub sys_ctrl: VL53L1_system_control_t,
    pub sys_results: VL53L1_system_results_t,
    pub nvm_copy_data: VL53L1_nvm_copy_data_t,
    pub offset_results: VL53L1_offset_range_results_t,
    pub core_results: VL53L1_core_results_t,
    pub dbg_results: VL53L1_debug_results_t,
    pub low_power_auto_data: VL53L1_low_power_auto_data_t,
}

/**
 * @struct VL53L1_LLDriverResults_t
*
* @brief VL53L1 LL Driver ST private results structure
*
*/

#[derive(Default)]
pub struct VL53L1_LLDriverResults_t {
    pub range_results: VL53L1_range_results_t,
}

/**
 * @struct VL53L1_calibration_data_t
*
* @brief Per Part calibration data
*
*/

#[derive(Default)]
pub struct VL53L1_calibration_data_t {
    pub struct_version: u32,

    pub customer: VL53L1_customer_nvm_managed_t,

    pub add_off_cal_data: VL53L1_additional_offset_cal_data_t,

    pub optical_centre: VL53L1_optical_centre_t,

    pub gain_cal: VL53L1_gain_calibration_data_t,

    pub cal_peak_rate_map: VL53L1_cal_peak_rate_map_t,
}

/**
 * @struct VL53L1_tuning_parameters_t
*
* @brief Tuning Parameters Debug data
*
*/

#[derive(Default)]
pub struct VL53L1_tuning_parameters_t {
    pub vl53l1_tuningparm_version: u16,
    pub vl53l1_tuningparm_key_table_version: u16,
    pub vl53l1_tuningparm_lld_version: u16,
    pub vl53l1_tuningparm_consistency_lite_phase_tolerance: u8,
    pub vl53l1_tuningparm_phasecal_target: u8,
    pub vl53l1_tuningparm_lite_cal_repeat_rate: u16,
    pub vl53l1_tuningparm_lite_ranging_gain_factor: u16,
    pub vl53l1_tuningparm_lite_min_clip_mm: u8,
    pub vl53l1_tuningparm_lite_long_sigma_thresh_mm: u16,
    pub vl53l1_tuningparm_lite_med_sigma_thresh_mm: u16,
    pub vl53l1_tuningparm_lite_short_sigma_thresh_mm: u16,
    pub vl53l1_tuningparm_lite_long_min_count_rate_rtn_mcps: u16,
    pub vl53l1_tuningparm_lite_med_min_count_rate_rtn_mcps: u16,
    pub vl53l1_tuningparm_lite_short_min_count_rate_rtn_mcps: u16,
    pub vl53l1_tuningparm_lite_sigma_est_pulse_width: u8,
    pub vl53l1_tuningparm_lite_sigma_est_amb_width_ns: u8,
    pub vl53l1_tuningparm_lite_sigma_ref_mm: u8,
    pub vl53l1_tuningparm_lite_rit_mult: u8,
    pub vl53l1_tuningparm_lite_seed_config: u8,
    pub vl53l1_tuningparm_lite_quantifier: u8,
    pub vl53l1_tuningparm_lite_first_order_select: u8,
    pub vl53l1_tuningparm_lite_xtalk_margin_kcps: i16,
    pub vl53l1_tuningparm_initial_phase_rtn_lite_long_range: u8,
    pub vl53l1_tuningparm_initial_phase_rtn_lite_med_range: u8,
    pub vl53l1_tuningparm_initial_phase_rtn_lite_short_range: u8,
    pub vl53l1_tuningparm_initial_phase_ref_lite_long_range: u8,
    pub vl53l1_tuningparm_initial_phase_ref_lite_med_range: u8,
    pub vl53l1_tuningparm_initial_phase_ref_lite_short_range: u8,
    pub vl53l1_tuningparm_timed_seed_config: u8,
    pub vl53l1_tuningparm_vhv_loopbound: u8,
    pub vl53l1_tuningparm_refspadchar_device_test_mode: u8,
    pub vl53l1_tuningparm_refspadchar_vcsel_period: u8,
    pub vl53l1_tuningparm_refspadchar_phasecal_timeout_us: u32,
    pub vl53l1_tuningparm_refspadchar_target_count_rate_mcps: u16,
    pub vl53l1_tuningparm_refspadchar_min_countrate_limit_mcps: u16,
    pub vl53l1_tuningparm_refspadchar_max_countrate_limit_mcps: u16,
    pub vl53l1_tuningparm_offset_cal_dss_rate_mcps: u16,
    pub vl53l1_tuningparm_offset_cal_phasecal_timeout_us: u32,
    pub vl53l1_tuningparm_offset_cal_mm_timeout_us: u32,
    pub vl53l1_tuningparm_offset_cal_range_timeout_us: u32,
    pub vl53l1_tuningparm_offset_cal_pre_samples: u8,
    pub vl53l1_tuningparm_offset_cal_mm1_samples: u8,
    pub vl53l1_tuningparm_offset_cal_mm2_samples: u8,
    pub vl53l1_tuningparm_spadmap_vcsel_period: u8,
    pub vl53l1_tuningparm_spadmap_vcsel_start: u8,
    pub vl53l1_tuningparm_spadmap_rate_limit_mcps: u16,
    pub vl53l1_tuningparm_lite_dss_config_target_total_rate_mcps: u16,
    pub vl53l1_tuningparm_timed_dss_config_target_total_rate_mcps: u16,
    pub vl53l1_tuningparm_lite_phasecal_config_timeout_us: u32,
    pub vl53l1_tuningparm_timed_phasecal_config_timeout_us: u32,
    pub vl53l1_tuningparm_lite_mm_config_timeout_us: u32,
    pub vl53l1_tuningparm_timed_mm_config_timeout_us: u32,
    pub vl53l1_tuningparm_lite_range_config_timeout_us: u32,
    pub vl53l1_tuningparm_timed_range_config_timeout_us: u32,
    pub vl53l1_tuningparm_lowpowerauto_vhv_loop_bound: u8,
    pub vl53l1_tuningparm_lowpowerauto_mm_config_timeout_us: u32,
    pub vl53l1_tuningparm_lowpowerauto_range_config_timeout_us: u32,
}

/**
 * @struct  VL53L1_spad_rate_data_t
* @brief   SPAD Rate Data output by SSC
*
* Container for the SPAD Rate data output by SPAD select check (SSC)
* The data is stored in the buffer in SPAD number order and not
* raster order
*
* Rate data is it either 1.15 or 9.7 fixed point format
*/

pub struct VL53L1_spad_rate_data_t {
    pub spad_type: u8,
    /* Type of rate data stored */
    pub buffer_size: u16,
    /* SPAD buffer size : should be at least 256 for EwokPlus25 */
    pub rate_data: [u16; VL53L1_NO_OF_SPAD_ENABLES],
    /* word buffer containing the SPAD rates  */
    pub no_of_values: u16,
    /* Number of bytes used in the buffer */
    pub fractional_bits: u8,
    /* Number of fractional bits either 7 or 15 */
    pub error_status: u8,
    /* Set if supplied buffer is too small */
}

impl Default for VL53L1_spad_rate_data_t {
    fn default() -> Self {
        Self {
            spad_type: 0,
            buffer_size: 0,
            rate_data: [0; VL53L1_NO_OF_SPAD_ENABLES],
            no_of_values: 0,
            fractional_bits: 0,
            error_status: 0,
        }
    }
}

/* Start Patch_AdditionalDebugData_11823 */

/**
 * @struct  VL53L1_additional_data_t
* @brief   Additional debug data
*
* Contains the LL Driver configuration information
*/

#[derive(Default)]
pub struct VL53L1_additional_data_t {
    pub preset_mode: VL53L1_DevicePresetModes,
    /* Current preset mode */
    pub measurement_mode: VL53L1_DeviceMeasurementModes,
    /* Current measurement mode */
    pub phasecal_config_timeout_us: u32,
    /* requested Phase Cal Timeout e.g. 1000us */
    pub mm_config_timeout_us: u32,
    /* requested MM Timeout e.g. 2000us */
    pub range_config_timeout_us: u32,
    /* requested Ranging Timeout e.g 13000us */
    pub inter_measurement_period_ms: u32,
    /* requested Timing mode repeat period e.g 100ms */
    pub dss_config__target_total_rate_mcps: u16,
    /* requested DSS Target Total Rate in 9.7 format e.g. 40.0Mcps*/
}

/* End Patch_AdditionalDebugData_11823 */
