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
use super::vl53l1_register_map::*;

/*
 * @file   vl53l1_register_structs.h
 * @brief  VL53L1 Register Structure definitions
 */

pub const VL53L1_STATIC_NVM_MANAGED_I2C_INDEX: u16 = VL53L1_I2C_SLAVE__DEVICE_ADDRESS;
pub const VL53L1_CUSTOMER_NVM_MANAGED_I2C_INDEX: u16 = VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_0;
pub const VL53L1_STATIC_CONFIG_I2C_INDEX: u16 = VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS;
pub const VL53L1_GENERAL_CONFIG_I2C_INDEX: u16 = VL53L1_GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE;
pub const VL53L1_TIMING_CONFIG_I2C_INDEX: u16 = VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_HI;
pub const VL53L1_DYNAMIC_CONFIG_I2C_INDEX: u16 = VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0;
pub const VL53L1_SYSTEM_CONTROL_I2C_INDEX: u16 = VL53L1_POWER_MANAGEMENT__GO1_POWER_FORCE;
pub const VL53L1_SYSTEM_RESULTS_I2C_INDEX: u16 = VL53L1_RESULT__INTERRUPT_STATUS;
pub const VL53L1_CORE_RESULTS_I2C_INDEX: u16 = VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0;
pub const VL53L1_DEBUG_RESULTS_I2C_INDEX: u16 = VL53L1_PHASECAL_RESULT__REFERENCE_PHASE;
pub const VL53L1_NVM_COPY_DATA_I2C_INDEX: u16 = VL53L1_IDENTIFICATION__MODEL_ID;
pub const VL53L1_PREV_SHADOW_SYSTEM_RESULTS_I2C_INDEX: u16 =
    VL53L1_PREV_SHADOW_RESULT__INTERRUPT_STATUS;
pub const VL53L1_PREV_SHADOW_CORE_RESULTS_I2C_INDEX: u16 =
    VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0;
pub const VL53L1_PATCH_DEBUG_I2C_INDEX: u16 = VL53L1_RESULT__DEBUG_STATUS;
pub const VL53L1_GPH_GENERAL_CONFIG_I2C_INDEX: u16 = VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH;
pub const VL53L1_GPH_STATIC_CONFIG_I2C_INDEX: u16 = VL53L1_GPH__DSS_CONFIG__ROI_MODE_CONTROL;
pub const VL53L1_GPH_TIMING_CONFIG_I2C_INDEX: u16 = VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI;
pub const VL53L1_FW_INTERNAL_I2C_INDEX: u16 = VL53L1_FIRMWARE__INTERNAL_STREAM_COUNT_DIV;
pub const VL53L1_PATCH_RESULTS_I2C_INDEX: u16 = VL53L1_DSS_CALC__ROI_CTRL;
pub const VL53L1_SHADOW_SYSTEM_RESULTS_I2C_INDEX: u16 = VL53L1_SHADOW_PHASECAL_RESULT__VCSEL_START;
pub const VL53L1_SHADOW_CORE_RESULTS_I2C_INDEX: u16 =
    VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0;

pub const VL53L1_STATIC_NVM_MANAGED_I2C_SIZE_BYTES: usize = 11;
pub const VL53L1_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES: usize = 23;
pub const VL53L1_STATIC_CONFIG_I2C_SIZE_BYTES: usize = 32;
pub const VL53L1_GENERAL_CONFIG_I2C_SIZE_BYTES: usize = 22;
pub const VL53L1_TIMING_CONFIG_I2C_SIZE_BYTES: usize = 23;
pub const VL53L1_DYNAMIC_CONFIG_I2C_SIZE_BYTES: usize = 18;
pub const VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES: usize = 5;
pub const VL53L1_SYSTEM_RESULTS_I2C_SIZE_BYTES: usize = 44;
pub const VL53L1_CORE_RESULTS_I2C_SIZE_BYTES: usize = 33;
pub const VL53L1_DEBUG_RESULTS_I2C_SIZE_BYTES: usize = 56;
pub const VL53L1_NVM_COPY_DATA_I2C_SIZE_BYTES: usize = 49;
pub const VL53L1_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES: usize = 44;
pub const VL53L1_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES: usize = 33;
pub const VL53L1_PATCH_DEBUG_I2C_SIZE_BYTES: usize = 2;
pub const VL53L1_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES: usize = 5;
pub const VL53L1_GPH_STATIC_CONFIG_I2C_SIZE_BYTES: usize = 6;
pub const VL53L1_GPH_TIMING_CONFIG_I2C_SIZE_BYTES: usize = 16;
pub const VL53L1_FW_INTERNAL_I2C_SIZE_BYTES: usize = 2;
pub const VL53L1_PATCH_RESULTS_I2C_SIZE_BYTES: usize = 90;
pub const VL53L1_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES: usize = 82;
pub const VL53L1_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES: usize = 33;

/**
 * @struct VL53L1_static_nvm_managed_t
*
* - registers    =     10
* - first_index  =      1 (0x0001)
* - last _index  =     11 (0x000B)
* - i2c_size     =     11
*/

#[derive(Default)]
pub struct VL53L1_static_nvm_managed_t {
    pub i2c_slave__device_address: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = i2c_slave_device_address
    */
    pub ana_config__vhv_ref_sel_vddpix: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = ref_sel_vddpix
    */
    pub ana_config__vhv_ref_sel_vquench: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  3
            - i2c_size =  1

        fields: \n
            - [6:3] = ref_sel_vquench
    */
    pub ana_config__reg_avdd1v2_sel: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = reg_avdd1v2_sel
    */
    pub ana_config__fast_osc__trim: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = fast_osc_trim
    */
    pub osc_measured__fast_osc__frequency: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = osc_frequency (fixed point 4.12)
    */
    pub vhv_config__timeout_macrop_loop_bound: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = vhv_timeout__macrop
            - [7:2] = vhv_loop_bound
    */
    pub vhv_config__count_thresh: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = vhv_count_thresh
    */
    pub vhv_config__offset: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = vhv_step_val
    */
    pub vhv_config__init: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [7] = vhv0_init_enable
            - [5:0] = vhv0_init_value
    */
}

/**
 * @struct VL53L1_customer_nvm_managed_t
*
* - registers    =     16
* - first_index  =     13 (0x000D)
* - last _index  =     34 (0x0022)
* - i2c_size     =     23
*/

#[derive(Default)]
pub struct VL53L1_customer_nvm_managed_t {
    pub global_config__spad_enables_ref_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_ref_0
    */
    pub global_config__spad_enables_ref_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_ref_1
    */
    pub global_config__spad_enables_ref_2: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_ref_2
    */
    pub global_config__spad_enables_ref_3: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_ref_3
    */
    pub global_config__spad_enables_ref_4: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_ref_4
    */
    pub global_config__spad_enables_ref_5: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = spad_enables_ref_5
    */
    pub global_config__ref_en_start_select: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = ref_en_start_select
    */
    pub ref_spad_man__num_requested_ref_spads: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = ref_spad_man__num_requested_ref_spad
    */
    pub ref_spad_man__ref_location: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = ref_spad_man__ref_location
    */
    pub algo__crosstalk_compensation_plane_offset_kcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = crosstalk_compensation_plane_offset_kcps (fixed point 7.9)
    */
    pub algo__crosstalk_compensation_x_plane_gradient_kcps: i16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = crosstalk_compensation_x_plane_gradient_kcps (fixed point 5.11)
    */
    pub algo__crosstalk_compensation_y_plane_gradient_kcps: i16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = crosstalk_compensation_y_plane_gradient_kcps (fixed point 5.11)
    */
    pub ref_spad_char__total_rate_target_mcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = ref_spad_char__total_rate_target_mcps (fixed point 9.7)
    */
    pub algo__part_to_part_range_offset_mm: i16,
    /*
     info: \n
            - msb = 12
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [12:0] = part_to_part_offset_mm (fixed point 11.2)
    */
    pub mm_config__inner_offset_mm: i16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = mm_config__inner_offset_mm
    */
    pub mm_config__outer_offset_mm: i16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = mm_config__outer_offset_mm
    */
}

/**
 * @struct VL53L1_static_config_t
*
* - registers    =     30
* - first_index  =     36 (0x0024)
* - last _index  =     67 (0x0043)
* - i2c_size     =     32
*/

#[derive(Default)]
pub struct VL53L1_static_config_t {
    pub dss_config__target_total_rate_mcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = dss_config__target_total_rate_mcps (fixed point 9.7)
    */
    pub debug__ctrl: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = enable_result_logging
    */
    pub test_mode__ctrl: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = test_mode__cmd
    */
    pub clk_gating__ctrl: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = clk_gate_en__mcu_bank
            -   [1] = clk_gate_en__mcu_patch_ctrl
            -   [2] = clk_gate_en__mcu_timers
            -   [3] = clk_gate_en__mcu_mult_div
    */
    pub nvm_bist__ctrl: u8,
    /*
     info: \n
            - msb =  4
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = nvm_bist__cmd
            -   [4] = nvm_bist__ctrl
    */
    pub nvm_bist__num_nvm_words: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = nvm_bist__num_nvm_words
    */
    pub nvm_bist__start_address: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = nvm_bist__start_address
    */
    pub host_if__status: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = host_interface
    */
    pub pad_i2c_hv__config: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = pad_scl_sda__vmodeint_hv
            -   [1] = i2c_pad__test_hv
            -   [2] = pad_scl__fpen_hv
            - [4:3] = pad_scl__progdel_hv
            -   [5] = pad_sda__fpen_hv
            - [7:6] = pad_sda__progdel_hv
    */
    pub pad_i2c_hv__extsup_config: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = pad_scl_sda__extsup_hv
    */
    pub gpio_hv_pad__ctrl: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = gpio__extsup_hv
            -   [1] = gpio__vmodeint_hv
    */
    pub gpio_hv_mux__ctrl: u8,
    /*
     info: \n
            - msb =  4
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = gpio__mux_select_hv
            -   [4] = gpio__mux_active_high_hv
    */
    pub gpio__tio_hv_status: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = gpio__tio_hv
            -   [1] = fresh_out_of_reset
    */
    pub gpio__fio_hv_status: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  1
            - i2c_size =  1

        fields: \n
            -   [1] = gpio__fio_hv
    */
    pub ana_config__spad_sel_pswidth: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = spad_sel_pswidth
    */
    pub ana_config__vcsel_pulse_width_offset: u8,
    /*
     info: \n
            - msb =  4
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [4:0] = vcsel_pulse_width_offset (fixed point 1.4)
    */
    pub ana_config__fast_osc__config_ctrl: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = osc_config__latch_bypass
    */
    pub sigma_estimator__effective_pulse_width_ns: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = sigma_estimator__eff_pulse_width
    */
    pub sigma_estimator__effective_ambient_width_ns: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = sigma_estimator__eff_ambient_width
    */
    pub sigma_estimator__sigma_ref_mm: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = sigma_estimator__sigma_ref
    */
    pub algo__crosstalk_compensation_valid_height_mm: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = crosstalk_compensation_valid_height_mm
    */
    pub spare_host_config__static_config_spare_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = static_config_spare_0
    */
    pub spare_host_config__static_config_spare_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = static_config_spare_1
    */
    pub algo__range_ignore_threshold_mcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = range_ignore_thresh_mcps (fixed point 3.13)
    */
    pub algo__range_ignore_valid_height_mm: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = range_ignore_height_mm
    */
    pub algo__range_min_clip: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = algo__range_min_clip_enable
            - [7:1] = algo__range_min_clip_value_mm
    */
    pub algo__consistency_check__tolerance: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = consistency_check_tolerance (fixed point 1.3)
    */
    pub spare_host_config__static_config_spare_2: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = static_config_spare_2
    */
    pub sd_config__reset_stages_msb: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = loop_init__clear_stage
    */
    pub sd_config__reset_stages_lsb: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:4] = accum_reset__clear_stage
            - [3:0] = count_reset__clear_stage
    */
}

/**
 * @struct VL53L1_general_config_t
*
* - registers    =     17
* - first_index  =     68 (0x0044)
* - last _index  =     89 (0x0059)
* - i2c_size     =     22
*/

#[derive(Default)]
pub struct VL53L1_general_config_t {
    pub gph_config__stream_count_update_value: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = stream_count_update_value
    */
    pub global_config__stream_divider: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = stream_count_internal_div
    */
    pub system__interrupt_config_gpio: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = int_mode_distance
            - [3:2] = int_mode_rate
            -   [4] = int_spare
            -   [5] = int_new_measure_ready
            -   [6] = int_no_target_en
            -   [7] = int_combined_mode
    */
    pub cal_config__vcsel_start: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = cal_config__vcsel_start
    */
    pub cal_config__repeat_rate: u16,
    /*
     info: \n
            - msb = 11
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [11:0] = cal_config__repeat_rate
    */
    pub global_config__vcsel_width: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = global_config__vcsel_width
    */
    pub phasecal_config__timeout_macrop: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = phasecal_config__timeout_macrop
    */
    pub phasecal_config__target: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = algo_phasecal_lim
    */
    pub phasecal_config__override: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = phasecal_config__override
    */
    pub dss_config__roi_mode_control: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = dss_config__input_mode
            -   [2] = calculate_roi_enable
    */
    pub system__thresh_rate_high: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = thresh_rate_high (fixed point 9.7)
    */
    pub system__thresh_rate_low: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = thresh_rate_low (fixed point 9.7)
    */
    pub dss_config__manual_effective_spads_select: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = dss_config__manual_effective_spads_select
    */
    pub dss_config__manual_block_select: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_config__manual_block_select
    */
    pub dss_config__aperture_attenuation: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_config__aperture_attenuation
    */
    pub dss_config__max_spads_limit: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_config__max_spads_limit
    */
    pub dss_config__min_spads_limit: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_config__min_spads_limit
    */
}

/**
 * @struct VL53L1_timing_config_t
*
* - registers    =     16
* - first_index  =     90 (0x005A)
* - last _index  =    112 (0x0070)
* - i2c_size     =     23
*/

#[derive(Default)]
pub struct VL53L1_timing_config_t {
    pub mm_config__timeout_macrop_a_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = mm_config__config_timeout_macrop_a_hi
    */
    pub mm_config__timeout_macrop_a_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = mm_config__config_timeout_macrop_a_lo
    */
    pub mm_config__timeout_macrop_b_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = mm_config__config_timeout_macrop_b_hi
    */
    pub mm_config__timeout_macrop_b_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = mm_config__config_timeout_macrop_b_lo
    */
    pub range_config__timeout_macrop_a_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = range_timeout_overall_periods_macrop_a_hi
    */
    pub range_config__timeout_macrop_a_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = range_timeout_overall_periods_macrop_a_lo
    */
    pub range_config__vcsel_period_a: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = range_config__vcsel_period_a
    */
    pub range_config__timeout_macrop_b_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = range_timeout_overall_periods_macrop_b_hi
    */
    pub range_config__timeout_macrop_b_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = range_timeout_overall_periods_macrop_b_lo
    */
    pub range_config__vcsel_period_b: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = range_config__vcsel_period_b
    */
    pub range_config__sigma_thresh: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = range_config__sigma_thresh (fixed point 14.2)
    */
    pub range_config__min_count_rate_rtn_limit_mcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = range_config__min_count_rate_rtn_limit_mcps (fixed point 9.7)
    */
    pub range_config__valid_phase_low: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = range_config__valid_phase_low (fixed point 5.3)
    */
    pub range_config__valid_phase_high: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = range_config__valid_phase_high (fixed point 5.3)
    */
    pub system__intermeasurement_period: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = intermeasurement_period
    */
    pub system__fractional_enable: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = range_fractional_enable
    */
}

/**
 * @struct VL53L1_dynamic_config_t
*
* - registers    =     16
* - first_index  =    113 (0x0071)
* - last _index  =    130 (0x0082)
* - i2c_size     =     18
*/

#[derive(Default)]
pub struct VL53L1_dynamic_config_t {
    pub system__grouped_parameter_hold_0: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = grouped_parameter_hold
            -   [1] = grouped_parameter_hold_id
    */
    pub system__thresh_high: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = thresh_high
    */
    pub system__thresh_low: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = thresh_low
    */
    pub system__enable_xtalk_per_quadrant: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = system__enable_xtalk_per_quadrant
    */
    pub system__seed_config: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = system__seed_config
            -   [2] = system__fw_pause_ctrl
    */
    pub sd_config__woi_sd0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = sd_config__woi_sd0
    */
    pub sd_config__woi_sd1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = sd_config__woi_sd1
    */
    pub sd_config__initial_phase_sd0: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = sd_config__initial_phase_sd0
    */
    pub sd_config__initial_phase_sd1: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = sd_config__initial_phase_sd1
    */
    pub system__grouped_parameter_hold_1: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = grouped_parameter_hold
            -   [1] = grouped_parameter_hold_id
    */
    pub sd_config__first_order_select: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = sd_config__first_order_select_rtn
            -   [1] = sd_config__first_order_select_ref
    */
    pub sd_config__quantifier: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = sd_config__quantifier
    */
    pub roi_config__user_roi_centre_spad: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = user_roi_center_spad
    */
    pub roi_config__user_roi_requested_global_xy_size: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = roi_config__user_roi_requested_global_xy_size
    */
    pub system__sequence_config: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = sequence_vhv_en
            -   [1] = sequence_phasecal_en
            -   [2] = sequence_reference_phase_en
            -   [3] = sequence_dss1_en
            -   [4] = sequence_dss2_en
            -   [5] = sequence_mm1_en
            -   [6] = sequence_mm2_en
            -   [7] = sequence_range_en
    */
    pub system__grouped_parameter_hold: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = grouped_parameter_hold
            -   [1] = grouped_parameter_hold_id
    */
}

/**
 * @struct VL53L1_system_control_t
*
* - registers    =      5
* - first_index  =    131 (0x0083)
* - last _index  =    135 (0x0087)
* - i2c_size     =      5
*/

#[derive(Default)]
pub struct VL53L1_system_control_t {
    pub power_management__go1_power_force: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = go1_dig_powerforce
    */
    pub system__stream_count_ctrl: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = retain_stream_count
    */
    pub firmware__enable: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = firmware_enable
    */
    pub system__interrupt_clear: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = sys_interrupt_clear_range
            -   [1] = sys_interrupt_clear_error
    */
    pub system__mode_start: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = scheduler_mode
            - [3:2] = readout_mode
            -   [4] = mode_range__single_shot
            -   [5] = mode_range__back_to_back
            -   [6] = mode_range__timed
            -   [7] = mode_range__abort
    */
}

/**
 * @struct VL53L1_system_results_t
*
* - registers    =     25
* - first_index  =    136 (0x0088)
* - last _index  =    179 (0x00B3)
* - i2c_size     =     44
*/

#[derive(Default)]
pub struct VL53L1_system_results_t {
    pub result__interrupt_status: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = int_status
            - [4:3] = int_error_status
            -   [5] = gph_id_gpio_status
    */
    pub result__range_status: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [4:0] = range_status
            -   [5] = max_threshold_hit
            -   [6] = min_threshold_hit
            -   [7] = gph_id_range_status
    */
    pub result__report_status: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = report_status
    */
    pub result__stream_count: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = result__stream_count
    */
    pub result__dss_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__dss_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub result__peak_signal_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__peak_signal_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub result__ambient_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__ambient_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub result__sigma_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__sigma_sd0 (fixed point 14.2)
    */
    pub result__phase_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__phase_sd0 (fixed point 5.11)
    */
    pub result__final_crosstalk_corrected_range_mm_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__final_crosstalk_corrected_range_mm_sd0
    */
    pub result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 (fixed point 9.7)
    */
    pub result__mm_inner_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__mm_inner_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub result__mm_outer_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__mm_outer_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub result__avg_signal_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__avg_signal_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub result__dss_actual_effective_spads_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__dss_actual_effective_spads_sd1 (fixed point 8.8)
    */
    pub result__peak_signal_count_rate_mcps_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__peak_signal_count_rate_mcps_sd1 (fixed point 9.7)
    */
    pub result__ambient_count_rate_mcps_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__ambient_count_rate_mcps_sd1 (fixed point 9.7)
    */
    pub result__sigma_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__sigma_sd1 (fixed point 14.2)
    */
    pub result__phase_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__phase_sd1 (fixed point 5.11)
    */
    pub result__final_crosstalk_corrected_range_mm_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__final_crosstalk_corrected_range_mm_sd1
    */
    pub result__spare_0_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__spare_0_sd1
    */
    pub result__spare_1_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__spare_1_sd1
    */
    pub result__spare_2_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result__spare_2_sd1
    */
    pub result__spare_3_sd1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = result__spare_3_sd1
    */
    pub result__thresh_info: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = result__distance_int_info
            - [7:4] = result__rate_int_info
    */
}

/**
 * @struct VL53L1_core_results_t
*
* - registers    =      9
* - first_index  =    180 (0x00B4)
* - last _index  =    212 (0x00D4)
* - i2c_size     =     33
*/

#[derive(Default)]
pub struct VL53L1_core_results_t {
    pub result_core__ambient_window_events_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__ambient_window_events_sd0
    */
    pub result_core__ranging_total_events_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__ranging_total_events_sd0
    */
    pub result_core__signal_total_events_sd0: i32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__signal_total_events_sd0
    */
    pub result_core__total_periods_elapsed_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__total_periods_elapsed_sd0
    */
    pub result_core__ambient_window_events_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__ambient_window_events_sd1
    */
    pub result_core__ranging_total_events_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__ranging_total_events_sd1
    */
    pub result_core__signal_total_events_sd1: i32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__signal_total_events_sd1
    */
    pub result_core__total_periods_elapsed_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = result_core__total_periods_elapsed_sd1
    */
    pub result_core__spare_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = result_core__spare_0
    */
}

/**
 * @struct VL53L1_debug_results_t
*
* - registers    =     43
* - first_index  =    214 (0x00D6)
* - last _index  =    269 (0x010D)
* - i2c_size     =     56
*/

#[derive(Default)]
pub struct VL53L1_debug_results_t {
    pub phasecal_result__reference_phase: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = result_phasecal__reference_phase (fixed point 5.11)
    */
    pub phasecal_result__vcsel_start: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = result_phasecal__vcsel_start
    */
    pub ref_spad_char_result__num_actual_ref_spads: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = ref_spad_char_result__num_actual_ref_spads
    */
    pub ref_spad_char_result__ref_location: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = ref_spad_char_result__ref_location
    */
    pub vhv_result__coldboot_status: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = vhv_result__coldboot_status
    */
    pub vhv_result__search_result: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = cp_sel_result
    */
    pub vhv_result__latest_setting: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = cp_sel_latest_setting
    */
    pub result__osc_calibrate_val: u16,
    /*
     info: \n
            - msb =  9
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [9:0] = osc_calibrate_val
    */
    pub ana_config__powerdown_go1: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = go2_ref_bg_disable_avdd
            -   [1] = go2_regdvdd1v2_enable_avdd
    */
    pub ana_config__ref_bg_ctrl: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = go2_ref_overdrvbg_avdd
            -   [1] = go2_ref_forcebgison_avdd
    */
    pub ana_config__regdvdd1v2_ctrl: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = go2_regdvdd1v2_sel_pulldown_avdd
            -   [1] = go2_regdvdd1v2_sel_boost_avdd
            - [3:2] = go2_regdvdd1v2_selv_avdd
    */
    pub ana_config__osc_slow_ctrl: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = osc_slow_en
            -   [1] = osc_slow_op_en
            -   [2] = osc_slow_freq_sel
    */
    pub test_mode__status: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = test_mode_status
    */
    pub firmware__system_status: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = firmware_bootup
            -   [1] = firmware_first_range
    */
    pub firmware__mode_status: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = firmware_mode_status
    */
    pub firmware__secondary_mode_status: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = fw_secondary_mode_status
    */
    pub firmware__cal_repeat_rate_counter: u16,
    /*
     info: \n
            - msb = 11
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [11:0] = firmware_cal_repeat_rate
    */
    pub gph__system__thresh_high: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_thresh_high
    */
    pub gph__system__thresh_low: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_thresh_low
    */
    pub gph__system__enable_xtalk_per_quadrant: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = shadow__enable_xtalk_per_quadrant
    */
    pub gph__spare_0: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = fw_safe_to_disable
            -   [1] = shadow__spare_0
            -   [2] = shadow__spare_1
    */
    pub gph__sd_config__woi_sd0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_sd_config__woi_sd0
    */
    pub gph__sd_config__woi_sd1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_sd_config__woi_sd1
    */
    pub gph__sd_config__initial_phase_sd0: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = shadow_sd_config__initial_phase_sd0
    */
    pub gph__sd_config__initial_phase_sd1: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = shadow_sd_config__initial_phase_sd1
    */
    pub gph__sd_config__first_order_select: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = shadow_sd_config__first_order_select_rtn
            -   [1] = shadow_sd_config__first_order_select_ref
    */
    pub gph__sd_config__quantifier: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = shadow_sd_config__quantifier
    */
    pub gph__roi_config__user_roi_centre_spad: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_user_roi_center_spad_q0
    */
    pub gph__roi_config__user_roi_requested_global_xy_size: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_user_roi_requested_global_xy_size
    */
    pub gph__system__sequence_config: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = shadow_sequence_vhv_en
            -   [1] = shadow_sequence_phasecal_en
            -   [2] = shadow_sequence_reference_phase_en
            -   [3] = shadow_sequence_dss1_en
            -   [4] = shadow_sequence_dss2_en
            -   [5] = shadow_sequence_mm1_en
            -   [6] = shadow_sequence_mm2_en
            -   [7] = shadow_sequence_range_en
    */
    pub gph__gph_id: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = shadow_gph_id
    */
    pub system__interrupt_set: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = sys_interrupt_set_range
            -   [1] = sys_interrupt_set_error
    */
    pub interrupt_manager__enables: u8,
    /*
     info: \n
            - msb =  4
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = interrupt_enable__single_shot
            -   [1] = interrupt_enable__back_to_back
            -   [2] = interrupt_enable__timed
            -   [3] = interrupt_enable__abort
            -   [4] = interrupt_enable__test
    */
    pub interrupt_manager__clear: u8,
    /*
     info: \n
            - msb =  4
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = interrupt_clear__single_shot
            -   [1] = interrupt_clear__back_to_back
            -   [2] = interrupt_clear__timed
            -   [3] = interrupt_clear__abort
            -   [4] = interrupt_clear__test
    */
    pub interrupt_manager__status: u8,
    /*
     info: \n
            - msb =  4
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = interrupt_status__single_shot
            -   [1] = interrupt_status__back_to_back
            -   [2] = interrupt_status__timed
            -   [3] = interrupt_status__abort
            -   [4] = interrupt_status__test
    */
    pub mcu_to_host_bank__wr_access_en: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = mcu_to_host_bank_wr_en
    */
    pub power_management__go1_reset_status: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = go1_status
    */
    pub pad_startup_mode__value_ro: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = pad_atest1_val_ro
            -   [1] = pad_atest2_val_ro
    */
    pub pad_startup_mode__value_ctrl: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = pad_atest1_val
            -   [1] = pad_atest2_val
            -   [4] = pad_atest1_dig_enable
            -   [5] = pad_atest2_dig_enable
    */
    pub pll_period_us: u32,
    /*
     info: \n
            - msb = 17
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [17:0] = pll_period_us (fixed point 0.24)
    */
    pub interrupt_scheduler__data_out: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = interrupt_scheduler_data_out
    */
    pub nvm_bist__complete: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = nvm_bist__complete
    */
    pub nvm_bist__status: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = nvm_bist__status
    */
}

/**
 * @struct VL53L1_nvm_copy_data_t
*
* - registers    =     48
* - first_index  =    271 (0x010F)
* - last _index  =    319 (0x013F)
* - i2c_size     =     49
*/

#[derive(Default)]
pub struct VL53L1_nvm_copy_data_t {
    pub identification__model_id: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = model_id
    */
    pub identification__module_type: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = module_type
    */
    pub identification__revision_id: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = nvm_revision_id
            - [7:4] = mask_revision_id
    */
    pub identification__module_id: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = module_id
    */
    pub ana_config__fast_osc__trim_max: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = osc_trim_max
    */
    pub ana_config__fast_osc__freq_set: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = osc_freq_set
    */
    pub ana_config__vcsel_trim: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = vcsel_trim
    */
    pub ana_config__vcsel_selion: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = vcsel_selion
    */
    pub ana_config__vcsel_selion_max: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = vcsel_selion_max
    */
    pub protected_laser_safety__lock_bit: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = laser_safety__lock_bit
    */
    pub laser_safety__key: u8,
    /*
     info: \n
            - msb =  6
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [6:0] = laser_safety__key
    */
    pub laser_safety__key_ro: u8,
    /*
     info: \n
            - msb =  0
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = laser_safety__key_ro
    */
    pub laser_safety__clip: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = vcsel_pulse_width_clip
    */
    pub laser_safety__mult: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = vcsel_pulse_width_mult
    */
    pub global_config__spad_enables_rtn_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_0
    */
    pub global_config__spad_enables_rtn_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_1
    */
    pub global_config__spad_enables_rtn_2: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_2
    */
    pub global_config__spad_enables_rtn_3: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_3
    */
    pub global_config__spad_enables_rtn_4: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_4
    */
    pub global_config__spad_enables_rtn_5: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_5
    */
    pub global_config__spad_enables_rtn_6: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_6
    */
    pub global_config__spad_enables_rtn_7: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_7
    */
    pub global_config__spad_enables_rtn_8: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_8
    */
    pub global_config__spad_enables_rtn_9: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_9
    */
    pub global_config__spad_enables_rtn_10: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_10
    */
    pub global_config__spad_enables_rtn_11: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_11
    */
    pub global_config__spad_enables_rtn_12: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_12
    */
    pub global_config__spad_enables_rtn_13: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_13
    */
    pub global_config__spad_enables_rtn_14: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_14
    */
    pub global_config__spad_enables_rtn_15: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_15
    */
    pub global_config__spad_enables_rtn_16: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_16
    */
    pub global_config__spad_enables_rtn_17: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_17
    */
    pub global_config__spad_enables_rtn_18: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_18
    */
    pub global_config__spad_enables_rtn_19: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_19
    */
    pub global_config__spad_enables_rtn_20: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_20
    */
    pub global_config__spad_enables_rtn_21: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_21
    */
    pub global_config__spad_enables_rtn_22: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_22
    */
    pub global_config__spad_enables_rtn_23: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_23
    */
    pub global_config__spad_enables_rtn_24: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_24
    */
    pub global_config__spad_enables_rtn_25: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_25
    */
    pub global_config__spad_enables_rtn_26: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_26
    */
    pub global_config__spad_enables_rtn_27: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_27
    */
    pub global_config__spad_enables_rtn_28: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_28
    */
    pub global_config__spad_enables_rtn_29: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_29
    */
    pub global_config__spad_enables_rtn_30: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_30
    */
    pub global_config__spad_enables_rtn_31: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = spad_enables_rtn_31
    */
    pub roi_config__mode_roi_centre_spad: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = mode_roi_center_spad
    */
    pub roi_config__mode_roi_xy_size: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = mode_roi_xy_size
    */
}

/**
 * @struct VL53L1_prev_shadow_system_results_t
*
* - registers    =     24
* - first_index  =   3792 (0x0ED0)
* - last _index  =   3834 (0x0EFA)
* - i2c_size     =     44
*/

#[derive(Default)]
pub struct VL53L1_prev_shadow_system_results_t {
    pub prev_shadow_result__interrupt_status: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = prev_shadow_int_status
            - [4:3] = prev_shadow_int_error_status
            -   [5] = prev_shadow_gph_id_gpio_status
    */
    pub prev_shadow_result__range_status: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [4:0] = prev_shadow_range_status
            -   [5] = prev_shadow_max_threshold_hit
            -   [6] = prev_shadow_min_threshold_hit
            -   [7] = prev_shadow_gph_id_range_status
    */
    pub prev_shadow_result__report_status: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = prev_shadow_report_status
    */
    pub prev_shadow_result__stream_count: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = prev_shadow_result__stream_count
    */
    pub prev_shadow_result__dss_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__dss_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub prev_shadow_result__peak_signal_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__peak_signal_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub prev_shadow_result__ambient_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__ambient_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub prev_shadow_result__sigma_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__sigma_sd0 (fixed point 14.2)
    */
    pub prev_shadow_result__phase_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__phase_sd0 (fixed point 5.11)
    */
    pub prev_shadow_result__final_crosstalk_corrected_range_mm_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__final_crosstalk_corrected_range_mm_sd0
    */
    pub prev_shadow_result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 (fixed point 9.7)
    */
    pub prev_shadow_result__mm_inner_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__mm_inner_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub prev_shadow_result__mm_outer_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__mm_outer_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub prev_shadow_result__avg_signal_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__avg_signal_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub prev_shadow_result__dss_actual_effective_spads_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__dss_actual_effective_spads_sd1 (fixed point 8.8)
    */
    pub prev_shadow_result__peak_signal_count_rate_mcps_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__peak_signal_count_rate_mcps_sd1 (fixed point 9.7)
    */
    pub prev_shadow_result__ambient_count_rate_mcps_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__ambient_count_rate_mcps_sd1 (fixed point 9.7)
    */
    pub prev_shadow_result__sigma_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__sigma_sd1 (fixed point 14.2)
    */
    pub prev_shadow_result__phase_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__phase_sd1 (fixed point 5.11)
    */
    pub prev_shadow_result__final_crosstalk_corrected_range_mm_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__final_crosstalk_corrected_range_mm_sd1
    */
    pub prev_shadow_result__spare_0_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__spare_0_sd1
    */
    pub prev_shadow_result__spare_1_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__spare_1_sd1
    */
    pub prev_shadow_result__spare_2_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__spare_2_sd1
    */
    pub prev_shadow_result__spare_3_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = prev_shadow_result__spare_3_sd1
    */
}

/**
 * @struct VL53L1_prev_shadow_core_results_t
*
* - registers    =      9
* - first_index  =   3836 (0x0EFC)
* - last _index  =   3868 (0x0F1C)
* - i2c_size     =     33
*/

#[derive(Default)]
pub struct VL53L1_prev_shadow_core_results_t {
    pub prev_shadow_result_core__ambient_window_events_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__ambient_window_events_sd0
    */
    pub prev_shadow_result_core__ranging_total_events_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__ranging_total_events_sd0
    */
    pub prev_shadow_result_core__signal_total_events_sd0: i32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__signal_total_events_sd0
    */
    pub prev_shadow_result_core__total_periods_elapsed_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__total_periods_elapsed_sd0
    */
    pub prev_shadow_result_core__ambient_window_events_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__ambient_window_events_sd1
    */
    pub prev_shadow_result_core__ranging_total_events_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__ranging_total_events_sd1
    */
    pub prev_shadow_result_core__signal_total_events_sd1: i32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__signal_total_events_sd1
    */
    pub prev_shadow_result_core__total_periods_elapsed_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = prev_shadow_result_core__total_periods_elapsed_sd1
    */
    pub prev_shadow_result_core__spare_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = prev_shadow_result_core__spare_0
    */
}

/**
 * @struct VL53L1_patch_debug_t
*
* - registers    =      2
* - first_index  =   3872 (0x0F20)
* - last _index  =   3873 (0x0F21)
* - i2c_size     =      2
*/

#[derive(Default)]
pub struct VL53L1_patch_debug_t {
    pub result__debug_status: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = result_debug_status
    */
    pub result__debug_stage: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = result_debug_stage
    */
}

/**
 * @struct VL53L1_gph_general_config_t
*
* - registers    =      3
* - first_index  =   3876 (0x0F24)
* - last _index  =   3880 (0x0F28)
* - i2c_size     =      5
*/

#[derive(Default)]
pub struct VL53L1_gph_general_config_t {
    pub gph__system__thresh_rate_high: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = gph__system_thresh_rate_high (fixed point 9.7)
    */
    pub gph__system__thresh_rate_low: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = gph__system_thresh_rate_low (fixed point 9.7)
    */
    pub gph__system__interrupt_config_gpio: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = gph__int_mode_distance
            - [3:2] = gph__int_mode_rate
            -   [4] = gph__int_spare
            -   [5] = gph__int_new_measure_ready
            -   [6] = gph__int_no_target_en
            -   [7] = gph__int_combined_mode
    */
}

/**
 * @struct VL53L1_gph_static_config_t
*
* - registers    =      5
* - first_index  =   3887 (0x0F2F)
* - last _index  =   3892 (0x0F34)
* - i2c_size     =      6
*/

#[derive(Default)]
pub struct VL53L1_gph_static_config_t {
    pub gph__dss_config__roi_mode_control: u8,
    /*
     info: \n
            - msb =  2
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [1:0] = gph__dss_config__input_mode
            -   [2] = gph__calculate_roi_enable
    */
    pub gph__dss_config__manual_effective_spads_select: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = gph__dss_config__manual_effective_spads_select
    */
    pub gph__dss_config__manual_block_select: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph__dss_config__manual_block_select
    */
    pub gph__dss_config__max_spads_limit: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph__dss_config__max_spads_limit
    */
    pub gph__dss_config__min_spads_limit: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph__dss_config__min_spads_limit
    */
}

/**
 * @struct VL53L1_gph_timing_config_t
*
* - registers    =     14
* - first_index  =   3894 (0x0F36)
* - last _index  =   3909 (0x0F45)
* - i2c_size     =     16
*/

#[derive(Default)]
pub struct VL53L1_gph_timing_config_t {
    pub gph__mm_config__timeout_macrop_a_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = gph_mm_config__config_timeout_macrop_a_hi
    */
    pub gph__mm_config__timeout_macrop_a_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph_mm_config__config_timeout_macrop_a_lo
    */
    pub gph__mm_config__timeout_macrop_b_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = gph_mm_config__config_timeout_macrop_b_hi
    */
    pub gph__mm_config__timeout_macrop_b_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph_mm_config__config_timeout_macrop_b_lo
    */
    pub gph__range_config__timeout_macrop_a_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = gph_range_timeout_overall_periods_macrop_a_hi
    */
    pub gph__range_config__timeout_macrop_a_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph_range_timeout_overall_periods_macrop_a_lo
    */
    pub gph__range_config__vcsel_period_a: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = gph_range_config__vcsel_period_a
    */
    pub gph__range_config__vcsel_period_b: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [5:0] = gph_range_config__vcsel_period_b
    */
    pub gph__range_config__timeout_macrop_b_hi: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = gph_range_timeout_overall_periods_macrop_b_hi
    */
    pub gph__range_config__timeout_macrop_b_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph_range_timeout_overall_periods_macrop_b_lo
    */
    pub gph__range_config__sigma_thresh: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = gph_range_config__sigma_thresh (fixed point 14.2)
    */
    pub gph__range_config__min_count_rate_rtn_limit_mcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = gph_range_config__min_count_rate_rtn_limit_mcps (fixed point 9.7)
    */
    pub gph__range_config__valid_phase_low: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph_range_config__valid_phase_low (fixed point 5.3)
    */
    pub gph__range_config__valid_phase_high: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = gph_range_config__valid_phase_high (fixed point 5.3)
    */
}

/**
 * @struct VL53L1_fw_internal_t
*
* - registers    =      2
* - first_index  =   3910 (0x0F46)
* - last _index  =   3911 (0x0F47)
* - i2c_size     =      2
*/

#[derive(Default)]
pub struct VL53L1_fw_internal_t {
    pub firmware__internal_stream_count_div: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = fw__internal_stream_count_div
    */
    pub firmware__internal_stream_counter_val: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = fw__internal_stream_counter_val
    */
}

/**
 * @struct VL53L1_patch_results_t
*
* - registers    =     60
* - first_index  =   3924 (0x0F54)
* - last _index  =   4012 (0x0FAC)
* - i2c_size     =     90
*/

#[derive(Default)]
pub struct VL53L1_patch_results_t {
    pub dss_calc__roi_ctrl: u8,
    /*
     info: \n
            - msb =  1
            - lsb =  0
            - i2c_size =  1

        fields: \n
            -   [0] = dss_calc__roi_intersect_enable
            -   [1] = dss_calc__roi_subtract_enable
    */
    pub dss_calc__spare_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_1
    */
    pub dss_calc__spare_2: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_2
    */
    pub dss_calc__spare_3: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_3
    */
    pub dss_calc__spare_4: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_4
    */
    pub dss_calc__spare_5: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_5
    */
    pub dss_calc__spare_6: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_6
    */
    pub dss_calc__spare_7: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__spare_7
    */
    pub dss_calc__user_roi_spad_en_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_0
    */
    pub dss_calc__user_roi_spad_en_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_1
    */
    pub dss_calc__user_roi_spad_en_2: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_2
    */
    pub dss_calc__user_roi_spad_en_3: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_3
    */
    pub dss_calc__user_roi_spad_en_4: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_4
    */
    pub dss_calc__user_roi_spad_en_5: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_5
    */
    pub dss_calc__user_roi_spad_en_6: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_6
    */
    pub dss_calc__user_roi_spad_en_7: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_7
    */
    pub dss_calc__user_roi_spad_en_8: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_8
    */
    pub dss_calc__user_roi_spad_en_9: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_9
    */
    pub dss_calc__user_roi_spad_en_10: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_10
    */
    pub dss_calc__user_roi_spad_en_11: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_11
    */
    pub dss_calc__user_roi_spad_en_12: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_12
    */
    pub dss_calc__user_roi_spad_en_13: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_13
    */
    pub dss_calc__user_roi_spad_en_14: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_14
    */
    pub dss_calc__user_roi_spad_en_15: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_15
    */
    pub dss_calc__user_roi_spad_en_16: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_16
    */
    pub dss_calc__user_roi_spad_en_17: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_17
    */
    pub dss_calc__user_roi_spad_en_18: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_18
    */
    pub dss_calc__user_roi_spad_en_19: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_19
    */
    pub dss_calc__user_roi_spad_en_20: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_20
    */
    pub dss_calc__user_roi_spad_en_21: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_21
    */
    pub dss_calc__user_roi_spad_en_22: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_22
    */
    pub dss_calc__user_roi_spad_en_23: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_23
    */
    pub dss_calc__user_roi_spad_en_24: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_24
    */
    pub dss_calc__user_roi_spad_en_25: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_25
    */
    pub dss_calc__user_roi_spad_en_26: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_26
    */
    pub dss_calc__user_roi_spad_en_27: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_27
    */
    pub dss_calc__user_roi_spad_en_28: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_28
    */
    pub dss_calc__user_roi_spad_en_29: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_29
    */
    pub dss_calc__user_roi_spad_en_30: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_30
    */
    pub dss_calc__user_roi_spad_en_31: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_spad_en_31
    */
    pub dss_calc__user_roi_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_0
    */
    pub dss_calc__user_roi_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__user_roi_1
    */
    pub dss_calc__mode_roi_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__mode_roi_0
    */
    pub dss_calc__mode_roi_1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_calc__mode_roi_1
    */
    pub sigma_estimator_calc__spare_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = sigma_estimator_calc__spare_0
    */
    pub vhv_result__peak_signal_rate_mcps: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = vhv_result__peak_signal_rate_mcps
    */
    pub vhv_result__signal_total_events_ref: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = vhv_result__signal_total_events_ref
    */
    pub phasecal_result__phase_output_ref: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = phasecal_result__normalised_phase_ref
    */
    pub dss_result__total_rate_per_spad: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = dss_result__total_rate_per_spad
    */
    pub dss_result__enabled_blocks: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = dss_result__enabled_blocks
    */
    pub dss_result__num_requested_spads: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = dss_result__num_requested_spads (fixed point 8.8)
    */
    pub mm_result__inner_intersection_rate: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = mm_result__inner_intersection_rate
    */
    pub mm_result__outer_complement_rate: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = mm_result__outer_complement_rate
    */
    pub mm_result__total_offset: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = mm_result__total_offset
    */
    pub xtalk_calc__xtalk_for_enabled_spads: u32,
    /*
     info: \n
            - msb = 23
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [23:0] = xtalk_calc__xtalk_for_enabled_spads (fixed point 11.13)
    */
    pub xtalk_result__avg_xtalk_user_roi_kcps: u32,
    /*
     info: \n
            - msb = 23
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [23:0] = xtalk_result__avg_xtalk_user_roi_kcps (fixed point 11.13)
    */
    pub xtalk_result__avg_xtalk_mm_inner_roi_kcps: u32,
    /*
     info: \n
            - msb = 23
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [23:0] = xtalk_result__avg_xtalk_mm_inner_roi_kcps (fixed point 11.13)
    */
    pub xtalk_result__avg_xtalk_mm_outer_roi_kcps: u32,
    /*
     info: \n
            - msb = 23
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [23:0] = xtalk_result__avg_xtalk_mm_outer_roi_kcps (fixed point 11.13)
    */
    pub range_result__accum_phase: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = range_result__accum_phase
    */
    pub range_result__offset_corrected_range: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = range_result__offset_corrected_range
    */
}

/**
 * @struct VL53L1_shadow_system_results_t
*
* - registers    =     28
* - first_index  =   4014 (0x0FAE)
* - last _index  =   4095 (0x0FFF)
* - i2c_size     =     82
*/

#[derive(Default)]
pub struct VL53L1_shadow_system_results_t {
    pub shadow_phasecal_result__vcsel_start: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_phasecal_result__vcsel_start
    */
    pub shadow_result__interrupt_status: u8,
    /*
     info: \n
            - msb =  5
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [2:0] = shadow_int_status
            - [4:3] = shadow_int_error_status
            -   [5] = shadow_gph_id_gpio_status
    */
    pub shadow_result__range_status: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [4:0] = shadow_range_status
            -   [5] = shadow_max_threshold_hit
            -   [6] = shadow_min_threshold_hit
            -   [7] = shadow_gph_id_range_status
    */
    pub shadow_result__report_status: u8,
    /*
     info: \n
            - msb =  3
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = shadow_report_status
    */
    pub shadow_result__stream_count: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_result__stream_count
    */
    pub shadow_result__dss_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__dss_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub shadow_result__peak_signal_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__peak_signal_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub shadow_result__ambient_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__ambient_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub shadow_result__sigma_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__sigma_sd0 (fixed point 14.2)
    */
    pub shadow_result__phase_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__phase_sd0 (fixed point 5.11)
    */
    pub shadow_result__final_crosstalk_corrected_range_mm_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__final_crosstalk_corrected_range_mm_sd0
    */
    pub shadow_result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 (fixed point 9.7)
    */
    pub shadow_result__mm_inner_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__mm_inner_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub shadow_result__mm_outer_actual_effective_spads_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__mm_outer_actual_effective_spads_sd0 (fixed point 8.8)
    */
    pub shadow_result__avg_signal_count_rate_mcps_sd0: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__avg_signal_count_rate_mcps_sd0 (fixed point 9.7)
    */
    pub shadow_result__dss_actual_effective_spads_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__dss_actual_effective_spads_sd1 (fixed point 8.8)
    */
    pub shadow_result__peak_signal_count_rate_mcps_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__peak_signal_count_rate_mcps_sd1 (fixed point 9.7)
    */
    pub shadow_result__ambient_count_rate_mcps_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__ambient_count_rate_mcps_sd1 (fixed point 9.7)
    */
    pub shadow_result__sigma_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__sigma_sd1 (fixed point 14.2)
    */
    pub shadow_result__phase_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__phase_sd1 (fixed point 5.11)
    */
    pub shadow_result__final_crosstalk_corrected_range_mm_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__final_crosstalk_corrected_range_mm_sd1
    */
    pub shadow_result__spare_0_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__spare_0_sd1
    */
    pub shadow_result__spare_1_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__spare_1_sd1
    */
    pub shadow_result__spare_2_sd1: u16,
    /*
     info: \n
            - msb = 15
            - lsb =  0
            - i2c_size =  2

        fields: \n
            - [15:0] = shadow_result__spare_2_sd1
    */
    pub shadow_result__spare_3_sd1: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_result__spare_3_sd1
    */
    pub shadow_result__thresh_info: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [3:0] = shadow_result__distance_int_info
            - [7:4] = shadow_result__rate_int_info
    */
    pub shadow_phasecal_result__reference_phase_hi: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_phasecal_result__reference_phase_hi
    */
    pub shadow_phasecal_result__reference_phase_lo: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_phasecal_result__reference_phase_lo
    */
}

/**
 * @struct VL53L1_shadow_core_results_t
*
* - registers    =      9
* - first_index  =   4060 (0x0FDC)
* - last _index  =   4092 (0x0FFC)
* - i2c_size     =     33
*/

#[derive(Default)]
pub struct VL53L1_shadow_core_results_t {
    pub shadow_result_core__ambient_window_events_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__ambient_window_events_sd0
    */
    pub shadow_result_core__ranging_total_events_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__ranging_total_events_sd0
    */
    pub shadow_result_core__signal_total_events_sd0: i32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__signal_total_events_sd0
    */
    pub shadow_result_core__total_periods_elapsed_sd0: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__total_periods_elapsed_sd0
    */
    pub shadow_result_core__ambient_window_events_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__ambient_window_events_sd1
    */
    pub shadow_result_core__ranging_total_events_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__ranging_total_events_sd1
    */
    pub shadow_result_core__signal_total_events_sd1: i32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__signal_total_events_sd1
    */
    pub shadow_result_core__total_periods_elapsed_sd1: u32,
    /*
     info: \n
            - msb = 31
            - lsb =  0
            - i2c_size =  4

        fields: \n
            - [31:0] = shadow_result_core__total_periods_elapsed_sd1
    */
    pub shadow_result_core__spare_0: u8,
    /*
     info: \n
            - msb =  7
            - lsb =  0
            - i2c_size =  1

        fields: \n
            - [7:0] = shadow_result_core__spare_0
    */
}
