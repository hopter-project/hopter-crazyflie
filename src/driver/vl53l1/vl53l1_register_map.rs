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
/*
 * @file   vl53l1_register_map.h
 * @brief  VL53L1 Register Map definitions
 */

/** @defgroup VL53L1_register_DefineRegisters_group Define Registers *  @brief List of all the defined registers
 */

pub const VL53L1_SOFT_RESET: u16 = 0x0000;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_I2C_SLAVE__DEVICE_ADDRESS: u16 = 0x0001;
/*
    type:     uint8_t \n
    default:  EWOK_I2C_DEV_ADDR_DEFAULT \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'system_config']

    fields: \n
        - [6:0] = i2c_slave_device_address
*/
pub const VL53L1_ANA_CONFIG__VHV_REF_SEL_VDDPIX: u16 = 0x0002;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'analog_config']

    fields: \n
        - [3:0] = ref_sel_vddpix
*/
pub const VL53L1_ANA_CONFIG__VHV_REF_SEL_VQUENCH: u16 = 0x0003;
/*
    type:     uint8_t \n
    default:  0x10 \n
    info: \n
        - msb =  6
        - lsb =  3
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'analog_config']

    fields: \n
        - [6:3] = ref_sel_vquench
*/
pub const VL53L1_ANA_CONFIG__REG_AVDD1V2_SEL: u16 = 0x0004;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'analog_config']

    fields: \n
        - [1:0] = reg_avdd1v2_sel
*/
pub const VL53L1_ANA_CONFIG__FAST_OSC__TRIM: u16 = 0x0005;
/*
    type:     uint8_t \n
    default:  0x48 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'analog_config']

    fields: \n
        - [6:0] = fast_osc_trim
*/
pub const VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY: u16 = 0x0006;
/*
    type:     uint16_t \n
    default:  OSC_FREQUENCY \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['static_nvm_managed', 'analog_config']

    fields: \n
        - [15:0] = osc_frequency (fixed point 4.12)
*/
pub const VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY_HI: u16 = 0x0006;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY_LO: u16 = 0x0007;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008;
/*
    type:     uint8_t \n
    default:  0x81 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'vhv_config']

    fields: \n
        - [1:0] = vhv_timeout__macrop
        - [7:2] = vhv_loop_bound
*/
pub const VL53L1_VHV_CONFIG__COUNT_THRESH: u16 = 0x0009;
/*
    type:     uint8_t \n
    default:  0x80 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'vhv_config']

    fields: \n
        - [7:0] = vhv_count_thresh
*/
pub const VL53L1_VHV_CONFIG__OFFSET: u16 = 0x000A;
/*
    type:     uint8_t \n
    default:  0x07 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'vhv_config']

    fields: \n
        - [5:0] = vhv_step_val
*/
pub const VL53L1_VHV_CONFIG__INIT: u16 = 0x000B;
/*
    type:     uint8_t \n
    default:  0x20 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_nvm_managed', 'vhv_config']

    fields: \n
        -   [7] = vhv0_init_enable
        - [5:0] = vhv0_init_value
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_0: u16 = 0x000D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_en']

    fields: \n
        - [7:0] = spad_enables_ref_0
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_1: u16 = 0x000E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_en']

    fields: \n
        - [7:0] = spad_enables_ref_1
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_2: u16 = 0x000F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_en']

    fields: \n
        - [7:0] = spad_enables_ref_2
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_3: u16 = 0x0010;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_en']

    fields: \n
        - [7:0] = spad_enables_ref_3
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_4: u16 = 0x0011;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_en']

    fields: \n
        - [7:0] = spad_enables_ref_4
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_5: u16 = 0x0012;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_en']

    fields: \n
        - [3:0] = spad_enables_ref_5
*/
pub const VL53L1_GLOBAL_CONFIG__REF_EN_START_SELECT: u16 = 0x0013;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_start']

    fields: \n
        - [7:0] = ref_en_start_select
*/
pub const VL53L1_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS: u16 = 0x0014;
/*
    type:     uint8_t \n
    default:  0x2C \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_config']

    fields: \n
        - [5:0] = ref_spad_man__num_requested_ref_spad
*/
pub const VL53L1_REF_SPAD_MAN__REF_LOCATION: u16 = 0x0015;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['customer_nvm_managed', 'ref_spad_config']

    fields: \n
        - [1:0] = ref_spad_man__ref_location
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS: u16 = 0x0016;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'algo_config']

    fields: \n
        - [15:0] = crosstalk_compensation_plane_offset_kcps (fixed point 7.9)
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_HI: u16 = 0x0016;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_LO: u16 = 0x0017;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS: u16 = 0x0018;
/*
    type:     int16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'algo_config']

    fields: \n
        - [15:0] = crosstalk_compensation_x_plane_gradient_kcps (fixed point 5.11)
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_HI: u16 = 0x0018;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_LO: u16 = 0x0019;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS: u16 = 0x001A;
/*
    type:     int16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'algo_config']

    fields: \n
        - [15:0] = crosstalk_compensation_y_plane_gradient_kcps (fixed point 5.11)
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_HI: u16 = 0x001A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_LO: u16 = 0x001B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS: u16 = 0x001C;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'ref_spad_char']

    fields: \n
        - [15:0] = ref_spad_char__total_rate_target_mcps (fixed point 9.7)
*/
pub const VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_HI: u16 = 0x001C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_LO: u16 = 0x001D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM: u16 = 0x001E;
/*
    type:     int16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 12
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'algo_config']

    fields: \n
        - [12:0] = part_to_part_offset_mm (fixed point 11.2)
*/
pub const VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM_HI: u16 = 0x001E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM_LO: u16 = 0x001F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_CONFIG__INNER_OFFSET_MM: u16 = 0x0020;
/*
    type:     int16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'mm_config']

    fields: \n
        - [15:0] = mm_config__inner_offset_mm
*/
pub const VL53L1_MM_CONFIG__INNER_OFFSET_MM_HI: u16 = 0x0020;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_CONFIG__INNER_OFFSET_MM_LO: u16 = 0x0021;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_CONFIG__OUTER_OFFSET_MM: u16 = 0x0022;
/*
    type:     int16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['customer_nvm_managed', 'mm_config']

    fields: \n
        - [15:0] = mm_config__outer_offset_mm
*/
pub const VL53L1_MM_CONFIG__OUTER_OFFSET_MM_HI: u16 = 0x0022;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_CONFIG__OUTER_OFFSET_MM_LO: u16 = 0x0023;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS: u16 = 0x0024;
/*
    type:     uint16_t \n
    default:  0x0380 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['static_config', 'dss_config']

    fields: \n
        - [15:0] = dss_config__target_total_rate_mcps (fixed point 9.7)
*/
pub const VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_HI: u16 = 0x0024;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_LO: u16 = 0x0025;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DEBUG__CTRL: u16 = 0x0026;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'debug_config']

    fields: \n
        -   [0] = enable_result_logging
*/
pub const VL53L1_TEST_MODE__CTRL: u16 = 0x0027;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'test_mode_config']

    fields: \n
        - [3:0] = test_mode__cmd
*/
pub const VL53L1_CLK_GATING__CTRL: u16 = 0x0028;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'clk_config']

    fields: \n
        -   [0] = clk_gate_en__mcu_bank
        -   [1] = clk_gate_en__mcu_patch_ctrl
        -   [2] = clk_gate_en__mcu_timers
        -   [3] = clk_gate_en__mcu_mult_div
*/
pub const VL53L1_NVM_BIST__CTRL: u16 = 0x0029;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'nvm_bist_config']

    fields: \n
        - [2:0] = nvm_bist__cmd
        -   [4] = nvm_bist__ctrl
*/
pub const VL53L1_NVM_BIST__NUM_NVM_WORDS: u16 = 0x002A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'nvm_bist_config']

    fields: \n
        - [6:0] = nvm_bist__num_nvm_words
*/
pub const VL53L1_NVM_BIST__START_ADDRESS: u16 = 0x002B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'nvm_bist_config']

    fields: \n
        - [6:0] = nvm_bist__start_address
*/
pub const VL53L1_HOST_IF__STATUS: u16 = 0x002C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'system_status']

    fields: \n
        -   [0] = host_interface
*/
pub const VL53L1_PAD_I2C_HV__CONFIG: u16 = 0x002D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'gpio_config']

    fields: \n
        -   [0] = pad_scl_sda__vmodeint_hv
        -   [1] = i2c_pad__test_hv
        -   [2] = pad_scl__fpen_hv
        - [4:3] = pad_scl__progdel_hv
        -   [5] = pad_sda__fpen_hv
        - [7:6] = pad_sda__progdel_hv
*/
pub const VL53L1_PAD_I2C_HV__EXTSUP_CONFIG: u16 = 0x002E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'gpio_config']

    fields: \n
        -   [0] = pad_scl_sda__extsup_hv
*/
pub const VL53L1_GPIO_HV_PAD__CTRL: u16 = 0x002F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'gpio_config']

    fields: \n
        -   [0] = gpio__extsup_hv
        -   [1] = gpio__vmodeint_hv
*/
pub const VL53L1_GPIO_HV_MUX__CTRL: u16 = 0x0030;
/*
    type:     uint8_t \n
    default:  0x11 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'gpio_config']

    fields: \n
        - [3:0] = gpio__mux_select_hv
        -   [4] = gpio__mux_active_high_hv
*/
pub const VL53L1_GPIO__TIO_HV_STATUS: u16 = 0x0031;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'gpio_config']

    fields: \n
        -   [0] = gpio__tio_hv
        -   [1] = fresh_out_of_reset
*/
pub const VL53L1_GPIO__FIO_HV_STATUS: u16 = 0x0032;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  1
        - i2c_size =  1

    groups: \n
        ['static_config', 'gpio_config']

    fields: \n
        -   [1] = gpio__fio_hv
*/
pub const VL53L1_ANA_CONFIG__SPAD_SEL_PSWIDTH: u16 = 0x0033;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'analog_config']

    fields: \n
        - [2:0] = spad_sel_pswidth
*/
pub const VL53L1_ANA_CONFIG__VCSEL_PULSE_WIDTH_OFFSET: u16 = 0x0034;
/*
    type:     uint8_t \n
    default:  0x08 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'analog_config']

    fields: \n
        - [4:0] = vcsel_pulse_width_offset (fixed point 1.4)
*/
pub const VL53L1_ANA_CONFIG__FAST_OSC__CONFIG_CTRL: u16 = 0x0035;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'analog_config']

    fields: \n
        -   [0] = osc_config__latch_bypass
*/
pub const VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS: u16 = 0x0036;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = sigma_estimator__eff_pulse_width
*/
pub const VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS: u16 = 0x0037;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = sigma_estimator__eff_ambient_width
*/
pub const VL53L1_SIGMA_ESTIMATOR__SIGMA_REF_MM: u16 = 0x0038;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = sigma_estimator__sigma_ref
*/
pub const VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM: u16 = 0x0039;
/*
    type:     uint8_t \n
    default:  0x14 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = crosstalk_compensation_valid_height_mm
*/
pub const VL53L1_SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_0: u16 = 0x003A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = static_config_spare_0
*/
pub const VL53L1_SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_1: u16 = 0x003B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = static_config_spare_1
*/
pub const VL53L1_ALGO__RANGE_IGNORE_THRESHOLD_MCPS: u16 = 0x003C;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [15:0] = range_ignore_thresh_mcps (fixed point 3.13)
*/
pub const VL53L1_ALGO__RANGE_IGNORE_THRESHOLD_MCPS_HI: u16 = 0x003C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__RANGE_IGNORE_THRESHOLD_MCPS_LO: u16 = 0x003D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM: u16 = 0x003E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = range_ignore_height_mm
*/
pub const VL53L1_ALGO__RANGE_MIN_CLIP: u16 = 0x003F;
/*
    type:     uint8_t \n
    default:  0x8D \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        -   [0] = algo__range_min_clip_enable
        - [7:1] = algo__range_min_clip_value_mm
*/
pub const VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE: u16 = 0x0040;
/*
    type:     uint8_t \n
    default:  0x08 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [3:0] = consistency_check_tolerance (fixed point 1.3)
*/
pub const VL53L1_SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_2: u16 = 0x0041;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'algo_config']

    fields: \n
        - [7:0] = static_config_spare_2
*/
pub const VL53L1_SD_CONFIG__RESET_STAGES_MSB: u16 = 0x0042;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'sigmadelta_config']

    fields: \n
        - [3:0] = loop_init__clear_stage
*/
pub const VL53L1_SD_CONFIG__RESET_STAGES_LSB: u16 = 0x0043;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['static_config', 'sigmadelta_config']

    fields: \n
        - [7:4] = accum_reset__clear_stage
        - [3:0] = count_reset__clear_stage
*/
pub const VL53L1_GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE: u16 = 0x0044;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'roi_config']

    fields: \n
        - [7:0] = stream_count_update_value
*/
pub const VL53L1_GLOBAL_CONFIG__STREAM_DIVIDER: u16 = 0x0045;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'roi_config']

    fields: \n
        - [7:0] = stream_count_internal_div
*/
pub const VL53L1_SYSTEM__INTERRUPT_CONFIG_GPIO: u16 = 0x0046;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'gph_config']

    fields: \n
        - [1:0] = int_mode_distance
        - [3:2] = int_mode_rate
        -   [4] = int_spare
        -   [5] = int_new_measure_ready
        -   [6] = int_no_target_en
        -   [7] = int_combined_mode
*/
pub const VL53L1_CAL_CONFIG__VCSEL_START: u16 = 0x0047;
/*
    type:     uint8_t \n
    default:  0x0B \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'cal_config']

    fields: \n
        - [6:0] = cal_config__vcsel_start
*/
pub const VL53L1_CAL_CONFIG__REPEAT_RATE: u16 = 0x0048;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 11
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['general_config', 'cal_config']

    fields: \n
        - [11:0] = cal_config__repeat_rate
*/
pub const VL53L1_CAL_CONFIG__REPEAT_RATE_HI: u16 = 0x0048;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_CAL_CONFIG__REPEAT_RATE_LO: u16 = 0x0049;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GLOBAL_CONFIG__VCSEL_WIDTH: u16 = 0x004A;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'global_config']

    fields: \n
        - [6:0] = global_config__vcsel_width
*/
pub const VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP: u16 = 0x004B;
/*
    type:     uint8_t \n
    default:  0x04 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'phasecal_config']

    fields: \n
        - [7:0] = phasecal_config__timeout_macrop
*/
pub const VL53L1_PHASECAL_CONFIG__TARGET: u16 = 0x004C;
/*
    type:     uint8_t \n
    default:  0x21 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'phasecal_config']

    fields: \n
        - [7:0] = algo_phasecal_lim
*/
pub const VL53L1_PHASECAL_CONFIG__OVERRIDE: u16 = 0x004D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'phasecal_config']

    fields: \n
        -   [0] = phasecal_config__override
*/
pub const VL53L1_DSS_CONFIG__ROI_MODE_CONTROL: u16 = 0x004F;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'dss_config']

    fields: \n
        - [1:0] = dss_config__input_mode
        -   [2] = calculate_roi_enable
*/
pub const VL53L1_SYSTEM__THRESH_RATE_HIGH: u16 = 0x0050;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['general_config', 'gph_config']

    fields: \n
        - [15:0] = thresh_rate_high (fixed point 9.7)
*/
pub const VL53L1_SYSTEM__THRESH_RATE_HIGH_HI: u16 = 0x0050;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__THRESH_RATE_HIGH_LO: u16 = 0x0051;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__THRESH_RATE_LOW: u16 = 0x0052;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['general_config', 'gph_config']

    fields: \n
        - [15:0] = thresh_rate_low (fixed point 9.7)
*/
pub const VL53L1_SYSTEM__THRESH_RATE_LOW_HI: u16 = 0x0052;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__THRESH_RATE_LOW_LO: u16 = 0x0053;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT: u16 = 0x0054;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['general_config', 'dss_config']

    fields: \n
        - [15:0] = dss_config__manual_effective_spads_select
*/
pub const VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI: u16 = 0x0054;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO: u16 = 0x0055;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_CONFIG__MANUAL_BLOCK_SELECT: u16 = 0x0056;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'dss_config']

    fields: \n
        - [7:0] = dss_config__manual_block_select
*/
pub const VL53L1_DSS_CONFIG__APERTURE_ATTENUATION: u16 = 0x0057;
/*
    type:     uint8_t \n
    default:  0x33 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'dss_config']

    fields: \n
        - [7:0] = dss_config__aperture_attenuation
*/
pub const VL53L1_DSS_CONFIG__MAX_SPADS_LIMIT: u16 = 0x0058;
/*
    type:     uint8_t \n
    default:  0xFF \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'dss_config']

    fields: \n
        - [7:0] = dss_config__max_spads_limit
*/
pub const VL53L1_DSS_CONFIG__MIN_SPADS_LIMIT: u16 = 0x0059;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['general_config', 'dss_config']

    fields: \n
        - [7:0] = dss_config__min_spads_limit
*/
pub const VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_HI: u16 = 0x005A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'mm_config']

    fields: \n
        - [3:0] = mm_config__config_timeout_macrop_a_hi
*/
pub const VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_LO: u16 = 0x005B;
/*
    type:     uint8_t \n
    default:  0x06 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'mm_config']

    fields: \n
        - [7:0] = mm_config__config_timeout_macrop_a_lo
*/
pub const VL53L1_MM_CONFIG__TIMEOUT_MACROP_B_HI: u16 = 0x005C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'mm_config']

    fields: \n
        - [3:0] = mm_config__config_timeout_macrop_b_hi
*/
pub const VL53L1_MM_CONFIG__TIMEOUT_MACROP_B_LO: u16 = 0x005D;
/*
    type:     uint8_t \n
    default:  0x06 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'mm_config']

    fields: \n
        - [7:0] = mm_config__config_timeout_macrop_b_lo
*/
pub const VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI: u16 = 0x005E;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [3:0] = range_timeout_overall_periods_macrop_a_hi
*/
pub const VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_LO: u16 = 0x005F;
/*
    type:     uint8_t \n
    default:  0x92 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [7:0] = range_timeout_overall_periods_macrop_a_lo
*/
pub const VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A: u16 = 0x0060;
/*
    type:     uint8_t \n
    default:  0x0B \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [5:0] = range_config__vcsel_period_a
*/
pub const VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI: u16 = 0x0061;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [3:0] = range_timeout_overall_periods_macrop_b_hi
*/
pub const VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_LO: u16 = 0x0062;
/*
    type:     uint8_t \n
    default:  0x92 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [7:0] = range_timeout_overall_periods_macrop_b_lo
*/
pub const VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B: u16 = 0x0063;
/*
    type:     uint8_t \n
    default:  0x09 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [5:0] = range_config__vcsel_period_b
*/
pub const VL53L1_RANGE_CONFIG__SIGMA_THRESH: u16 = 0x0064;
/*
    type:     uint16_t \n
    default:  0x0080 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [15:0] = range_config__sigma_thresh (fixed point 14.2)
*/
pub const VL53L1_RANGE_CONFIG__SIGMA_THRESH_HI: u16 = 0x0064;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_CONFIG__SIGMA_THRESH_LO: u16 = 0x0065;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS: u16 = 0x0066;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [15:0] = range_config__min_count_rate_rtn_limit_mcps (fixed point 9.7)
*/
pub const VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI: u16 = 0x0066;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO: u16 = 0x0067;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_CONFIG__VALID_PHASE_LOW: u16 = 0x0068;
/*
    type:     uint8_t \n
    default:  0x08 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [7:0] = range_config__valid_phase_low (fixed point 5.3)
*/
pub const VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH: u16 = 0x0069;
/*
    type:     uint8_t \n
    default:  0x80 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'range_config']

    fields: \n
        - [7:0] = range_config__valid_phase_high (fixed point 5.3)
*/
pub const VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD: u16 = 0x006C;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['timing_config', 'system_config']

    fields: \n
        - [31:0] = intermeasurement_period
*/
pub const VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_3: u16 = 0x006C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_2: u16 = 0x006D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_1: u16 = 0x006E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_0: u16 = 0x006F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__FRACTIONAL_ENABLE: u16 = 0x0070;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['timing_config', 'system_config']

    fields: \n
        -   [0] = range_fractional_enable
*/
pub const VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0: u16 = 0x0071;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        -   [0] = grouped_parameter_hold
        -   [1] = grouped_parameter_hold_id
*/
pub const VL53L1_SYSTEM__THRESH_HIGH: u16 = 0x0072;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [15:0] = thresh_high
*/
pub const VL53L1_SYSTEM__THRESH_HIGH_HI: u16 = 0x0072;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__THRESH_HIGH_LO: u16 = 0x0073;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__THRESH_LOW: u16 = 0x0074;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [15:0] = thresh_low
*/
pub const VL53L1_SYSTEM__THRESH_LOW_HI: u16 = 0x0074;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__THRESH_LOW_LO: u16 = 0x0075;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SYSTEM__ENABLE_XTALK_PER_QUADRANT: u16 = 0x0076;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        -   [0] = system__enable_xtalk_per_quadrant
*/
pub const VL53L1_SYSTEM__SEED_CONFIG: u16 = 0x0077;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [1:0] = system__seed_config
        -   [2] = system__fw_pause_ctrl
*/
pub const VL53L1_SD_CONFIG__WOI_SD0: u16 = 0x0078;
/*
    type:     uint8_t \n
    default:  0x04 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [7:0] = sd_config__woi_sd0
*/
pub const VL53L1_SD_CONFIG__WOI_SD1: u16 = 0x0079;
/*
    type:     uint8_t \n
    default:  0x04 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [7:0] = sd_config__woi_sd1
*/
pub const VL53L1_SD_CONFIG__INITIAL_PHASE_SD0: u16 = 0x007A;
/*
    type:     uint8_t \n
    default:  0x03 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [6:0] = sd_config__initial_phase_sd0
*/
pub const VL53L1_SD_CONFIG__INITIAL_PHASE_SD1: u16 = 0x007B;
/*
    type:     uint8_t \n
    default:  0x03 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [6:0] = sd_config__initial_phase_sd1
*/
pub const VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1: u16 = 0x007C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        -   [0] = grouped_parameter_hold
        -   [1] = grouped_parameter_hold_id
*/
pub const VL53L1_SD_CONFIG__FIRST_ORDER_SELECT: u16 = 0x007D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        -   [0] = sd_config__first_order_select_rtn
        -   [1] = sd_config__first_order_select_ref
*/
pub const VL53L1_SD_CONFIG__QUANTIFIER: u16 = 0x007E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [3:0] = sd_config__quantifier
*/
pub const VL53L1_ROI_CONFIG__USER_ROI_CENTRE_SPAD: u16 = 0x007F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [7:0] = user_roi_center_spad
*/
pub const VL53L1_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE: u16 = 0x0080;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        - [7:0] = roi_config__user_roi_requested_global_xy_size
*/
pub const VL53L1_SYSTEM__SEQUENCE_CONFIG: u16 = 0x0081;
/*
    type:     uint8_t \n
    default:  0xFF \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

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
pub const VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD: u16 = 0x0082;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['dynamic_config', 'gph_config']

    fields: \n
        -   [0] = grouped_parameter_hold
        -   [1] = grouped_parameter_hold_id
*/
pub const VL53L1_POWER_MANAGEMENT__GO1_POWER_FORCE: u16 = 0x0083;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_control', 'pwrman_ctrl']

    fields: \n
        -   [0] = go1_dig_powerforce
*/
pub const VL53L1_SYSTEM__STREAM_COUNT_CTRL: u16 = 0x0084;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_control', 'stream_ctrl']

    fields: \n
        -   [0] = retain_stream_count
*/
pub const VL53L1_FIRMWARE__ENABLE: u16 = 0x0085;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_control', 'firmware_ctrl']

    fields: \n
        -   [0] = firmware_enable
*/
pub const VL53L1_SYSTEM__INTERRUPT_CLEAR: u16 = 0x0086;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_control', 'system_int_clr']

    fields: \n
        -   [0] = sys_interrupt_clear_range
        -   [1] = sys_interrupt_clear_error
*/
pub const VL53L1_SYSTEM__MODE_START: u16 = 0x0087;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_control', 'system_start']

    fields: \n
        - [1:0] = scheduler_mode
        - [3:2] = readout_mode
        -   [4] = mode_range__single_shot
        -   [5] = mode_range__back_to_back
        -   [6] = mode_range__timed
        -   [7] = mode_range__abort
*/
pub const VL53L1_RESULT__INTERRUPT_STATUS: u16 = 0x0088;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [2:0] = int_status
        - [4:3] = int_error_status
        -   [5] = gph_id_gpio_status
*/
pub const VL53L1_RESULT__RANGE_STATUS: u16 = 0x0089;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [4:0] = range_status
        -   [5] = max_threshold_hit
        -   [6] = min_threshold_hit
        -   [7] = gph_id_range_status
*/
pub const VL53L1_RESULT__REPORT_STATUS: u16 = 0x008A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [3:0] = report_status
*/
pub const VL53L1_RESULT__STREAM_COUNT: u16 = 0x008B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [7:0] = result__stream_count
*/
pub const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x008C;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__dss_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x008C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x008D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0: u16 = 0x008E;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__peak_signal_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI: u16 = 0x008E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO: u16 = 0x008F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0: u16 = 0x0090;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__ambient_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0090;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0091;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SIGMA_SD0: u16 = 0x0092;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__sigma_sd0 (fixed point 14.2)
*/
pub const VL53L1_RESULT__SIGMA_SD0_HI: u16 = 0x0092;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SIGMA_SD0_LO: u16 = 0x0093;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PHASE_SD0: u16 = 0x0094;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__phase_sd0 (fixed point 5.11)
*/
pub const VL53L1_RESULT__PHASE_SD0_HI: u16 = 0x0094;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PHASE_SD0_LO: u16 = 0x0095;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: u16 = 0x0096;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__final_crosstalk_corrected_range_mm_sd0
*/
pub const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI: u16 = 0x0096;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO: u16 = 0x0097;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0: u16 = 0x0098;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI: u16 = 0x0098;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO: u16 = 0x0099;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x009A;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__mm_inner_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x009A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x009B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x009C;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__mm_outer_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x009C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x009D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0: u16 = 0x009E;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__avg_signal_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI: u16 = 0x009E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO: u16 = 0x009F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1: u16 = 0x00A0;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__dss_actual_effective_spads_sd1 (fixed point 8.8)
*/
pub const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI: u16 = 0x00A0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO: u16 = 0x00A1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1: u16 = 0x00A2;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__peak_signal_count_rate_mcps_sd1 (fixed point 9.7)
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI: u16 = 0x00A2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO: u16 = 0x00A3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1: u16 = 0x00A4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__ambient_count_rate_mcps_sd1 (fixed point 9.7)
*/
pub const VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI: u16 = 0x00A4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO: u16 = 0x00A5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SIGMA_SD1: u16 = 0x00A6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__sigma_sd1 (fixed point 14.2)
*/
pub const VL53L1_RESULT__SIGMA_SD1_HI: u16 = 0x00A6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SIGMA_SD1_LO: u16 = 0x00A7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PHASE_SD1: u16 = 0x00A8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__phase_sd1 (fixed point 5.11)
*/
pub const VL53L1_RESULT__PHASE_SD1_HI: u16 = 0x00A8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__PHASE_SD1_LO: u16 = 0x00A9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1: u16 = 0x00AA;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__final_crosstalk_corrected_range_mm_sd1
*/
pub const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI: u16 = 0x00AA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO: u16 = 0x00AB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_0_SD1: u16 = 0x00AC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__spare_0_sd1
*/
pub const VL53L1_RESULT__SPARE_0_SD1_HI: u16 = 0x00AC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_0_SD1_LO: u16 = 0x00AD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_1_SD1: u16 = 0x00AE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__spare_1_sd1
*/
pub const VL53L1_RESULT__SPARE_1_SD1_HI: u16 = 0x00AE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_1_SD1_LO: u16 = 0x00AF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_2_SD1: u16 = 0x00B0;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [15:0] = result__spare_2_sd1
*/
pub const VL53L1_RESULT__SPARE_2_SD1_HI: u16 = 0x00B0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_2_SD1_LO: u16 = 0x00B1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__SPARE_3_SD1: u16 = 0x00B2;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [7:0] = result__spare_3_sd1
*/
pub const VL53L1_RESULT__THRESH_INFO: u16 = 0x00B3;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['system_results', 'results']

    fields: \n
        - [3:0] = result__distance_int_info
        - [7:4] = result__rate_int_info
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0: u16 = 0x00B4;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__ambient_window_events_sd0
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3: u16 = 0x00B4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2: u16 = 0x00B5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1: u16 = 0x00B6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0: u16 = 0x00B7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0: u16 = 0x00B8;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__ranging_total_events_sd0
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3: u16 = 0x00B8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2: u16 = 0x00B9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1: u16 = 0x00BA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0: u16 = 0x00BB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0: u16 = 0x00BC;
/*
    type:     int32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__signal_total_events_sd0
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3: u16 = 0x00BC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2: u16 = 0x00BD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1: u16 = 0x00BE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0: u16 = 0x00BF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0: u16 = 0x00C0;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__total_periods_elapsed_sd0
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3: u16 = 0x00C0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2: u16 = 0x00C1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1: u16 = 0x00C2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0: u16 = 0x00C3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1: u16 = 0x00C4;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__ambient_window_events_sd1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3: u16 = 0x00C4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2: u16 = 0x00C5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1: u16 = 0x00C6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0: u16 = 0x00C7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1: u16 = 0x00C8;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__ranging_total_events_sd1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3: u16 = 0x00C8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2: u16 = 0x00C9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1: u16 = 0x00CA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0: u16 = 0x00CB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1: u16 = 0x00CC;
/*
    type:     int32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__signal_total_events_sd1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3: u16 = 0x00CC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2: u16 = 0x00CD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1: u16 = 0x00CE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0: u16 = 0x00CF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1: u16 = 0x00D0;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = result_core__total_periods_elapsed_sd1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3: u16 = 0x00D0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2: u16 = 0x00D1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1: u16 = 0x00D2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0: u16 = 0x00D3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT_CORE__SPARE_0: u16 = 0x00D4;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['core_results', 'ranging_core_results']

    fields: \n
        - [7:0] = result_core__spare_0
*/
pub const VL53L1_PHASECAL_RESULT__REFERENCE_PHASE: u16 = 0x00D6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['debug_results', 'phasecal_results']

    fields: \n
        - [15:0] = result_phasecal__reference_phase (fixed point 5.11)
*/
pub const VL53L1_PHASECAL_RESULT__REFERENCE_PHASE_HI: u16 = 0x00D6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PHASECAL_RESULT__REFERENCE_PHASE_LO: u16 = 0x00D7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PHASECAL_RESULT__VCSEL_START: u16 = 0x00D8;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'phasecal_results']

    fields: \n
        - [6:0] = result_phasecal__vcsel_start
*/
pub const VL53L1_REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS: u16 = 0x00D9;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'ref_spad_status']

    fields: \n
        - [5:0] = ref_spad_char_result__num_actual_ref_spads
*/
pub const VL53L1_REF_SPAD_CHAR_RESULT__REF_LOCATION: u16 = 0x00DA;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'ref_spad_status']

    fields: \n
        - [1:0] = ref_spad_char_result__ref_location
*/
pub const VL53L1_VHV_RESULT__COLDBOOT_STATUS: u16 = 0x00DB;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'vhv_results']

    fields: \n
        -   [0] = vhv_result__coldboot_status
*/
pub const VL53L1_VHV_RESULT__SEARCH_RESULT: u16 = 0x00DC;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'vhv_results']

    fields: \n
        - [5:0] = cp_sel_result
*/
pub const VL53L1_VHV_RESULT__LATEST_SETTING: u16 = 0x00DD;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'vhv_results']

    fields: \n
        - [5:0] = cp_sel_latest_setting
*/
pub const VL53L1_RESULT__OSC_CALIBRATE_VAL: u16 = 0x00DE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb =  9
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['debug_results', 'misc_results']

    fields: \n
        - [9:0] = osc_calibrate_val
*/
pub const VL53L1_RESULT__OSC_CALIBRATE_VAL_HI: u16 = 0x00DE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RESULT__OSC_CALIBRATE_VAL_LO: u16 = 0x00DF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ANA_CONFIG__POWERDOWN_GO1: u16 = 0x00E0;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'analog_config']

    fields: \n
        -   [0] = go2_ref_bg_disable_avdd
        -   [1] = go2_regdvdd1v2_enable_avdd
*/
pub const VL53L1_ANA_CONFIG__REF_BG_CTRL: u16 = 0x00E1;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'analog_config']

    fields: \n
        -   [0] = go2_ref_overdrvbg_avdd
        -   [1] = go2_ref_forcebgison_avdd
*/
pub const VL53L1_ANA_CONFIG__REGDVDD1V2_CTRL: u16 = 0x00E2;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'analog_config']

    fields: \n
        -   [0] = go2_regdvdd1v2_sel_pulldown_avdd
        -   [1] = go2_regdvdd1v2_sel_boost_avdd
        - [3:2] = go2_regdvdd1v2_selv_avdd
*/
pub const VL53L1_ANA_CONFIG__OSC_SLOW_CTRL: u16 = 0x00E3;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'analog_config']

    fields: \n
        -   [0] = osc_slow_en
        -   [1] = osc_slow_op_en
        -   [2] = osc_slow_freq_sel
*/
pub const VL53L1_TEST_MODE__STATUS: u16 = 0x00E4;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'test_mode_status']

    fields: \n
        -   [0] = test_mode_status
*/
pub const VL53L1_FIRMWARE__SYSTEM_STATUS: u16 = 0x00E5;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'firmware_status']

    fields: \n
        -   [0] = firmware_bootup
        -   [1] = firmware_first_range
*/
pub const VL53L1_FIRMWARE__MODE_STATUS: u16 = 0x00E6;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'firmware_status']

    fields: \n
        - [7:0] = firmware_mode_status
*/
pub const VL53L1_FIRMWARE__SECONDARY_MODE_STATUS: u16 = 0x00E7;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'firmware_status']

    fields: \n
        - [7:0] = fw_secondary_mode_status
*/
pub const VL53L1_FIRMWARE__CAL_REPEAT_RATE_COUNTER: u16 = 0x00E8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 11
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['debug_results', 'firmware_status']

    fields: \n
        - [11:0] = firmware_cal_repeat_rate
*/
pub const VL53L1_FIRMWARE__CAL_REPEAT_RATE_COUNTER_HI: u16 = 0x00E8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_FIRMWARE__CAL_REPEAT_RATE_COUNTER_LO: u16 = 0x00E9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_FIRMWARE__HISTOGRAM_BIN: u16 = 0x00EA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_HIGH: u16 = 0x00EC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [15:0] = shadow_thresh_high
*/
pub const VL53L1_GPH__SYSTEM__THRESH_HIGH_HI: u16 = 0x00EC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_HIGH_LO: u16 = 0x00ED;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_LOW: u16 = 0x00EE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [15:0] = shadow_thresh_low
*/
pub const VL53L1_GPH__SYSTEM__THRESH_LOW_HI: u16 = 0x00EE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_LOW_LO: u16 = 0x00EF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__ENABLE_XTALK_PER_QUADRANT: u16 = 0x00F0;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        -   [0] = shadow__enable_xtalk_per_quadrant
*/
pub const VL53L1_GPH__SPARE_0: u16 = 0x00F1;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        -   [0] = fw_safe_to_disable
        -   [1] = shadow__spare_0
        -   [2] = shadow__spare_1
*/
pub const VL53L1_GPH__SD_CONFIG__WOI_SD0: u16 = 0x00F2;
/*
    type:     uint8_t \n
    default:  0x04 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [7:0] = shadow_sd_config__woi_sd0
*/
pub const VL53L1_GPH__SD_CONFIG__WOI_SD1: u16 = 0x00F3;
/*
    type:     uint8_t \n
    default:  0x04 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [7:0] = shadow_sd_config__woi_sd1
*/
pub const VL53L1_GPH__SD_CONFIG__INITIAL_PHASE_SD0: u16 = 0x00F4;
/*
    type:     uint8_t \n
    default:  0x03 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [6:0] = shadow_sd_config__initial_phase_sd0
*/
pub const VL53L1_GPH__SD_CONFIG__INITIAL_PHASE_SD1: u16 = 0x00F5;
/*
    type:     uint8_t \n
    default:  0x03 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [6:0] = shadow_sd_config__initial_phase_sd1
*/
pub const VL53L1_GPH__SD_CONFIG__FIRST_ORDER_SELECT: u16 = 0x00F6;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        -   [0] = shadow_sd_config__first_order_select_rtn
        -   [1] = shadow_sd_config__first_order_select_ref
*/
pub const VL53L1_GPH__SD_CONFIG__QUANTIFIER: u16 = 0x00F7;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [3:0] = shadow_sd_config__quantifier
*/
pub const VL53L1_GPH__ROI_CONFIG__USER_ROI_CENTRE_SPAD: u16 = 0x00F8;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [7:0] = shadow_user_roi_center_spad_q0
*/
pub const VL53L1_GPH__ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE: u16 = 0x00F9;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        - [7:0] = shadow_user_roi_requested_global_xy_size
*/
pub const VL53L1_GPH__SYSTEM__SEQUENCE_CONFIG: u16 = 0x00FA;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

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
pub const VL53L1_GPH__GPH_ID: u16 = 0x00FB;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'gph_actual']

    fields: \n
        -   [0] = shadow_gph_id
*/
pub const VL53L1_SYSTEM__INTERRUPT_SET: u16 = 0x00FC;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'system_int_set']

    fields: \n
        -   [0] = sys_interrupt_set_range
        -   [1] = sys_interrupt_set_error
*/
pub const VL53L1_INTERRUPT_MANAGER__ENABLES: u16 = 0x00FD;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'interrupt_manager']

    fields: \n
        -   [0] = interrupt_enable__single_shot
        -   [1] = interrupt_enable__back_to_back
        -   [2] = interrupt_enable__timed
        -   [3] = interrupt_enable__abort
        -   [4] = interrupt_enable__test
*/
pub const VL53L1_INTERRUPT_MANAGER__CLEAR: u16 = 0x00FE;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'interrupt_manager']

    fields: \n
        -   [0] = interrupt_clear__single_shot
        -   [1] = interrupt_clear__back_to_back
        -   [2] = interrupt_clear__timed
        -   [3] = interrupt_clear__abort
        -   [4] = interrupt_clear__test
*/
pub const VL53L1_INTERRUPT_MANAGER__STATUS: u16 = 0x00FF;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'interrupt_manager']

    fields: \n
        -   [0] = interrupt_status__single_shot
        -   [1] = interrupt_status__back_to_back
        -   [2] = interrupt_status__timed
        -   [3] = interrupt_status__abort
        -   [4] = interrupt_status__test
*/
pub const VL53L1_MCU_TO_HOST_BANK__WR_ACCESS_EN: u16 = 0x0100;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'host_bank_ctrl']

    fields: \n
        -   [0] = mcu_to_host_bank_wr_en
*/
pub const VL53L1_POWER_MANAGEMENT__GO1_RESET_STATUS: u16 = 0x0101;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'power_man_status']

    fields: \n
        -   [0] = go1_status
*/
pub const VL53L1_PAD_STARTUP_MODE__VALUE_RO: u16 = 0x0102;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'pad_config']

    fields: \n
        -   [0] = pad_atest1_val_ro
        -   [1] = pad_atest2_val_ro
*/
pub const VL53L1_PAD_STARTUP_MODE__VALUE_CTRL: u16 = 0x0103;
/*
    type:     uint8_t \n
    default:  0x30 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'pad_config']

    fields: \n
        -   [0] = pad_atest1_val
        -   [1] = pad_atest2_val
        -   [4] = pad_atest1_dig_enable
        -   [5] = pad_atest2_dig_enable
*/
pub const VL53L1_PLL_PERIOD_US: u16 = 0x0104;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 17
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['debug_results', 'pll_config']

    fields: \n
        - [17:0] = pll_period_us (fixed point 0.24)
*/
pub const VL53L1_PLL_PERIOD_US_3: u16 = 0x0104;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PLL_PERIOD_US_2: u16 = 0x0105;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PLL_PERIOD_US_1: u16 = 0x0106;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PLL_PERIOD_US_0: u16 = 0x0107;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_INTERRUPT_SCHEDULER__DATA_OUT: u16 = 0x0108;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['debug_results', 'debug_timer']

    fields: \n
        - [31:0] = interrupt_scheduler_data_out
*/
pub const VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_3: u16 = 0x0108;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_2: u16 = 0x0109;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_1: u16 = 0x010A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_0: u16 = 0x010B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_NVM_BIST__COMPLETE: u16 = 0x010C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'nvm_bist_status']

    fields: \n
        -   [0] = nvm_bist__complete
*/
pub const VL53L1_NVM_BIST__STATUS: u16 = 0x010D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['debug_results', 'nvm_bist_status']

    fields: \n
        -   [0] = nvm_bist__status
*/
pub const VL53L1_IDENTIFICATION__MODEL_ID: u16 = 0x010F;
/*
    type:     uint8_t \n
    default:  0xEA \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'identification']

    fields: \n
        - [7:0] = model_id
*/
pub const VL53L1_IDENTIFICATION__MODULE_TYPE: u16 = 0x0110;
/*
    type:     uint8_t \n
    default:  0xAA \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'identification']

    fields: \n
        - [7:0] = module_type
*/
pub const VL53L1_IDENTIFICATION__REVISION_ID: u16 = 0x0111;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'identification']

    fields: \n
        - [3:0] = nvm_revision_id
        - [7:4] = mask_revision_id
*/
pub const VL53L1_IDENTIFICATION__MODULE_ID: u16 = 0x0112;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['nvm_copy_data', 'identification']

    fields: \n
        - [15:0] = module_id
*/
pub const VL53L1_IDENTIFICATION__MODULE_ID_HI: u16 = 0x0112;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_IDENTIFICATION__MODULE_ID_LO: u16 = 0x0113;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_ANA_CONFIG__FAST_OSC__TRIM_MAX: u16 = 0x0114;
/*
    type:     uint8_t \n
    default:  OSC_TRIM_DEFAULT \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'analog_config']

    fields: \n
        - [6:0] = osc_trim_max
*/
pub const VL53L1_ANA_CONFIG__FAST_OSC__FREQ_SET: u16 = 0x0115;
/*
    type:     uint8_t \n
    default:  OSC_FREQ_SET_DEFAULT \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'analog_config']

    fields: \n
        - [2:0] = osc_freq_set
*/
pub const VL53L1_ANA_CONFIG__VCSEL_TRIM: u16 = 0x0116;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'analog_config']

    fields: \n
        - [2:0] = vcsel_trim
*/
pub const VL53L1_ANA_CONFIG__VCSEL_SELION: u16 = 0x0117;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'analog_config']

    fields: \n
        - [5:0] = vcsel_selion
*/
pub const VL53L1_ANA_CONFIG__VCSEL_SELION_MAX: u16 = 0x0118;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'analog_config']

    fields: \n
        - [5:0] = vcsel_selion_max
*/
pub const VL53L1_PROTECTED_LASER_SAFETY__LOCK_BIT: u16 = 0x0119;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'laser_safety']

    fields: \n
        -   [0] = laser_safety__lock_bit
*/
pub const VL53L1_LASER_SAFETY__KEY: u16 = 0x011A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  6
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'laser_safety']

    fields: \n
        - [6:0] = laser_safety__key
*/
pub const VL53L1_LASER_SAFETY__KEY_RO: u16 = 0x011B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'laser_safety']

    fields: \n
        -   [0] = laser_safety__key_ro
*/
pub const VL53L1_LASER_SAFETY__CLIP: u16 = 0x011C;
/*
    type:     uint8_t \n
    default:  0x02 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'laser_safety']

    fields: \n
        - [5:0] = vcsel_pulse_width_clip
*/
pub const VL53L1_LASER_SAFETY__MULT: u16 = 0x011D;
/*
    type:     uint8_t \n
    default:  0x32 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'laser_safety']

    fields: \n
        - [5:0] = vcsel_pulse_width_mult
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_0: u16 = 0x011E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_0
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_1: u16 = 0x011F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_1
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_2: u16 = 0x0120;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_2
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_3: u16 = 0x0121;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_3
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_4: u16 = 0x0122;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_4
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_5: u16 = 0x0123;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_5
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_6: u16 = 0x0124;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_6
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_7: u16 = 0x0125;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_7
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_8: u16 = 0x0126;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_8
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_9: u16 = 0x0127;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_9
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_10: u16 = 0x0128;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_10
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_11: u16 = 0x0129;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_11
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_12: u16 = 0x012A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_12
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_13: u16 = 0x012B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_13
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_14: u16 = 0x012C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_14
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_15: u16 = 0x012D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_15
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_16: u16 = 0x012E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_16
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_17: u16 = 0x012F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_17
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_18: u16 = 0x0130;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_18
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_19: u16 = 0x0131;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_19
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_20: u16 = 0x0132;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_20
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_21: u16 = 0x0133;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_21
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_22: u16 = 0x0134;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_22
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_23: u16 = 0x0135;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_23
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_24: u16 = 0x0136;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_24
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_25: u16 = 0x0137;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_25
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_26: u16 = 0x0138;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_26
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_27: u16 = 0x0139;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_27
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_28: u16 = 0x013A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_28
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_29: u16 = 0x013B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_29
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_30: u16 = 0x013C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_30
*/
pub const VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_31: u16 = 0x013D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'ret_spad_config']

    fields: \n
        - [7:0] = spad_enables_rtn_31
*/
pub const VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD: u16 = 0x013E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'roi_config']

    fields: \n
        - [7:0] = mode_roi_center_spad
*/
pub const VL53L1_ROI_CONFIG__MODE_ROI_XY_SIZE: u16 = 0x013F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['nvm_copy_data', 'roi_config']

    fields: \n
        - [7:0] = mode_roi_xy_size
*/
pub const VL53L1_GO2_HOST_BANK_ACCESS__OVERRIDE: u16 = 0x0300;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND: u16 = 0x0400;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_3: u16 = 0x0400;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_2: u16 = 0x0401;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_1: u16 = 0x0402;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_0: u16 = 0x0403;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER: u16 = 0x0404;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_3: u16 = 0x0404;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_2: u16 = 0x0405;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_1: u16 = 0x0406;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_0: u16 = 0x0407;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI: u16 = 0x0408;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_3: u16 = 0x0408;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_2: u16 = 0x0409;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_1: u16 = 0x040A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_0: u16 = 0x040B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO: u16 = 0x040C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_3: u16 = 0x040C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_2: u16 = 0x040D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_1: u16 = 0x040E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_0: u16 = 0x040F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__START: u16 = 0x0410;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_MULTIPLIER__STATUS: u16 = 0x0411;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__START: u16 = 0x0412;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__STATUS: u16 = 0x0413;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVIDEND: u16 = 0x0414;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_3: u16 = 0x0414;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_2: u16 = 0x0415;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_1: u16 = 0x0416;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_0: u16 = 0x0417;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVISOR: u16 = 0x0418;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVISOR_3: u16 = 0x0418;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVISOR_2: u16 = 0x0419;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVISOR_1: u16 = 0x041A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__DIVISOR_0: u16 = 0x041B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__QUOTIENT: u16 = 0x041C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_3: u16 = 0x041C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_2: u16 = 0x041D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_1: u16 = 0x041E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_0: u16 = 0x041F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER0__VALUE_IN: u16 = 0x0420;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER0__VALUE_IN_3: u16 = 0x0420;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER0__VALUE_IN_2: u16 = 0x0421;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER0__VALUE_IN_1: u16 = 0x0422;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER0__VALUE_IN_0: u16 = 0x0423;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER1__VALUE_IN: u16 = 0x0424;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER1__VALUE_IN_3: u16 = 0x0424;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER1__VALUE_IN_2: u16 = 0x0425;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER1__VALUE_IN_1: u16 = 0x0426;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER1__VALUE_IN_0: u16 = 0x0427;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER0__CTRL: u16 = 0x0428;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TIMER1__CTRL: u16 = 0x0429;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_GENERAL_PURPOSE__GP_0: u16 = 0x042C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_gp_0
*/
pub const VL53L1_MCU_GENERAL_PURPOSE__GP_1: u16 = 0x042D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_gp_1
*/
pub const VL53L1_MCU_GENERAL_PURPOSE__GP_2: u16 = 0x042E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_gp_2
*/
pub const VL53L1_MCU_GENERAL_PURPOSE__GP_3: u16 = 0x042F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_gp_3
*/
pub const VL53L1_MCU_RANGE_CALC__CONFIG: u16 = 0x0430;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = fw_calc__sigma_delta_sel
        -   [2] = fw_calc__phase_output_en
        -   [3] = fw_calc__peak_signal_rate_en
        -   [4] = fw_calc__ambient_rate_en
        -   [5] = fw_calc__total_rate_per_spad_en
        -   [6] = fw_calc__snr_avg_signal_rate_en
        -   [7] = fw_calc__sigma_en
*/
pub const VL53L1_MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE: u16 = 0x0432;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = offset_corrected_range
*/
pub const VL53L1_MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_HI: u16 = 0x0432;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_LO: u16 = 0x0433;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_4: u16 = 0x0434;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 16
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['']

    fields: \n
        - [16:0] = mcu_calc__spare_4
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_4_3: u16 = 0x0434;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_4_2: u16 = 0x0435;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_4_1: u16 = 0x0436;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_4_0: u16 = 0x0437;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC: u16 = 0x0438;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 13
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [13:0] = ambient_duration_prec_calc
*/
pub const VL53L1_MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_HI: u16 = 0x0438;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_LO: u16 = 0x0439;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_VCSEL_PERIOD: u16 = 0x043C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = algo_vcsel_period
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_5: u16 = 0x043D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_5
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_TOTAL_PERIODS: u16 = 0x043E;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = algo_total_periods
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_HI: u16 = 0x043E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_LO: u16 = 0x043F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE: u16 = 0x0440;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['']

    fields: \n
        - [31:0] = algo_accum_phase
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_3: u16 = 0x0440;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_2: u16 = 0x0441;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_1: u16 = 0x0442;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_0: u16 = 0x0443;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS: u16 = 0x0444;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['']

    fields: \n
        - [31:0] = algo_signal_events
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_3: u16 = 0x0444;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_2: u16 = 0x0445;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_1: u16 = 0x0446;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_0: u16 = 0x0447;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS: u16 = 0x0448;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['']

    fields: \n
        - [31:0] = algo_ambient_events
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_3: u16 = 0x0448;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_2: u16 = 0x0449;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_1: u16 = 0x044A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_0: u16 = 0x044B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_6: u16 = 0x044C;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = mcu_calc__spare_6
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_6_HI: u16 = 0x044C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_6_LO: u16 = 0x044D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD: u16 = 0x044E;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = algo_adjust_vcsel_period
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_HI: u16 = 0x044E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_LO: u16 = 0x044F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__NUM_SPADS: u16 = 0x0450;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = num_spads
*/
pub const VL53L1_MCU_RANGE_CALC__NUM_SPADS_HI: u16 = 0x0450;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__NUM_SPADS_LO: u16 = 0x0451;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__PHASE_OUTPUT: u16 = 0x0452;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = phase_output
*/
pub const VL53L1_MCU_RANGE_CALC__PHASE_OUTPUT_HI: u16 = 0x0452;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__PHASE_OUTPUT_LO: u16 = 0x0453;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS: u16 = 0x0454;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 19
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['']

    fields: \n
        - [19:0] = rate_per_spad_mcps
*/
pub const VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_3: u16 = 0x0454;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_2: u16 = 0x0455;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_1: u16 = 0x0456;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_0: u16 = 0x0457;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_7: u16 = 0x0458;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_7
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_8: u16 = 0x0459;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_8
*/
pub const VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS: u16 = 0x045A;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = peak_signal_rate
*/
pub const VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_HI: u16 = 0x045A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_LO: u16 = 0x045B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS: u16 = 0x045C;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = avg_signal_rate
*/
pub const VL53L1_MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_HI: u16 = 0x045C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_LO: u16 = 0x045D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__AMBIENT_RATE_MCPS: u16 = 0x045E;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = ambient_rate
*/
pub const VL53L1_MCU_RANGE_CALC__AMBIENT_RATE_MCPS_HI: u16 = 0x045E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__AMBIENT_RATE_MCPS_LO: u16 = 0x045F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__XTALK: u16 = 0x0460;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = crosstalk (fixed point 9.7)
*/
pub const VL53L1_MCU_RANGE_CALC__XTALK_HI: u16 = 0x0460;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__XTALK_LO: u16 = 0x0461;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__CALC_STATUS: u16 = 0x0462;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = calc_status
*/
pub const VL53L1_MCU_RANGE_CALC__DEBUG: u16 = 0x0463;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = calc_debug__divide_by_zero
*/
pub const VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS: u16 = 0x0464;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['']

    fields: \n
        - [15:0] = peak_signal_rate_xtalk_corr
*/
pub const VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_HI: u16 = 0x0464;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_LO: u16 = 0x0465;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_0: u16 = 0x0468;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_0
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_1: u16 = 0x0469;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_1
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_2: u16 = 0x046A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_2
*/
pub const VL53L1_MCU_RANGE_CALC__SPARE_3: u16 = 0x046B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [7:0] = mcu_calc__spare_3
*/
pub const VL53L1_PATCH__CTRL: u16 = 0x0470;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__JMP_ENABLES: u16 = 0x0472;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__JMP_ENABLES_HI: u16 = 0x0472;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__JMP_ENABLES_LO: u16 = 0x0473;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__DATA_ENABLES: u16 = 0x0474;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__DATA_ENABLES_HI: u16 = 0x0474;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__DATA_ENABLES_LO: u16 = 0x0475;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_0: u16 = 0x0476;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_0_HI: u16 = 0x0476;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_0_LO: u16 = 0x0477;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_1: u16 = 0x0478;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_1_HI: u16 = 0x0478;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_1_LO: u16 = 0x0479;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_2: u16 = 0x047A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_2_HI: u16 = 0x047A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_2_LO: u16 = 0x047B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_3: u16 = 0x047C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_3_HI: u16 = 0x047C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_3_LO: u16 = 0x047D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_4: u16 = 0x047E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_4_HI: u16 = 0x047E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_4_LO: u16 = 0x047F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_5: u16 = 0x0480;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_5_HI: u16 = 0x0480;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_5_LO: u16 = 0x0481;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_6: u16 = 0x0482;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_6_HI: u16 = 0x0482;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_6_LO: u16 = 0x0483;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_7: u16 = 0x0484;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_7_HI: u16 = 0x0484;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_7_LO: u16 = 0x0485;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_8: u16 = 0x0486;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_8_HI: u16 = 0x0486;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_8_LO: u16 = 0x0487;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_9: u16 = 0x0488;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_9_HI: u16 = 0x0488;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_9_LO: u16 = 0x0489;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_10: u16 = 0x048A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_10_HI: u16 = 0x048A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_10_LO: u16 = 0x048B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_11: u16 = 0x048C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_11_HI: u16 = 0x048C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_11_LO: u16 = 0x048D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_12: u16 = 0x048E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_12_HI: u16 = 0x048E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_12_LO: u16 = 0x048F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_13: u16 = 0x0490;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_13_HI: u16 = 0x0490;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_13_LO: u16 = 0x0491;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_14: u16 = 0x0492;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_14_HI: u16 = 0x0492;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_14_LO: u16 = 0x0493;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_15: u16 = 0x0494;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_15_HI: u16 = 0x0494;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__OFFSET_15_LO: u16 = 0x0495;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_0: u16 = 0x0496;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_0_HI: u16 = 0x0496;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_0_LO: u16 = 0x0497;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_1: u16 = 0x0498;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_1_HI: u16 = 0x0498;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_1_LO: u16 = 0x0499;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_2: u16 = 0x049A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_2_HI: u16 = 0x049A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_2_LO: u16 = 0x049B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_3: u16 = 0x049C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_3_HI: u16 = 0x049C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_3_LO: u16 = 0x049D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_4: u16 = 0x049E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_4_HI: u16 = 0x049E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_4_LO: u16 = 0x049F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_5: u16 = 0x04A0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_5_HI: u16 = 0x04A0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_5_LO: u16 = 0x04A1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_6: u16 = 0x04A2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_6_HI: u16 = 0x04A2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_6_LO: u16 = 0x04A3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_7: u16 = 0x04A4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_7_HI: u16 = 0x04A4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_7_LO: u16 = 0x04A5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_8: u16 = 0x04A6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_8_HI: u16 = 0x04A6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_8_LO: u16 = 0x04A7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_9: u16 = 0x04A8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_9_HI: u16 = 0x04A8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_9_LO: u16 = 0x04A9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_10: u16 = 0x04AA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_10_HI: u16 = 0x04AA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_10_LO: u16 = 0x04AB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_11: u16 = 0x04AC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_11_HI: u16 = 0x04AC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_11_LO: u16 = 0x04AD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_12: u16 = 0x04AE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_12_HI: u16 = 0x04AE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_12_LO: u16 = 0x04AF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_13: u16 = 0x04B0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_13_HI: u16 = 0x04B0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_13_LO: u16 = 0x04B1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_14: u16 = 0x04B2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_14_HI: u16 = 0x04B2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_14_LO: u16 = 0x04B3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_15: u16 = 0x04B4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_15_HI: u16 = 0x04B4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PATCH__ADDRESS_15_LO: u16 = 0x04B5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SPI_ASYNC_MUX__CTRL: u16 = 0x04C0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_CLK__CONFIG: u16 = 0x04C4;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = clk_mcu_en
*/
pub const VL53L1_GPIO_LV_MUX__CTRL: u16 = 0x04CC;
/*
    type:     uint8_t \n
    default:  0x08 \n
    info: \n
        - msb =  4
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        - [3:0] = gpio__mux_select_lv
        -   [4] = gpio__mux_active_high_lv
*/
pub const VL53L1_GPIO_LV_PAD__CTRL: u16 = 0x04CD;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = gpio__extsup_lv
*/
pub const VL53L1_PAD_I2C_LV__CONFIG: u16 = 0x04D0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PAD_STARTUP_MODE__VALUE_RO_GO1: u16 = 0x04D4;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = pad_spi_csn_val_ro
*/
pub const VL53L1_HOST_IF__STATUS_GO1: u16 = 0x04D5;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = host_interface_lv
*/
pub const VL53L1_MCU_CLK_GATING__CTRL: u16 = 0x04D8;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['']

    fields: \n
        -   [0] = clk_gate_en__go1_mcu_bank
        -   [1] = clk_gate_en__go1_mcu_patch_ctrl
        -   [2] = clk_gate_en__go1_mcu_timers
        -   [3] = clk_gate_en__go1_mcu_mult_div
*/
pub const VL53L1_TEST__BIST_ROM_CTRL: u16 = 0x04E0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__BIST_ROM_RESULT: u16 = 0x04E1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__BIST_ROM_MCU_SIG: u16 = 0x04E2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__BIST_ROM_MCU_SIG_HI: u16 = 0x04E2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__BIST_ROM_MCU_SIG_LO: u16 = 0x04E3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__BIST_RAM_CTRL: u16 = 0x04E4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__BIST_RAM_RESULT: u16 = 0x04E5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__TMC: u16 = 0x04E8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_MIN_THRESHOLD: u16 = 0x04F0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_MIN_THRESHOLD_HI: u16 = 0x04F0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_MIN_THRESHOLD_LO: u16 = 0x04F1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_MAX_THRESHOLD: u16 = 0x04F2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_MAX_THRESHOLD_HI: u16 = 0x04F2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_MAX_THRESHOLD_LO: u16 = 0x04F3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_COUNT_OUT: u16 = 0x04F4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_COUNT_OUT_HI: u16 = 0x04F4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_COUNT_OUT_LO: u16 = 0x04F5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_GONOGO: u16 = 0x04F6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_TEST__PLL_BIST_CTRL: u16 = 0x04F7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__DEVICE_ID: u16 = 0x0680;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REVISION_ID: u16 = 0x0681;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CLK_CTRL1: u16 = 0x0683;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CLK_CTRL2: u16 = 0x0684;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__WOI_1: u16 = 0x0685;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__WOI_REF_1: u16 = 0x0686;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__START_RANGING: u16 = 0x0687;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__LOW_LIMIT_1: u16 = 0x0690;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__HIGH_LIMIT_1: u16 = 0x0691;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__LOW_LIMIT_REF_1: u16 = 0x0692;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__HIGH_LIMIT_REF_1: u16 = 0x0693;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__QUANTIFIER_1_MSB: u16 = 0x0694;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__QUANTIFIER_1_LSB: u16 = 0x0695;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__QUANTIFIER_REF_1_MSB: u16 = 0x0696;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__QUANTIFIER_REF_1_LSB: u16 = 0x0697;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_OFFSET_1_MSB: u16 = 0x0698;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_OFFSET_1_LSB: u16 = 0x0699;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_OFFSET_REF_1_MSB: u16 = 0x069A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_OFFSET_REF_1_LSB: u16 = 0x069B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__FILTER_STRENGTH_1: u16 = 0x069C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__FILTER_STRENGTH_REF_1: u16 = 0x069D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_1_MSB: u16 = 0x069E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_1_LSB: u16 = 0x069F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_MSB: u16 = 0x06A0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_LSB: u16 = 0x06A1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TIMEOUT_OVERALL_PERIODS_MSB: u16 = 0x06A4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TIMEOUT_OVERALL_PERIODS_LSB: u16 = 0x06A5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__INVERT_HW: u16 = 0x06A6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__FORCE_HW: u16 = 0x06A7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATIC_HW_VALUE: u16 = 0x06A8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__FORCE_CONTINUOUS_AMBIENT: u16 = 0x06A9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TEST_PHASE_SELECT_TO_FILTER: u16 = 0x06AA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TEST_PHASE_SELECT_TO_TIMING_GEN: u16 = 0x06AB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__INITIAL_PHASE_VALUE_1: u16 = 0x06AC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__INITIAL_PHASE_VALUE_REF_1: u16 = 0x06AD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__FORCE_UP_IN: u16 = 0x06AE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__FORCE_DN_IN: u16 = 0x06AF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATIC_UP_VALUE_1: u16 = 0x06B0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATIC_UP_VALUE_REF_1: u16 = 0x06B1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATIC_DN_VALUE_1: u16 = 0x06B2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATIC_DN_VALUE_REF_1: u16 = 0x06B3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__MONITOR_UP_DN: u16 = 0x06B4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__INVERT_UP_DN: u16 = 0x06B5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CPUMP_1: u16 = 0x06B6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CPUMP_2: u16 = 0x06B7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CPUMP_3: u16 = 0x06B8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__OSC_1: u16 = 0x06B9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__PLL_1: u16 = 0x06BB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__PLL_2: u16 = 0x06BC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REFERENCE_1: u16 = 0x06BD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REFERENCE_3: u16 = 0x06BF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REFERENCE_4: u16 = 0x06C0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REFERENCE_5: u16 = 0x06C1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REGAVDD1V2: u16 = 0x06C3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CALIB_1: u16 = 0x06C4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CALIB_2: u16 = 0x06C5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CALIB_3: u16 = 0x06C6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TST_MUX_SEL1: u16 = 0x06C9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TST_MUX_SEL2: u16 = 0x06CA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TST_MUX: u16 = 0x06CB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__GPIO_OUT_TESTMUX: u16 = 0x06CC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CUSTOM_FE: u16 = 0x06CD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CUSTOM_FE_2: u16 = 0x06CE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_READOUT: u16 = 0x06CF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_READOUT_1: u16 = 0x06D0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_READOUT_2: u16 = 0x06D1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_PS: u16 = 0x06D2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__LASER_SAFETY_2: u16 = 0x06D4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__MODE: u16 = 0x0780;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__PDN: u16 = 0x0781;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__PROGN: u16 = 0x0782;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__READN: u16 = 0x0783;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__PULSE_WIDTH_MSB: u16 = 0x0784;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__PULSE_WIDTH_LSB: u16 = 0x0785;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__HV_RISE_MSB: u16 = 0x0786;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__HV_RISE_LSB: u16 = 0x0787;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__HV_FALL_MSB: u16 = 0x0788;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__HV_FALL_LSB: u16 = 0x0789;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__TST: u16 = 0x078A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__TESTREAD: u16 = 0x078B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_MMM: u16 = 0x078C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_LMM: u16 = 0x078D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_LLM: u16 = 0x078E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_LLL: u16 = 0x078F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_MMM: u16 = 0x0790;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_LMM: u16 = 0x0791;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_LLM: u16 = 0x0792;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_LLL: u16 = 0x0793;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__ADDR: u16 = 0x0794;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_ECC: u16 = 0x0795;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_0: u16 = 0x0796;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_1: u16 = 0x0797;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_2: u16 = 0x0798;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_3: u16 = 0x0799;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_4: u16 = 0x079A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_5: u16 = 0x079B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_6: u16 = 0x079C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_7: u16 = 0x079D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_8: u16 = 0x079E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_9: u16 = 0x079F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_10: u16 = 0x07A0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_11: u16 = 0x07A1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_12: u16 = 0x07A2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_13: u16 = 0x07A3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_14: u16 = 0x07A4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_15: u16 = 0x07A5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_16: u16 = 0x07A6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_17: u16 = 0x07A7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_SHIFT_EN: u16 = 0x07BA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_DISABLE_CTRL: u16 = 0x07BB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_EN_SHIFT_OUT_DEBUG: u16 = 0x07BC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPI_MODE: u16 = 0x07BD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__GPIO_DIR: u16 = 0x07BE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_PERIOD: u16 = 0x0880;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_START: u16 = 0x0881;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_STOP: u16 = 0x0882;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_1: u16 = 0x0885;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_STATUS: u16 = 0x088D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATUS: u16 = 0x0980;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__LASER_CONTINUITY_STATE: u16 = 0x0981;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_1_MMM: u16 = 0x0982;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_1_LMM: u16 = 0x0983;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_1_LLM: u16 = 0x0984;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_1_LLL: u16 = 0x0985;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_REF_1_MMM: u16 = 0x0986;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_REF_1_LMM: u16 = 0x0987;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_REF_1_LLM: u16 = 0x0988;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGE_REF_1_LLL: u16 = 0x0989;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_MMM: u16 = 0x098A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LMM: u16 = 0x098B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLM: u16 = 0x098C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLL: u16 = 0x098D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_MMM: u16 = 0x098E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_LMM: u16 = 0x098F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLM: u16 = 0x0990;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLL: u16 = 0x0991;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_MMM: u16 = 0x0992;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LMM: u16 = 0x0993;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLM: u16 = 0x0994;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLL: u16 = 0x0995;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_MM: u16 = 0x0996;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LM: u16 = 0x0997;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LL: u16 = 0x0998;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_MISMATCH_MM: u16 = 0x0999;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_MISMATCH_LM: u16 = 0x099A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_MISMATCH_LL: u16 = 0x099B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_MMM: u16 = 0x099C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LMM: u16 = 0x099D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLM: u16 = 0x099E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLL: u16 = 0x099F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_MMM: u16 = 0x09A0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LMM: u16 = 0x09A1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLM: u16 = 0x09A2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLL: u16 = 0x09A3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_MMM: u16 = 0x09A4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LMM: u16 = 0x09A5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLM: u16 = 0x09A6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLL: u16 = 0x09A7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_MM: u16 = 0x09A8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LM: u16 = 0x09A9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LL: u16 = 0x09AA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_MISMATCH_REF_MM: u16 = 0x09AB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_MISMATCH_REF_LM: u16 = 0x09AC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__AMBIENT_MISMATCH_REF_LL: u16 = 0x09AD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__GPIO_CONFIG__A0: u16 = 0x0A00;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RESET_CONTROL__A0: u16 = 0x0A01;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__INTR_MANAGER__A0: u16 = 0x0A02;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__POWER_FSM_TIME_OSC__A0: u16 = 0x0A06;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_ATEST__A0: u16 = 0x0A07;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_PERIOD_CLIPPED__A0: u16 = 0x0A08;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_STOP_CLIPPED__A0: u16 = 0x0A09;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CALIB_2__A0: u16 = 0x0A0A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STOP_CONDITION__A0: u16 = 0x0A0B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__STATUS_RESET__A0: u16 = 0x0A0C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__READOUT_CFG__A0: u16 = 0x0A0D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__WINDOW_SETTING__A0: u16 = 0x0A0E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_DELAY__A0: u16 = 0x0A1A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REFERENCE_2__A0: u16 = 0x0A1B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REGAVDD1V2__A0: u16 = 0x0A1D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__TST_MUX__A0: u16 = 0x0A1F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CUSTOM_FE_2__A0: u16 = 0x0A20;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPAD_READOUT__A0: u16 = 0x0A21;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__CPUMP_1__A0: u16 = 0x0A22;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__SPARE_REGISTER__A0: u16 = 0x0A23;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__VCSEL_CONT_STAGE5_BYPASS__A0: u16 = 0x0A24;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_18: u16 = 0x0A25;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_19: u16 = 0x0A26;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_20: u16 = 0x0A27;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_21: u16 = 0x0A28;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_22: u16 = 0x0A29;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_23: u16 = 0x0A2A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_24: u16 = 0x0A2B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_25: u16 = 0x0A2C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_26: u16 = 0x0A2D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_27: u16 = 0x0A2E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_28: u16 = 0x0A2F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_29: u16 = 0x0A30;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_30: u16 = 0x0A31;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__RET_SPAD_EN_31: u16 = 0x0A32;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_SPAD_EN_0__EWOK: u16 = 0x0A33;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_SPAD_EN_1__EWOK: u16 = 0x0A34;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_SPAD_EN_2__EWOK: u16 = 0x0A35;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_SPAD_EN_3__EWOK: u16 = 0x0A36;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_SPAD_EN_4__EWOK: u16 = 0x0A37;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_SPAD_EN_5__EWOK: u16 = 0x0A38;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REF_EN_START_SELECT: u16 = 0x0A39;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGING_CORE__REGDVDD1V2_ATEST__EWOK: u16 = 0x0A41;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SOFT_RESET_GO1: u16 = 0x0B00;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PRIVATE__PATCH_BASE_ADDR_RSLV: u16 = 0x0E00;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__INTERRUPT_STATUS: u16 = 0x0ED0;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [2:0] = prev_shadow_int_status
        - [4:3] = prev_shadow_int_error_status
        -   [5] = prev_shadow_gph_id_gpio_status
*/
pub const VL53L1_PREV_SHADOW_RESULT__RANGE_STATUS: u16 = 0x0ED1;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [4:0] = prev_shadow_range_status
        -   [5] = prev_shadow_max_threshold_hit
        -   [6] = prev_shadow_min_threshold_hit
        -   [7] = prev_shadow_gph_id_range_status
*/
pub const VL53L1_PREV_SHADOW_RESULT__REPORT_STATUS: u16 = 0x0ED2;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [3:0] = prev_shadow_report_status
*/
pub const VL53L1_PREV_SHADOW_RESULT__STREAM_COUNT: u16 = 0x0ED3;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [7:0] = prev_shadow_result__stream_count
*/
pub const VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x0ED4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__dss_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x0ED4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x0ED5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0: u16 = 0x0ED6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__peak_signal_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0ED6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0ED7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0: u16 = 0x0ED8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__ambient_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0ED8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0ED9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SIGMA_SD0: u16 = 0x0EDA;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__sigma_sd0 (fixed point 14.2)
*/
pub const VL53L1_PREV_SHADOW_RESULT__SIGMA_SD0_HI: u16 = 0x0EDA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SIGMA_SD0_LO: u16 = 0x0EDB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PHASE_SD0: u16 = 0x0EDC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__phase_sd0 (fixed point 5.11)
*/
pub const VL53L1_PREV_SHADOW_RESULT__PHASE_SD0_HI: u16 = 0x0EDC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PHASE_SD0_LO: u16 = 0x0EDD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: u16 = 0x0EDE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__final_crosstalk_corrected_range_mm_sd0
*/
pub const VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI: u16 = 0x0EDE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO: u16 = 0x0EDF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0: u16 =
    0x0EE0;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI: u16 =
    0x0EE0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO: u16 =
    0x0EE1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x0EE2;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__mm_inner_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x0EE2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x0EE3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x0EE4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__mm_outer_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x0EE4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x0EE5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0: u16 = 0x0EE6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__avg_signal_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0EE6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0EE7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1: u16 = 0x0EE8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__dss_actual_effective_spads_sd1 (fixed point 8.8)
*/
pub const VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI: u16 = 0x0EE8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO: u16 = 0x0EE9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1: u16 = 0x0EEA;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__peak_signal_count_rate_mcps_sd1 (fixed point 9.7)
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI: u16 = 0x0EEA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO: u16 = 0x0EEB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1: u16 = 0x0EEC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__ambient_count_rate_mcps_sd1 (fixed point 9.7)
*/
pub const VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI: u16 = 0x0EEC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO: u16 = 0x0EED;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SIGMA_SD1: u16 = 0x0EEE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__sigma_sd1 (fixed point 14.2)
*/
pub const VL53L1_PREV_SHADOW_RESULT__SIGMA_SD1_HI: u16 = 0x0EEE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SIGMA_SD1_LO: u16 = 0x0EEF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PHASE_SD1: u16 = 0x0EF0;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__phase_sd1 (fixed point 5.11)
*/
pub const VL53L1_PREV_SHADOW_RESULT__PHASE_SD1_HI: u16 = 0x0EF0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__PHASE_SD1_LO: u16 = 0x0EF1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1: u16 = 0x0EF2;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__final_crosstalk_corrected_range_mm_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI: u16 = 0x0EF2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO: u16 = 0x0EF3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_0_SD1: u16 = 0x0EF4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__spare_0_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_0_SD1_HI: u16 = 0x0EF4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_0_SD1_LO: u16 = 0x0EF5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_1_SD1: u16 = 0x0EF6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__spare_1_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_1_SD1_HI: u16 = 0x0EF6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_1_SD1_LO: u16 = 0x0EF7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_2_SD1: u16 = 0x0EF8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__spare_2_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_2_SD1_HI: u16 = 0x0EF8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_2_SD1_LO: u16 = 0x0EF9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_3_SD1: u16 = 0x0EFA;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['prev_shadow_system_results', 'results']

    fields: \n
        - [15:0] = prev_shadow_result__spare_3_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_3_SD1_HI: u16 = 0x0EFA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT__SPARE_3_SD1_LO: u16 = 0x0EFB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0: u16 = 0x0EFC;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__ambient_window_events_sd0
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3: u16 = 0x0EFC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2: u16 = 0x0EFD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1: u16 = 0x0EFE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0: u16 = 0x0EFF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0: u16 = 0x0F00;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__ranging_total_events_sd0
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3: u16 = 0x0F00;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2: u16 = 0x0F01;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1: u16 = 0x0F02;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0: u16 = 0x0F03;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0: u16 = 0x0F04;
/*
    type:     int32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__signal_total_events_sd0
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3: u16 = 0x0F04;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2: u16 = 0x0F05;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1: u16 = 0x0F06;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0: u16 = 0x0F07;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0: u16 = 0x0F08;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__total_periods_elapsed_sd0
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3: u16 = 0x0F08;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2: u16 = 0x0F09;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1: u16 = 0x0F0A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0: u16 = 0x0F0B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1: u16 = 0x0F0C;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__ambient_window_events_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3: u16 = 0x0F0C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2: u16 = 0x0F0D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1: u16 = 0x0F0E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0: u16 = 0x0F0F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1: u16 = 0x0F10;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__ranging_total_events_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3: u16 = 0x0F10;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2: u16 = 0x0F11;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1: u16 = 0x0F12;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0: u16 = 0x0F13;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1: u16 = 0x0F14;
/*
    type:     int32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__signal_total_events_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3: u16 = 0x0F14;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2: u16 = 0x0F15;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1: u16 = 0x0F16;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0: u16 = 0x0F17;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1: u16 = 0x0F18;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = prev_shadow_result_core__total_periods_elapsed_sd1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3: u16 = 0x0F18;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2: u16 = 0x0F19;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1: u16 = 0x0F1A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0: u16 = 0x0F1B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PREV_SHADOW_RESULT_CORE__SPARE_0: u16 = 0x0F1C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['prev_shadow_core_results', 'ranging_core_results']

    fields: \n
        - [7:0] = prev_shadow_result_core__spare_0
*/
pub const VL53L1_RESULT__DEBUG_STATUS: u16 = 0x0F20;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_debug', 'misc_results']

    fields: \n
        - [7:0] = result_debug_status
*/
pub const VL53L1_RESULT__DEBUG_STAGE: u16 = 0x0F21;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_debug', 'misc_results']

    fields: \n
        - [7:0] = result_debug_stage
*/
pub const VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH: u16 = 0x0F24;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['gph_general_config', 'dss_config']

    fields: \n
        - [15:0] = gph__system_thresh_rate_high (fixed point 9.7)
*/
pub const VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH_HI: u16 = 0x0F24;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH_LO: u16 = 0x0F25;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_RATE_LOW: u16 = 0x0F26;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['gph_general_config', 'dss_config']

    fields: \n
        - [15:0] = gph__system_thresh_rate_low (fixed point 9.7)
*/
pub const VL53L1_GPH__SYSTEM__THRESH_RATE_LOW_HI: u16 = 0x0F26;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__THRESH_RATE_LOW_LO: u16 = 0x0F27;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__SYSTEM__INTERRUPT_CONFIG_GPIO: u16 = 0x0F28;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_general_config', 'gph_config']

    fields: \n
        - [1:0] = gph__int_mode_distance
        - [3:2] = gph__int_mode_rate
        -   [4] = gph__int_spare
        -   [5] = gph__int_new_measure_ready
        -   [6] = gph__int_no_target_en
        -   [7] = gph__int_combined_mode
*/
pub const VL53L1_GPH__DSS_CONFIG__ROI_MODE_CONTROL: u16 = 0x0F2F;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  2
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_static_config', 'dss_config']

    fields: \n
        - [1:0] = gph__dss_config__input_mode
        -   [2] = gph__calculate_roi_enable
*/
pub const VL53L1_GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT: u16 = 0x0F30;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['gph_static_config', 'dss_config']

    fields: \n
        - [15:0] = gph__dss_config__manual_effective_spads_select
*/
pub const VL53L1_GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI: u16 = 0x0F30;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO: u16 = 0x0F31;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__DSS_CONFIG__MANUAL_BLOCK_SELECT: u16 = 0x0F32;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_static_config', 'dss_config']

    fields: \n
        - [7:0] = gph__dss_config__manual_block_select
*/
pub const VL53L1_GPH__DSS_CONFIG__MAX_SPADS_LIMIT: u16 = 0x0F33;
/*
    type:     uint8_t \n
    default:  0xFF \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_static_config', 'dss_config']

    fields: \n
        - [7:0] = gph__dss_config__max_spads_limit
*/
pub const VL53L1_GPH__DSS_CONFIG__MIN_SPADS_LIMIT: u16 = 0x0F34;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_static_config', 'dss_config']

    fields: \n
        - [7:0] = gph__dss_config__min_spads_limit
*/
pub const VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI: u16 = 0x0F36;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'mm_config']

    fields: \n
        - [3:0] = gph_mm_config__config_timeout_macrop_a_hi
*/
pub const VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_A_LO: u16 = 0x0F37;
/*
    type:     uint8_t \n
    default:  0x06 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'mm_config']

    fields: \n
        - [7:0] = gph_mm_config__config_timeout_macrop_a_lo
*/
pub const VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_B_HI: u16 = 0x0F38;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'mm_config']

    fields: \n
        - [3:0] = gph_mm_config__config_timeout_macrop_b_hi
*/
pub const VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_B_LO: u16 = 0x0F39;
/*
    type:     uint8_t \n
    default:  0x06 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'mm_config']

    fields: \n
        - [7:0] = gph_mm_config__config_timeout_macrop_b_lo
*/
pub const VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_HI: u16 = 0x0F3A;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [3:0] = gph_range_timeout_overall_periods_macrop_a_hi
*/
pub const VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_LO: u16 = 0x0F3B;
/*
    type:     uint8_t \n
    default:  0x92 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [7:0] = gph_range_timeout_overall_periods_macrop_a_lo
*/
pub const VL53L1_GPH__RANGE_CONFIG__VCSEL_PERIOD_A: u16 = 0x0F3C;
/*
    type:     uint8_t \n
    default:  0x0B \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [5:0] = gph_range_config__vcsel_period_a
*/
pub const VL53L1_GPH__RANGE_CONFIG__VCSEL_PERIOD_B: u16 = 0x0F3D;
/*
    type:     uint8_t \n
    default:  0x09 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [5:0] = gph_range_config__vcsel_period_b
*/
pub const VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_HI: u16 = 0x0F3E;
/*
    type:     uint8_t \n
    default:  0x01 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [3:0] = gph_range_timeout_overall_periods_macrop_b_hi
*/
pub const VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_LO: u16 = 0x0F3F;
/*
    type:     uint8_t \n
    default:  0x92 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [7:0] = gph_range_timeout_overall_periods_macrop_b_lo
*/
pub const VL53L1_GPH__RANGE_CONFIG__SIGMA_THRESH: u16 = 0x0F40;
/*
    type:     uint16_t \n
    default:  0x0080 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [15:0] = gph_range_config__sigma_thresh (fixed point 14.2)
*/
pub const VL53L1_GPH__RANGE_CONFIG__SIGMA_THRESH_HI: u16 = 0x0F40;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__RANGE_CONFIG__SIGMA_THRESH_LO: u16 = 0x0F41;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS: u16 = 0x0F42;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [15:0] = gph_range_config__min_count_rate_rtn_limit_mcps (fixed point 9.7)
*/
pub const VL53L1_GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI: u16 = 0x0F42;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO: u16 = 0x0F43;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_GPH__RANGE_CONFIG__VALID_PHASE_LOW: u16 = 0x0F44;
/*
    type:     uint8_t \n
    default:  0x08 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [7:0] = gph_range_config__valid_phase_low (fixed point 5.3)
*/
pub const VL53L1_GPH__RANGE_CONFIG__VALID_PHASE_HIGH: u16 = 0x0F45;
/*
    type:     uint8_t \n
    default:  0x80 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['gph_timing_config', 'range_config']

    fields: \n
        - [7:0] = gph_range_config__valid_phase_high (fixed point 5.3)
*/
pub const VL53L1_FIRMWARE__INTERNAL_STREAM_COUNT_DIV: u16 = 0x0F46;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['fw_internal']

    fields: \n
        - [7:0] = fw__internal_stream_count_div
*/
pub const VL53L1_FIRMWARE__INTERNAL_STREAM_COUNTER_VAL: u16 = 0x0F47;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['fw_internal']

    fields: \n
        - [7:0] = fw__internal_stream_counter_val
*/
pub const VL53L1_DSS_CALC__ROI_CTRL: u16 = 0x0F54;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  1
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        -   [0] = dss_calc__roi_intersect_enable
        -   [1] = dss_calc__roi_subtract_enable
*/
pub const VL53L1_DSS_CALC__SPARE_1: u16 = 0x0F55;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_1
*/
pub const VL53L1_DSS_CALC__SPARE_2: u16 = 0x0F56;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_2
*/
pub const VL53L1_DSS_CALC__SPARE_3: u16 = 0x0F57;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_3
*/
pub const VL53L1_DSS_CALC__SPARE_4: u16 = 0x0F58;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_4
*/
pub const VL53L1_DSS_CALC__SPARE_5: u16 = 0x0F59;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_5
*/
pub const VL53L1_DSS_CALC__SPARE_6: u16 = 0x0F5A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_6
*/
pub const VL53L1_DSS_CALC__SPARE_7: u16 = 0x0F5B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__spare_7
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_0: u16 = 0x0F5C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_0
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_1: u16 = 0x0F5D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_1
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_2: u16 = 0x0F5E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_2
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_3: u16 = 0x0F5F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_3
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_4: u16 = 0x0F60;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_4
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_5: u16 = 0x0F61;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_5
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_6: u16 = 0x0F62;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_6
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_7: u16 = 0x0F63;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_7
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_8: u16 = 0x0F64;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_8
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_9: u16 = 0x0F65;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_9
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_10: u16 = 0x0F66;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_10
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_11: u16 = 0x0F67;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_11
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_12: u16 = 0x0F68;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_12
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_13: u16 = 0x0F69;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_13
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_14: u16 = 0x0F6A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_14
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_15: u16 = 0x0F6B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_15
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_16: u16 = 0x0F6C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_16
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_17: u16 = 0x0F6D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_17
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_18: u16 = 0x0F6E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_18
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_19: u16 = 0x0F6F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_19
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_20: u16 = 0x0F70;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_20
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_21: u16 = 0x0F71;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_21
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_22: u16 = 0x0F72;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_22
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_23: u16 = 0x0F73;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_23
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_24: u16 = 0x0F74;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_24
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_25: u16 = 0x0F75;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_25
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_26: u16 = 0x0F76;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_26
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_27: u16 = 0x0F77;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_27
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_28: u16 = 0x0F78;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_28
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_29: u16 = 0x0F79;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_29
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_30: u16 = 0x0F7A;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_30
*/
pub const VL53L1_DSS_CALC__USER_ROI_SPAD_EN_31: u16 = 0x0F7B;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_spad_en_31
*/
pub const VL53L1_DSS_CALC__USER_ROI_0: u16 = 0x0F7C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_0
*/
pub const VL53L1_DSS_CALC__USER_ROI_1: u16 = 0x0F7D;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__user_roi_1
*/
pub const VL53L1_DSS_CALC__MODE_ROI_0: u16 = 0x0F7E;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__mode_roi_0
*/
pub const VL53L1_DSS_CALC__MODE_ROI_1: u16 = 0x0F7F;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_calc']

    fields: \n
        - [7:0] = dss_calc__mode_roi_1
*/
pub const VL53L1_SIGMA_ESTIMATOR_CALC__SPARE_0: u16 = 0x0F80;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'sigma_est_spare']

    fields: \n
        - [7:0] = sigma_estimator_calc__spare_0
*/
pub const VL53L1_VHV_RESULT__PEAK_SIGNAL_RATE_MCPS: u16 = 0x0F82;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'vhv_results']

    fields: \n
        - [15:0] = vhv_result__peak_signal_rate_mcps
*/
pub const VL53L1_VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_HI: u16 = 0x0F82;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_LO: u16 = 0x0F83;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF: u16 = 0x0F84;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['patch_results', 'vhv_results']

    fields: \n
        - [31:0] = vhv_result__signal_total_events_ref
*/
pub const VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_3: u16 = 0x0F84;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_2: u16 = 0x0F85;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_1: u16 = 0x0F86;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_0: u16 = 0x0F87;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PHASECAL_RESULT__PHASE_OUTPUT_REF: u16 = 0x0F88;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'phasecal_results']

    fields: \n
        - [15:0] = phasecal_result__normalised_phase_ref
*/
pub const VL53L1_PHASECAL_RESULT__PHASE_OUTPUT_REF_HI: u16 = 0x0F88;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_PHASECAL_RESULT__PHASE_OUTPUT_REF_LO: u16 = 0x0F89;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_RESULT__TOTAL_RATE_PER_SPAD: u16 = 0x0F8A;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'dss_results']

    fields: \n
        - [15:0] = dss_result__total_rate_per_spad
*/
pub const VL53L1_DSS_RESULT__TOTAL_RATE_PER_SPAD_HI: u16 = 0x0F8A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_RESULT__TOTAL_RATE_PER_SPAD_LO: u16 = 0x0F8B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_RESULT__ENABLED_BLOCKS: u16 = 0x0F8C;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['patch_results', 'dss_results']

    fields: \n
        - [7:0] = dss_result__enabled_blocks
*/
pub const VL53L1_DSS_RESULT__NUM_REQUESTED_SPADS: u16 = 0x0F8E;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'dss_results']

    fields: \n
        - [15:0] = dss_result__num_requested_spads (fixed point 8.8)
*/
pub const VL53L1_DSS_RESULT__NUM_REQUESTED_SPADS_HI: u16 = 0x0F8E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_DSS_RESULT__NUM_REQUESTED_SPADS_LO: u16 = 0x0F8F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_RESULT__INNER_INTERSECTION_RATE: u16 = 0x0F92;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'mm_results']

    fields: \n
        - [15:0] = mm_result__inner_intersection_rate
*/
pub const VL53L1_MM_RESULT__INNER_INTERSECTION_RATE_HI: u16 = 0x0F92;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_RESULT__INNER_INTERSECTION_RATE_LO: u16 = 0x0F93;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_RESULT__OUTER_COMPLEMENT_RATE: u16 = 0x0F94;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'mm_results']

    fields: \n
        - [15:0] = mm_result__outer_complement_rate
*/
pub const VL53L1_MM_RESULT__OUTER_COMPLEMENT_RATE_HI: u16 = 0x0F94;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_RESULT__OUTER_COMPLEMENT_RATE_LO: u16 = 0x0F95;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_RESULT__TOTAL_OFFSET: u16 = 0x0F96;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'mm_results']

    fields: \n
        - [15:0] = mm_result__total_offset
*/
pub const VL53L1_MM_RESULT__TOTAL_OFFSET_HI: u16 = 0x0F96;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_MM_RESULT__TOTAL_OFFSET_LO: u16 = 0x0F97;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS: u16 = 0x0F98;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 23
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['patch_results', 'xtalk_calc']

    fields: \n
        - [23:0] = xtalk_calc__xtalk_for_enabled_spads (fixed point 11.13)
*/
pub const VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_3: u16 = 0x0F98;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_2: u16 = 0x0F99;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_1: u16 = 0x0F9A;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_0: u16 = 0x0F9B;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS: u16 = 0x0F9C;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 23
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['patch_results', 'xtalk_results']

    fields: \n
        - [23:0] = xtalk_result__avg_xtalk_user_roi_kcps (fixed point 11.13)
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_3: u16 = 0x0F9C;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_2: u16 = 0x0F9D;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_1: u16 = 0x0F9E;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_0: u16 = 0x0F9F;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS: u16 = 0x0FA0;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 23
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['patch_results', 'xtalk_results']

    fields: \n
        - [23:0] = xtalk_result__avg_xtalk_mm_inner_roi_kcps (fixed point 11.13)
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_3: u16 = 0x0FA0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_2: u16 = 0x0FA1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_1: u16 = 0x0FA2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_0: u16 = 0x0FA3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS: u16 = 0x0FA4;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 23
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['patch_results', 'xtalk_results']

    fields: \n
        - [23:0] = xtalk_result__avg_xtalk_mm_outer_roi_kcps (fixed point 11.13)
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_3: u16 = 0x0FA4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_2: u16 = 0x0FA5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_1: u16 = 0x0FA6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_0: u16 = 0x0FA7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_RESULT__ACCUM_PHASE: u16 = 0x0FA8;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['patch_results', 'range_results']

    fields: \n
        - [31:0] = range_result__accum_phase
*/
pub const VL53L1_RANGE_RESULT__ACCUM_PHASE_3: u16 = 0x0FA8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_RESULT__ACCUM_PHASE_2: u16 = 0x0FA9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_RESULT__ACCUM_PHASE_1: u16 = 0x0FAA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_RESULT__ACCUM_PHASE_0: u16 = 0x0FAB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_RESULT__OFFSET_CORRECTED_RANGE: u16 = 0x0FAC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['patch_results', 'range_results']

    fields: \n
        - [15:0] = range_result__offset_corrected_range
*/
pub const VL53L1_RANGE_RESULT__OFFSET_CORRECTED_RANGE_HI: u16 = 0x0FAC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_RANGE_RESULT__OFFSET_CORRECTED_RANGE_LO: u16 = 0x0FAD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_PHASECAL_RESULT__VCSEL_START: u16 = 0x0FAE;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'histogram_results']

    fields: \n
        - [7:0] = shadow_phasecal_result__vcsel_start
*/
pub const VL53L1_SHADOW_RESULT__INTERRUPT_STATUS: u16 = 0x0FB0;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  5
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [2:0] = shadow_int_status
        - [4:3] = shadow_int_error_status
        -   [5] = shadow_gph_id_gpio_status
*/
pub const VL53L1_SHADOW_RESULT__RANGE_STATUS: u16 = 0x0FB1;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [4:0] = shadow_range_status
        -   [5] = shadow_max_threshold_hit
        -   [6] = shadow_min_threshold_hit
        -   [7] = shadow_gph_id_range_status
*/
pub const VL53L1_SHADOW_RESULT__REPORT_STATUS: u16 = 0x0FB2;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  3
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [3:0] = shadow_report_status
*/
pub const VL53L1_SHADOW_RESULT__STREAM_COUNT: u16 = 0x0FB3;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [7:0] = shadow_result__stream_count
*/
pub const VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x0FB4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__dss_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x0FB4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x0FB5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0: u16 = 0x0FB6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__peak_signal_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0FB6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0FB7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0: u16 = 0x0FB8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__ambient_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0FB8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0FB9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SIGMA_SD0: u16 = 0x0FBA;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__sigma_sd0 (fixed point 14.2)
*/
pub const VL53L1_SHADOW_RESULT__SIGMA_SD0_HI: u16 = 0x0FBA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SIGMA_SD0_LO: u16 = 0x0FBB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PHASE_SD0: u16 = 0x0FBC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__phase_sd0 (fixed point 5.11)
*/
pub const VL53L1_SHADOW_RESULT__PHASE_SD0_HI: u16 = 0x0FBC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PHASE_SD0_LO: u16 = 0x0FBD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: u16 = 0x0FBE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__final_crosstalk_corrected_range_mm_sd0
*/
pub const VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI: u16 = 0x0FBE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO: u16 = 0x0FBF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0: u16 = 0x0FC0;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI: u16 =
    0x0FC0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO: u16 =
    0x0FC1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x0FC2;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__mm_inner_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x0FC2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x0FC3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x0FC4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__mm_outer_actual_effective_spads_sd0 (fixed point 8.8)
*/
pub const VL53L1_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI: u16 = 0x0FC4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO: u16 = 0x0FC5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0: u16 = 0x0FC6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__avg_signal_count_rate_mcps_sd0 (fixed point 9.7)
*/
pub const VL53L1_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI: u16 = 0x0FC6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO: u16 = 0x0FC7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1: u16 = 0x0FC8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__dss_actual_effective_spads_sd1 (fixed point 8.8)
*/
pub const VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI: u16 = 0x0FC8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO: u16 = 0x0FC9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1: u16 = 0x0FCA;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__peak_signal_count_rate_mcps_sd1 (fixed point 9.7)
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI: u16 = 0x0FCA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO: u16 = 0x0FCB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1: u16 = 0x0FCC;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__ambient_count_rate_mcps_sd1 (fixed point 9.7)
*/
pub const VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI: u16 = 0x0FCC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO: u16 = 0x0FCD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SIGMA_SD1: u16 = 0x0FCE;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__sigma_sd1 (fixed point 14.2)
*/
pub const VL53L1_SHADOW_RESULT__SIGMA_SD1_HI: u16 = 0x0FCE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SIGMA_SD1_LO: u16 = 0x0FCF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PHASE_SD1: u16 = 0x0FD0;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__phase_sd1 (fixed point 5.11)
*/
pub const VL53L1_SHADOW_RESULT__PHASE_SD1_HI: u16 = 0x0FD0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__PHASE_SD1_LO: u16 = 0x0FD1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1: u16 = 0x0FD2;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__final_crosstalk_corrected_range_mm_sd1
*/
pub const VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI: u16 = 0x0FD2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO: u16 = 0x0FD3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_0_SD1: u16 = 0x0FD4;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__spare_0_sd1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_0_SD1_HI: u16 = 0x0FD4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_0_SD1_LO: u16 = 0x0FD5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_1_SD1: u16 = 0x0FD6;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__spare_1_sd1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_1_SD1_HI: u16 = 0x0FD6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_1_SD1_LO: u16 = 0x0FD7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_2_SD1: u16 = 0x0FD8;
/*
    type:     uint16_t \n
    default:  0x0000 \n
    info: \n
        - msb = 15
        - lsb =  0
        - i2c_size =  2

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [15:0] = shadow_result__spare_2_sd1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_2_SD1_HI: u16 = 0x0FD8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_2_SD1_LO: u16 = 0x0FD9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT__SPARE_3_SD1: u16 = 0x0FDA;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [7:0] = shadow_result__spare_3_sd1
*/
pub const VL53L1_SHADOW_RESULT__THRESH_INFO: u16 = 0x0FDB;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'results']

    fields: \n
        - [3:0] = shadow_result__distance_int_info
        - [7:4] = shadow_result__rate_int_info
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0: u16 = 0x0FDC;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__ambient_window_events_sd0
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3: u16 = 0x0FDC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2: u16 = 0x0FDD;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1: u16 = 0x0FDE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0: u16 = 0x0FDF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0: u16 = 0x0FE0;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__ranging_total_events_sd0
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3: u16 = 0x0FE0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2: u16 = 0x0FE1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1: u16 = 0x0FE2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0: u16 = 0x0FE3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0: u16 = 0x0FE4;
/*
    type:     int32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__signal_total_events_sd0
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3: u16 = 0x0FE4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2: u16 = 0x0FE5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1: u16 = 0x0FE6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0: u16 = 0x0FE7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0: u16 = 0x0FE8;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__total_periods_elapsed_sd0
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3: u16 = 0x0FE8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2: u16 = 0x0FE9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1: u16 = 0x0FEA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0: u16 = 0x0FEB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1: u16 = 0x0FEC;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__ambient_window_events_sd1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3: u16 = 0x0FEC;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2: u16 = 0x0FED;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1: u16 = 0x0FEE;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0: u16 = 0x0FEF;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1: u16 = 0x0FF0;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__ranging_total_events_sd1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3: u16 = 0x0FF0;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2: u16 = 0x0FF1;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1: u16 = 0x0FF2;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0: u16 = 0x0FF3;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1: u16 = 0x0FF4;
/*
    type:     int32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__signal_total_events_sd1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3: u16 = 0x0FF4;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2: u16 = 0x0FF5;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1: u16 = 0x0FF6;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0: u16 = 0x0FF7;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1: u16 = 0x0FF8;
/*
    type:     uint32_t \n
    default:  0x00000000 \n
    info: \n
        - msb = 31
        - lsb =  0
        - i2c_size =  4

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [31:0] = shadow_result_core__total_periods_elapsed_sd1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3: u16 = 0x0FF8;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2: u16 = 0x0FF9;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1: u16 = 0x0FFA;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0: u16 = 0x0FFB;
/*
    info: \n
        - msb =  0
        - lsb =  0
        - i2c_size =  1
*/
pub const VL53L1_SHADOW_RESULT_CORE__SPARE_0: u16 = 0x0FFC;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_core_results', 'ranging_core_results']

    fields: \n
        - [7:0] = shadow_result_core__spare_0
*/
pub const VL53L1_SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_HI: u16 = 0x0FFE;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'histogram_results']

    fields: \n
        - [7:0] = shadow_phasecal_result__reference_phase_hi
*/
pub const VL53L1_SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_LO: u16 = 0x0FFF;
/*
    type:     uint8_t \n
    default:  0x00 \n
    info: \n
        - msb =  7
        - lsb =  0
        - i2c_size =  1

    groups: \n
        ['shadow_system_results', 'histogram_results']

    fields: \n
        - [7:0] = shadow_phasecal_result__reference_phase_lo
*/
