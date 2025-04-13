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
 * @file vl53l1_register_settings.h
 *
 * @brief Device register setting defines.
 */

/* @defgroup VL53L1_RegisterSettings_group  Functionality
 *  @brief Defines the register settings for key device
*         configuration registers
*/

/* @defgroup VL53L1_DeviceSchedulerMode_group - Pseudo, Streaming & Hist
 *  @brief Values below match the bit positions in the SYSTEM__MODE_START
*         register do not change
*/

pub const VL53L1_DEVICESCHEDULERMODE_PSEUDO_SOLO: u8 = 0x00;
pub const VL53L1_DEVICESCHEDULERMODE_STREAMING: u8 = 0x01;
pub const VL53L1_DEVICESCHEDULERMODE_HISTOGRAM: u8 = 0x02;

/* @defgroup VL53L1_DeviceReadoutMode_group - Single, Dual, Split & Manual
 *  @brief Values below match the bit positions in the SYSTEM__MODE_START
*         register do not change
*/

pub const VL53L1_DEVICEREADOUTMODE_SINGLE_SD: u8 = 0x00 << 2;
pub const VL53L1_DEVICEREADOUTMODE_DUAL_SD: u8 = 0x01 << 2;
pub const VL53L1_DEVICEREADOUTMODE_SPLIT_READOUT: u8 = 0x02 << 2;
pub const VL53L1_DEVICEREADOUTMODE_SPLIT_MANUAL: u8 = 0x03 << 2;

/* @defgroup VL53L1_DeviceMeasurementMode_group - SingleShot, BackToBack & timed
 *  @brief Values below match the bit positions in the SYSTEM__MODE_START
*         register do not change
*/

/*
pub const VL53L1_DEVICEMEASUREMENTMODE_STOP: u8 = 0x00;
pub const VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT: u8 = 0x10;
pub const VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK: u8 = 0x20;
pub const VL53L1_DEVICEMEASUREMENTMODE_TIMED: u8 = 0x40;
pub const VL53L1_DEVICEMEASUREMENTMODE_ABORT: u8 = 0x80;
*/
pub const VL53L1_DEVICEMEASUREMENTMODE_MODE_MASK: u8 = 0xF0;
pub const VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK: u8 = 0x0F;

pub const VL53L1_GROUPEDPARAMETERHOLD_ID_MASK: u8 = 0x02;

pub const VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT: u8 = 0x29;
/* Device default 7-bit I2C address */
pub const VL53L1_OSC_FREQUENCY: u8 = 0x00;
pub const VL53L1_OSC_TRIM_DEFAULT: u8 = 0x00;
pub const VL53L1_OSC_FREQ_SET_DEFAULT: u8 = 0x00;

pub const VL53L1_RANGE_HISTOGRAM_REF: u8 = 0x08;
pub const VL53L1_RANGE_HISTOGRAM_RET: u8 = 0x10;
pub const VL53L1_RANGE_HISTOGRAM_BOTH: u8 = 0x18;
pub const VL53L1_RANGE_HISTOGRAM_INIT: u8 = 0x20;
pub const VL53L1_RANGE_VHV_INIT: u8 = 0x40;

/* Result Status */
pub const VL53L1_RESULT_RANGE_STATUS: u8 = 0x1F;

/* */
pub const VL53L1_SYSTEM__SEED_CONFIG__MANUAL: u8 = 0x00;
pub const VL53L1_SYSTEM__SEED_CONFIG__STANDARD: u8 = 0x01;
pub const VL53L1_SYSTEM__SEED_CONFIG__EVEN_UPDATE_ONLY: u8 = 0x02;

/* Interrupt Config */
pub const VL53L1_INTERRUPT_CONFIG_LEVEL_LOW: u8 = 0x00;
pub const VL53L1_INTERRUPT_CONFIG_LEVEL_HIGH: u8 = 0x01;
pub const VL53L1_INTERRUPT_CONFIG_OUT_OF_WINDOW: u8 = 0x02;
pub const VL53L1_INTERRUPT_CONFIG_IN_WINDOW: u8 = 0x03;
pub const VL53L1_INTERRUPT_CONFIG_NEW_SAMPLE_READY: u8 = 0x20;

/* Interrupt Clear */
pub const VL53L1_CLEAR_RANGE_INT: u8 = 0x01;
pub const VL53L1_CLEAR_ERROR_INT: u8 = 0x02;

/* Sequence Config */
pub const VL53L1_SEQUENCE_VHV_EN: u8 = 0x01;
pub const VL53L1_SEQUENCE_PHASECAL_EN: u8 = 0x02;
pub const VL53L1_SEQUENCE_REFERENCE_PHASE_EN: u8 = 0x04;
pub const VL53L1_SEQUENCE_DSS1_EN: u8 = 0x08;
pub const VL53L1_SEQUENCE_DSS2_EN: u8 = 0x10;
pub const VL53L1_SEQUENCE_MM1_EN: u8 = 0x20;
pub const VL53L1_SEQUENCE_MM2_EN: u8 = 0x40;
pub const VL53L1_SEQUENCE_RANGE_EN: u8 = 0x80;

/* defines for DSS__ROI_CONTROL */
pub const VL53L1_DSS_CONTROL__ROI_SUBTRACT: u8 = 0x20;
pub const VL53L1_DSS_CONTROL__ROI_INTERSECT: u8 = 0x10;

pub const VL53L1_DSS_CONTROL__MODE_DISABLED: u8 = 0x00;
pub const VL53L1_DSS_CONTROL__MODE_TARGET_RATE: u8 = 0x01;
pub const VL53L1_DSS_CONTROL__MODE_EFFSPADS: u8 = 0x02;
pub const VL53L1_DSS_CONTROL__MODE_BLOCKSELECT: u8 = 0x03;

/* SPAD Readout defines
*
* 7:6 - SPAD_IN_SEL_REF
* 5:4 - SPAD_IN_SEL_RTN
*   2 - SPAD_PS_BYPASS
*   0 - SPAD_EN_PULSE_EXTENDER
*/

pub const VL53L1_RANGING_CORE__SPAD_READOUT__STANDARD: u8 = 0x45;
pub const VL53L1_RANGING_CORE__SPAD_READOUT__RETURN_ARRAY_ONLY: u8 = 0x05;
pub const VL53L1_RANGING_CORE__SPAD_READOUT__REFERENCE_ARRAY_ONLY: u8 = 0x55;
pub const VL53L1_RANGING_CORE__SPAD_READOUT__RETURN_SPLIT_ARRAY: u8 = 0x25;
pub const VL53L1_RANGING_CORE__SPAD_READOUT__CALIB_PULSES: u8 = 0xF5;

pub const VL53L1_LASER_SAFETY__KEY_VALUE: u8 = 0x6C;

/* Range Status defines
*
*   7 - GPH ID
*   6 - Min threshold hit
*   5 - Max threshold hit
* 4:0 - Range Status
*/

pub const VL53L1_RANGE_STATUS__RANGE_STATUS_MASK: u8 = 0x1F;
pub const VL53L1_RANGE_STATUS__MAX_THRESHOLD_HIT_MASK: u8 = 0x20;
pub const VL53L1_RANGE_STATUS__MIN_THRESHOLD_HIT_MASK: u8 = 0x40;
pub const VL53L1_RANGE_STATUS__GPH_ID_RANGE_STATUS_MASK: u8 = 0x80;

/* Interrupt Status defines
*
*   5 - GPH ID
* 4:3 - Interrupt Error Status
* 2:0 - Interrupt Status
*/

pub const VL53L1_INTERRUPT_STATUS__INT_STATUS_MASK: u8 = 0x07;
pub const VL53L1_INTERRUPT_STATUS__INT_ERROR_STATUS_MASK: u8 = 0x18;
pub const VL53L1_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK: u8 = 0x20;
