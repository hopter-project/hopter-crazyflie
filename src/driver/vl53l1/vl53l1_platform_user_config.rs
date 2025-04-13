/*******************************************************************************
Copyright (C) 2015, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/*
 * @file  vl53l1_platform_user_config.h
 *
 * @brief EwokPlus compile time user modifiable configuration
 */
#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
pub const VL53L1_BYTES_PER_WORD: u32 = 2;
pub const VL53L1_BYTES_PER_DWORD: u32 = 4;

/* Define polling delays */
pub const VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS: u32 = 500;
pub const VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS: u32 = 2000;
pub const VL53L1_TEST_COMPLETION_POLLING_TIMEOUT_MS: u32 = 60000;

pub const VL53L1_POLLING_DELAY_MS: u32 = 1;

/* Define LLD TuningParms Page Base Address
* - Part of Patch_AddedTuningParms_11761
*/
pub const VL53L1_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS: u16 = 0x8000;
pub const VL53L1_TUNINGPARM_PRIVATE_PAGE_BASE_ADDRESS: u16 = 0xC000;

pub const VL53L1_GAIN_FACTOR__STANDARD_DEFAULT: u16 = 0x0800;
/*  Default standard ranging gain correction factor
1.11 format. 1.0 = 0x0800, 0.980 = 0x07D7 */

pub const VL53L1_OFFSET_CAL_MIN_EFFECTIVE_SPADS: u16 = 0x0500;
/* Lower Limit for the  MM1 effective SPAD count during offset
calibration Format 8.8 0x0500 -> 5.0 effective SPADs */

pub const VL53L1_OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS: u16 = 0x1900;
/* Max Limit for the pre range peak rate during offset
 calibration Format 9.7 0x1900 -> 50.0 Mcps.
If larger then in pile up */

pub const VL53L1_OFFSET_CAL_MAX_SIGMA_MM: u16 = 0x0040;
/* Max sigma estimate limit during offset calibration
 Check applies to pre-range, mm1 and mm2 ranges
Format 14.2 0x0040 -> 16.0mm. */

pub const VL53L1_MAX_USER_ZONES: usize = 1;
/* Max number of user Zones - maximal limitation from
FW stream divide - value of 254 */

pub const VL53L1_MAX_RANGE_RESULTS: usize = 2;
/* Allocates storage for return and reference restults */

pub const VL53L1_MAX_STRING_LENGTH: usize = 512;

pub const VL53L1_USE_EMPTY_STRING: bool = false;
pub const VL53L1_NOCALIB: bool = false;
