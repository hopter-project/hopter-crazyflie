#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use alloc::borrow::ToOwned;

use super::{
    vl53l1_api::*, vl53l1_api_core::VL53L1_copy_sys_and_core_results_to_range_results,
    vl53l1_api_preset_modes::*, vl53l1_core::*, vl53l1_core_support::*, vl53l1_def::*,
    vl53l1_error_codes::*, vl53l1_ll_def::*, vl53l1_ll_device::*, vl53l1_platform_user_config::*,
    vl53l1_preset_setup::*, vl53l1_register_funcs::*, vl53l1_register_map::*,
    vl53l1_register_settings::*, vl53l1_register_structs::*, vl53l1_tuning_parm_defaults::*,
};

type vl53l1_read_fptr_t = fn(u16, &mut [u8], u32) -> i8;
type vl53l1_write_fptr_t = fn(u16, &[u8], u32) -> i8;
type vl53l1_delay_ms = fn(u32);

pub struct VL53L1_Dev_t {
    pub Data: VL53L1_DevData_t,
    /* Low Level Driver data structure */
    pub i2c_slave_address: u8,
    /* i2c device address user specific field */
    pub comms_type: u8,
    /* Type of comms : VL53L1_I2C or VL53L1_SPI */
    pub comms_speed_khz: u16,
    /* Comms speed [kHz] : typically 400kHz for I2C  */
    pub new_data_ready_poll_duration_ms: u32,
    /* New data ready poll duration in ms - for debug */
    // pub millis: vl53l1_timestamp_fptr_t,
}

impl VL53L1_Dev_t {
    pub fn init(&mut self) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        self.comms_type = VL53L1_I2C;
        self.comms_speed_khz = 400;
        status = self.VL53L1_software_reset();
        if status != VL53L1_ERROR_NONE {
            dbg_println!("VL53L1 not found");
            return status;
        }
        status = self.VL53L1_DataInit();
        status |= self.VL53L1_StaticInit();
        if status != VL53L1_ERROR_NONE {
            dbg_println!("VL53L1 init [FAILED]");
            return status;
        }

        status = self.VL53L1_StopMeasurement();
        status |= self.VL53L1_SetPresetMode(VL53L1_PRESETMODE_LITE_RANGING);
        status |= self.VL53L1_SetDistanceMode(VL53L1_DISTANCEMODE_SHORT);
        status |= self.VL53L1_SetMeasurementTimingBudgetMicroSeconds(10_000);
        status |= self.VL53L1_StartMeasurement();

        if status != VL53L1_ERROR_NONE {
            dbg_println!("VL53L1 config [FAILED]");
            return status;
        }
        dbg_println!("VL53L1 init [OK]");
        return status;
    }

    pub fn VL53L1_WaitMeasurementDataReady(&mut self) -> VL53L1_Error {
        self.VL53L1_poll_for_range_completion(VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS)
    }

    pub fn VL53L1_GetRangingMeasurementData(
        &mut self,
        pRangingMeasurementData: &mut VL53L1_RangingMeasurementData_t,
    ) -> VL53L1_Error {
        let mut results = VL53L1_range_results_t::default();
        let mut status =
            self.VL53L1_get_device_results(VL53L1_DEVICERESULTSLEVEL_FULL, &mut results);
        if status == VL53L1_ERROR_NONE {
            pRangingMeasurementData.StreamCount = results.stream_count;
            /* in case of lite ranging or autonomous the following function
             * returns index = 0
             */
            let presults_data = &mut results.data[0];
            status = self.SetSimpleData(
                1,
                results.device_status,
                presults_data,
                pRangingMeasurementData,
            );
        }
        return status;
    }

    pub fn VL53L1_ClearInterruptAndStartMeasurement(&mut self) -> VL53L1_Error {
        let DeviceMeasurementMode = self.Data.LLData.measurement_mode;
        self.VL53L1_clear_interrupt_and_enable_next_range(DeviceMeasurementMode)
    }

    pub fn VL53L1_clear_interrupt_and_enable_next_range(
        &mut self,
        measurement_mode: u8,
    ) -> VL53L1_Error {
        self.VL53L1_init_and_start_range(measurement_mode, VL53L1_DEVICECONFIGLEVEL_GENERAL_ONWARDS)
    }

    pub fn SetSimpleData(
        &mut self,
        active_results: u8,
        device_status: u8,
        presults_data: &mut VL53L1_range_data_t,
        pRangeData: &mut VL53L1_RangingMeasurementData_t,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut FilteredRangeStatus = 0_u8;
        let mut SigmaLimitflag = 0_u8;
        let mut SignalLimitflag = 0_u8;
        let mut Temp8Enable = 0_u8;
        let mut Temp8 = 0_u8;
        let mut AmbientRate = 0_u32;
        let mut SignalRate = 0_u32;
        let mut TempFix1616 = 0_u32;
        let mut LimitCheckValue = 0_u32;
        let mut Range = 0_i32;

        pRangeData.TimeStamp = presults_data.time_stamp;
        FilteredRangeStatus = presults_data.range_status & 0x1F;
        pRangeData.RangeQualityLevel =
            ComputeRQL(active_results, FilteredRangeStatus, presults_data);
        SignalRate = VL53L1_FIXPOINT97TOFIXPOINT1616(presults_data.peak_signal_count_rate_mcps);
        pRangeData.SignalRateRtnMegaCps = SignalRate;

        AmbientRate = VL53L1_FIXPOINT97TOFIXPOINT1616(presults_data.ambient_count_rate_mcps);
        pRangeData.AmbientRateRtnMegaCps = AmbientRate;

        pRangeData.EffectiveSpadRtnCount = presults_data.actual_effective_spads;

        TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(presults_data.sigma_mm);
        pRangeData.SigmaMilliMeter = TempFix1616;

        pRangeData.RangeMilliMeter = presults_data.median_range_mm;
        pRangeData.RangeFractionalPart = 0;
        /* Treat device error status first */
        match device_status {
            VL53L1_DEVICEERROR_MULTCLIPFAIL
            | VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE
            | VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE
            | VL53L1_DEVICEERROR_NOVHVVALUEFOUND => {
                pRangeData.RangeStatus = VL53L1_RANGESTATUS_HARDWARE_FAIL;
            }
            VL53L1_DEVICEERROR_USERROICLIP => {
                pRangeData.RangeStatus = VL53L1_RANGESTATUS_MIN_RANGE_FAIL
            }
            _ => pRangeData.RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID,
        }

        /* Now deal with range status according to the ranging preset */
        if pRangeData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID {
            pRangeData.RangeStatus = ConvertStatusLite(FilteredRangeStatus);
        }

        /* Update current Limit Check */
        TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(presults_data.sigma_mm);
        self.Data.CurrentParameters.LimitChecksCurrent
            [VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE as usize] = TempFix1616;

        TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(presults_data.peak_signal_count_rate_mcps);
        self.Data.CurrentParameters.LimitChecksCurrent
            [VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE as usize] = TempFix1616;

        /* Update Limit Check Status */
        /* Sigma */
        self.VL53L1_GetLimitCheckValue(VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, &mut LimitCheckValue);
        SigmaLimitflag = if FilteredRangeStatus == VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK {
            1
        } else {
            0
        };

        self.VL53L1_GetLimitCheckEnable(VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, &mut Temp8Enable);
        Temp8 = if Temp8Enable == 1 && SigmaLimitflag == 1 {
            1
        } else {
            0
        };
        self.Data.CurrentParameters.LimitChecksCurrent
            [VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE as usize] = Temp8 as u32;

        /* Signal Rate */
        self.VL53L1_GetLimitCheckValue(
            VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            &mut LimitCheckValue,
        );
        SignalLimitflag = if FilteredRangeStatus == VL53L1_DEVICEERROR_MSRCNOTARGET {
            1
        } else {
            0
        };

        self.VL53L1_GetLimitCheckEnable(
            VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
            &mut Temp8Enable,
        );
        Temp8 = if Temp8Enable == 1 && SignalLimitflag == 1 {
            1
        } else {
            0
        };
        self.Data.CurrentParameters.LimitChecksCurrent
            [VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE as usize] = Temp8 as u32;

        Range = pRangeData.RangeMilliMeter as i32;
        if pRangeData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID && Range < 0 {
            if Range < BDTABLE[VL53L1_Tuning_t::VL53L1_TUNING_PROXY_MIN as usize] {
                pRangeData.RangeStatus = VL53L1_RANGESTATUS_RANGE_INVALID;
            } else {
                pRangeData.RangeStatus = 0;
            }
        }
        return status;
    }

    pub fn VL53L1_GetLimitCheckEnable(
        &mut self,
        LimitCheckId: u16,
        pLimitCheckEnable: &mut u8,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;

        if LimitCheckId as usize >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS {
            status = VL53L1_ERROR_INVALID_PARAMS;
            *pLimitCheckEnable = 0;
        } else {
            *pLimitCheckEnable =
                self.Data.CurrentParameters.LimitChecksEnable[LimitCheckId as usize];
        }

        return status;
    }

    pub fn VL53L1_GetLimitCheckValue(
        &mut self,
        LimitCheckId: u16,
        pLimitCheckValue: &mut u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;

        let mut TempFix1616 = 0_u32;
        let mut SigmaThresh = 0_u16;
        let mut MinCountRate = 0_u16;

        match LimitCheckId {
            VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE => {
                status = self.VL53L1_get_lite_sigma_threshold(&mut SigmaThresh);
                TempFix1616 = VL53L1_FIXPOINT142TOFIXPOINT1616(SigmaThresh);
            }
            VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE => {
                status = self.VL53L1_get_lite_min_count_rate(&mut MinCountRate);
                TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(MinCountRate);
            }
            _ => status = VL53L1_ERROR_INVALID_PARAMS,
        }

        if status == VL53L1_ERROR_NONE {
            if TempFix1616 == 0 {
                TempFix1616 = self.Data.CurrentParameters.LimitChecksValue[LimitCheckId as usize];
                *pLimitCheckValue = TempFix1616;
                self.Data.CurrentParameters.LimitChecksEnable[LimitCheckId as usize] = 0;
            } else {
                *pLimitCheckValue = TempFix1616;
                self.Data.CurrentParameters.LimitChecksEnable[LimitCheckId as usize] = 1;
            }
        }
        return status;
    }

    pub fn VL53L1_get_lite_min_count_rate(&self, plite_mincountrate: &mut u16) -> VL53L1_Error {
        /*
         * Gets the Min Count Rate value for Lite Mode
         *
         * (fixed point 9.7 Mcps)
         */
        let pdev = &self.Data.LLData;
        *plite_mincountrate = pdev.tim_cfg.range_config__min_count_rate_rtn_limit_mcps;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_lite_sigma_threshold(&self, plite_sigma: &mut u16) -> VL53L1_Error {
        /*
         * Gets the Sigma Threshold value for Lite Mode
         *
         * (fixed point 14.2)
         */
        let pdev = &self.Data.LLData;
        *plite_sigma = pdev.tim_cfg.range_config__sigma_thresh;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_device_results(
        &mut self,
        device_results_level: VL53L1_DeviceResultsLevel,
        prange_results: &mut VL53L1_range_results_t,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;

        status = self.VL53L1_get_measurement_results(device_results_level);
        {
            let pdev = &mut self.Data.LLData;
            let mut pres = &mut self.Data.llresults;
            if status == VL53L1_ERROR_NONE {
                status = VL53L1_copy_sys_and_core_results_to_range_results(
                    pdev.gain_cal.standard_ranging_gain_factor as i32,
                    &mut pdev.sys_results,
                    &mut pdev.core_results,
                    &mut pres.range_results,
                );
            }
        }

        /* Start Patch_LowPowerAutoMode */
        /* process results from first range of low power auto */
        if self.Data.LLData.low_power_auto_data.is_low_power_auto_mode == 1 {
            /* change to manual calibrations. Only needed on the
             * first range out  */
            if (status == VL53L1_ERROR_NONE)
                && (self
                    .Data
                    .LLData
                    .low_power_auto_data
                    .low_power_auto_range_count
                    == 0)
            {
                status = self.VL53L1_low_power_auto_setup_manual_calibration();
                self.Data
                    .LLData
                    .low_power_auto_data
                    .low_power_auto_range_count = 1;
            } else if (status == VL53L1_ERROR_NONE)
                && (self
                    .Data
                    .LLData
                    .low_power_auto_data
                    .low_power_auto_range_count
                    == 1)
            {
                self.Data
                    .LLData
                    .low_power_auto_data
                    .low_power_auto_range_count = 2;
            }
        }
        /* End Patch_LowPowerAutoMode */

        /* copy current state into results */
        let pdev = &self.Data.LLData;
        let pres = &mut self.Data.llresults;
        pres.range_results.cfg_device_state = pdev.ll_state.cfg_device_state;
        pres.range_results.rd_device_state = pdev.ll_state.rd_device_state;

        /* copy internal structure to supplied output pointer */
        *prange_results = pres.range_results;
        /*
         * Check LL driver and Device are in Sync
         * If not an error is raised
         */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_check_ll_driver_rd_state();
        }

        return status;
    }

    pub fn VL53L1_low_power_auto_setup_manual_calibration(&mut self) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        /* save original vhv configs */
        pdev.low_power_auto_data.saved_vhv_init = pdev.stat_nvm.vhv_config__init;
        pdev.low_power_auto_data.saved_vhv_timeout =
            pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound;
        /* disable VHV init */
        pdev.stat_nvm.vhv_config__init &= 0x7F;
        /* set loop bound to tuning param */
        pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound =
            (pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03)
                + (pdev.low_power_auto_data.vhv_loop_bound << 2);
        /* override phasecal */
        pdev.gen_cfg.phasecal_config__override = 0x01;
        pdev.low_power_auto_data.first_run_phasecal_result =
            pdev.dbg_results.phasecal_result__vcsel_start;
        pdev.gen_cfg.cal_config__vcsel_start = pdev.low_power_auto_data.first_run_phasecal_result;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_check_ll_driver_rd_state(&self) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let pdev = &self.Data.LLData;
        let pstate = &pdev.ll_state;
        let psys_results = &pdev.sys_results;
        let device_range_status =
            psys_results.result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK;
        let device_stream_count = psys_results.result__stream_count;
        /* load the correct GPH ID */
        let device_gph_id = (psys_results.result__interrupt_status
            & VL53L1_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK)
            >> 4;
        /* only apply checks in back to back mode */
        if pdev.sys_ctrl.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK
            == VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK
        {
            /* if read state is wait for GPH sync interrupt then check the
             * device returns a GPH range status value otherwise check that
             * the stream count matches
             *
             * In theory the stream count should zero for the GPH interrupt
             * but that is not the case after at abort ....
             */
            if pstate.rd_device_state == VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC {
                if device_range_status != VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY {
                    status = VL53L1_ERROR_GPH_SYNC_CHECK_FAIL;
                }
            } else {
                if pstate.rd_stream_count != device_stream_count {
                    status = VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL;
                }
            }
            /*
             * Check Read state GPH ID
             */
            if pstate.rd_gph_id != device_gph_id {
                status = VL53L1_ERROR_GPH_ID_CHECK_FAIL;
            }
        }
        return status;
    }

    pub fn VL53L1_get_measurement_results(
        &mut self,
        device_results_level: VL53L1_DeviceResultsLevel,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut buffer = [0_u8; 256];

        let i2c_index = VL53L1_SYSTEM_RESULTS_I2C_INDEX;
        let mut i2c_buffer_offset_bytes = 0_u16;
        let mut i2c_buffer_size_bytes = 0_u16;
        let mut start: usize = 0;
        let mut end: usize = 0;

        /* Determine multi byte read transaction size */
        match device_results_level {
            VL53L1_DEVICERESULTSLEVEL_FULL => {
                i2c_buffer_size_bytes = (VL53L1_DEBUG_RESULTS_I2C_INDEX
                    + VL53L1_DEBUG_RESULTS_I2C_SIZE_BYTES as u16)
                    - i2c_index;
            }
            VL53L1_DEVICERESULTSLEVEL_UPTO_CORE => {
                i2c_buffer_size_bytes = (VL53L1_CORE_RESULTS_I2C_INDEX
                    + VL53L1_CORE_RESULTS_I2C_SIZE_BYTES as u16)
                    - i2c_index;
            }
            _ => i2c_buffer_size_bytes = VL53L1_SYSTEM_RESULTS_I2C_SIZE_BYTES as u16,
        }

        /* Read Result Data */
        status = self.VL53L1_ReadMulti(i2c_index, &mut buffer[0..i2c_buffer_size_bytes as usize]);

        let pdev = &mut self.Data.LLData;
        let psystem_results = &mut pdev.sys_results;
        let pcore_results = &mut pdev.core_results;
        let pdebug_results = &mut pdev.dbg_results;
        /* Decode  I2C buffer */
        if device_results_level >= VL53L1_DEVICERESULTSLEVEL_FULL && status == VL53L1_ERROR_NONE {
            i2c_buffer_offset_bytes = VL53L1_DEBUG_RESULTS_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_DEBUG_RESULTS_I2C_SIZE_BYTES;
            status = VL53L1_i2c_decode_debug_results(&buffer[start..end], pdebug_results);
        }

        if device_results_level >= VL53L1_DEVICERESULTSLEVEL_UPTO_CORE
            && status == VL53L1_ERROR_NONE
        {
            i2c_buffer_offset_bytes = VL53L1_CORE_RESULTS_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_CORE_RESULTS_I2C_SIZE_BYTES;
            status = VL53L1_i2c_decode_core_results(&mut buffer[start..end], pcore_results);
        }

        if status == VL53L1_ERROR_NONE {
            i2c_buffer_offset_bytes = 0;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_SYSTEM_RESULTS_I2C_SIZE_BYTES;
            status = VL53L1_i2c_decode_system_results(&buffer[start..end], psystem_results);
        }

        return status;
    }

    pub fn VL53L1_poll_for_range_completion(&mut self, timeout_ms: u32) -> VL53L1_Error {
        let pdev = &self.Data.LLData;
        let gpio__mux_active_high_hv =
            pdev.stat_cfg.gpio_hv_mux__ctrl & VL53L1_DEVICEINTERRUPTLEVEL_ACTIVE_MASK;
        let interrupt_ready: u8;
        if gpio__mux_active_high_hv == VL53L1_DEVICEINTERRUPTLEVEL_ACTIVE_HIGH {
            interrupt_ready = 0x01;
        } else {
            interrupt_ready = 0x00;
        }

        self.VL53L1_WaitValueMaskEx(
            timeout_ms,
            VL53L1_GPIO__TIO_HV_STATUS,
            interrupt_ready,
            0x01,
            VL53L1_POLLING_DELAY_MS,
        )
    }

    pub fn VL53L1_software_reset(&mut self) -> VL53L1_Error {
        /* apply reset - note despite the name soft reset is active low! */
        let mut status = self.VL53L1_WrByte(VL53L1_SOFT_RESET, 0x00);
        /* wait for a while before releasing the reset */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WaitUs(VL53L1_SOFTWARE_RESET_DURATION_US);
        }
        /* release reset */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WrByte(VL53L1_SOFT_RESET, 0x01);
        }
        /* wait for firmware boot to complete */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_wait_for_boot_completion();
        }
        return status;
    }

    pub fn VL53L1_wait_for_boot_completion(&mut self) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        let mut status = VL53L1_ERROR_NONE;

        if pdev.wait_method == VL53L1_WAIT_METHOD_BLOCKING {
            status =
                self.VL53L1_poll_for_boot_completion(VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS);
        } else {
            let mut fw_ready = 0_u8;
            while fw_ready == 0x00 && status == VL53L1_ERROR_NONE {
                status = self.VL53L1_is_boot_complete(&mut fw_ready);
                if status == VL53L1_ERROR_NONE && fw_ready == 0x00 {
                    self.VL53L1_WaitMs(VL53L1_POLLING_DELAY_MS);
                }
            }
        }
        return status;
    }

    pub fn VL53L1_is_boot_complete(&mut self, pready: &mut u8) -> VL53L1_Error {
        let mut firmware__system_status = 0_u8;
        /* read current range interrupt state */
        let status =
            self.VL53L1_RdByte(VL53L1_FIRMWARE__SYSTEM_STATUS, &mut firmware__system_status);
        /* set *pready = 1 if new range data ready complete
         * zero otherwise
         */
        if firmware__system_status & 0x01 == 0x01 {
            *pready = 0x01;
            self.VL53L1_init_ll_driver_state(VL53L1_DEVICESTATE_SW_STANDBY);
        } else {
            *pready = 0x00;
            self.VL53L1_init_ll_driver_state(VL53L1_DEVICESTATE_FW_COLDBOOT);
        }
        return status;
    }

    pub fn VL53L1_poll_for_boot_completion(&mut self, timeout_ms: u32) -> VL53L1_Error {
        let mut status = self.VL53L1_WaitMs(VL53L1_FIRMWARE_BOOT_TIME_US);

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WaitValueMaskEx(
                timeout_ms,
                VL53L1_FIRMWARE__SYSTEM_STATUS,
                0x01,
                0x01,
                VL53L1_POLLING_DELAY_MS,
            );
        }

        if status == VL53L1_ERROR_NONE {
            self.VL53L1_init_ll_driver_state(VL53L1_DEVICESTATE_SW_STANDBY);
        }
        return status;
    }

    pub fn VL53L1_RdByte(&mut self, index: u16, data: &mut u8) -> VL53L1_Error {
        let mut buffer = [0_u8; 1];
        let status = self.VL53L1_ReadMulti(index, &mut buffer);
        *data = buffer[0];
        return status;
    }

    pub fn VL53L1_WaitValueMaskEx(
        &mut self,
        timeout_ms: u32,
        index: u16,
        value: u8,
        mask: u8,
        poll_delay_ms: u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut byte_value = 0_u8;
        let mut start_time_ms = 0_u32;
        let mut found = false;

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_GetTickCount(&mut start_time_ms);
        }
        self.new_data_ready_poll_duration_ms = 0;

        while status == VL53L1_ERROR_NONE
            && self.new_data_ready_poll_duration_ms < timeout_ms
            && found == false
        {
            status = self.VL53L1_RdByte(index, &mut byte_value);

            if byte_value & mask == value {
                found = true;
            }

            if status == VL53L1_ERROR_NONE && found == false && poll_delay_ms > 0 {
                status = self.VL53L1_WaitMs(poll_delay_ms);
                self.new_data_ready_poll_duration_ms += poll_delay_ms;
            }
        }

        if status == VL53L1_ERROR_NONE && found == false {
            status = VL53L1_ERROR_TIME_OUT;
        }
        return status;
    }

    pub fn VL53L1_GetTickCount(&mut self, ptime_ms: &mut u32) -> VL53L1_Error {
        *ptime_ms = 0;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_WaitUs(&mut self, wait_us: u32) -> VL53L1_Error {
        let wait_ms = (wait_us + 500) / 1000;
        if wait_ms == 0 {
            hopter::time::sleep_ms(1);
        } else {
            hopter::time::sleep_ms(wait_ms);
        }
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_WaitMs(&mut self, wait_ms: u32) -> VL53L1_Error {
        hopter::time::sleep_ms(wait_ms);
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_GetDeviceInfo(
        &mut self,
        pVL53L1_DeviceInfo: &mut VL53L1_DeviceInfo_t,
    ) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        let revision_id = pdev.nvm_copy_data.identification__revision_id;
        pVL53L1_DeviceInfo.ProductType = pdev.nvm_copy_data.identification__module_type;
        pVL53L1_DeviceInfo.ProductRevisionMajor = 1;
        pVL53L1_DeviceInfo.ProductRevisionMinor = (revision_id & 0xF0) >> 4;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_StaticInit(&mut self) -> VL53L1_Error {
        self.Data.PalState = VL53L1_STATE_IDLE;
        self.Data.LLData.measurement_mode = VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK;

        /* ticket 472728 fix */
        let status = self.VL53L1_SetPresetMode(VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS);
        self.Data.CurrentParameters.PresetMode = VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS;
        /* end of ticket 472728 fix */
        return status;
    }

    pub fn VL53L1_StopMeasurement(&mut self) -> VL53L1_Error {
        let mut status = self.VL53L1_stop_range();
        if self.Data.LLData.measurement_mode != VL53L1_DEVICEMEASUREMENTMODE_TIMED {
            status = self.VL53L1_UnloadPatch();
        }
        /* Set PAL State to Idle */
        if status == VL53L1_ERROR_NONE {
            self.Data.PalState = VL53L1_STATE_IDLE;
        }
        return status;
    }

    pub fn VL53L1_StartMeasurement(&mut self) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let DeviceMeasurementMode = self.Data.LLData.measurement_mode;
        let CurrPalState = self.Data.PalState;
        let mut MTBus = 0_u32;
        let mut IMPms = 0_u32;

        if DeviceMeasurementMode != VL53L1_DEVICEMEASUREMENTMODE_TIMED {
            self.VL53L1_LoadPatch();
        }

        match CurrPalState {
            VL53L1_STATE_IDLE => status = VL53L1_ERROR_NONE,
            VL53L1_STATE_POWERDOWN
            | VL53L1_STATE_WAIT_STATICINIT
            | VL53L1_STATE_STANDBY
            | VL53L1_STATE_RUNNING
            | VL53L1_STATE_RESET
            | VL53L1_STATE_UNKNOWN
            | VL53L1_STATE_ERROR => status = VL53L1_ERROR_INVALID_COMMAND,
            _ => status = VL53L1_ERROR_UNDEFINED,
        }

        /* Check timing configuration between timing budget and
         * inter measurement period */
        if status == VL53L1_ERROR_NONE
            && DeviceMeasurementMode == VL53L1_DEVICEMEASUREMENTMODE_TIMED
        {
            self.VL53L1_GetMeasurementTimingBudgetMicroSeconds(&mut MTBus);
            /* convert timing budget in ms */
            MTBus /= 1000;
            self.VL53L1_GetInterMeasurementPeriodMilliSeconds(&mut IMPms);
            /* trick to get rid of compiler "set but not used" warning */
            if IMPms < MTBus + 4 {
                status = VL53L1_ERROR_INVALID_PARAMS;
            }
        }

        if status == VL53L1_ERROR_NONE {
            status = self
                .VL53L1_init_and_start_range(DeviceMeasurementMode, VL53L1_DEVICECONFIGLEVEL_FULL);
        }
        /* Set PAL State to Running */
        if status == VL53L1_ERROR_NONE {
            self.Data.PalState = VL53L1_STATE_RUNNING;
        }

        return status;
    }

    pub fn VL53L1_init_and_start_range(
        &mut self,
        measurement_mode: u8,
        device_config_level: VL53L1_DeviceConfigLevel,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut buffer = [0_u8; 256];
        let mut device_config_level = device_config_level;

        let mut i2c_index = 0_u16;
        let mut i2c_buffer_offset_bytes = 0_u16;
        let mut i2c_buffer_size_bytes = 0_u16;

        let pdev = &mut self.Data.LLData;

        /* save measurement mode */
        pdev.measurement_mode = measurement_mode;

        /* Merge measurement mode with mode_start */
        pdev.sys_ctrl.system__mode_start = (pdev.sys_ctrl.system__mode_start
            & VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK)
            | measurement_mode;
        /* copy in rit from xtalk config */
        pdev.stat_cfg.algo__range_ignore_threshold_mcps =
            pdev.xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps;

        /* Start Patch_LowPowerAutoMode */
        /* doing this ensures stop_range followed by a get_device_results does
        	* not mess up the counters */
        if pdev.low_power_auto_data.low_power_auto_range_count == 0xFF {
            pdev.low_power_auto_data.low_power_auto_range_count = 0x0;
        }

        /* For Presence. Override threshold config */
        if (pdev.low_power_auto_data.is_low_power_auto_mode == 1)
            && (pdev.low_power_auto_data.low_power_auto_range_count == 0)
        {
            /* save interrupt config */
            pdev.low_power_auto_data.saved_interrupt_config =
                pdev.gen_cfg.system__interrupt_config_gpio;
            /* set intr_new_measure_ready */
            pdev.gen_cfg.system__interrupt_config_gpio = 1 << 5;
            /* check MM1/MM2 disabled? */
            if (pdev.dyn_cfg.system__sequence_config
                & (VL53L1_SEQUENCE_MM1_EN | VL53L1_SEQUENCE_MM2_EN))
                == 0x0
            {
                pdev.customer.algo__part_to_part_range_offset_mm =
                    pdev.customer.mm_config__outer_offset_mm * 4;
            } else {
                pdev.customer.algo__part_to_part_range_offset_mm = 0x0;
            }

            /* make sure config gets written out */
            if device_config_level < VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS {
                device_config_level = VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS;
            }
        }

        if (pdev.low_power_auto_data.is_low_power_auto_mode == 1)
            && (pdev.low_power_auto_data.low_power_auto_range_count == 1)
        {
            /* restore interrupt config */
            pdev.gen_cfg.system__interrupt_config_gpio =
                pdev.low_power_auto_data.saved_interrupt_config;

            /* make sure config gets written out including VHV config */
            device_config_level = VL53L1_DEVICECONFIGLEVEL_FULL;
        }

        /* End Patch_LowPowerAutoMode */

        /*
        	* Determine Initial I2C index
        	*/
        match device_config_level {
            VL53L1_DEVICECONFIGLEVEL_FULL => i2c_index = VL53L1_STATIC_NVM_MANAGED_I2C_INDEX,
            VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS => {
                i2c_index = VL53L1_CUSTOMER_NVM_MANAGED_I2C_INDEX
            }
            VL53L1_DEVICECONFIGLEVEL_STATIC_ONWARDS => i2c_index = VL53L1_STATIC_CONFIG_I2C_INDEX,
            VL53L1_DEVICECONFIGLEVEL_GENERAL_ONWARDS => i2c_index = VL53L1_GENERAL_CONFIG_I2C_INDEX,
            VL53L1_DEVICECONFIGLEVEL_TIMING_ONWARDS => i2c_index = VL53L1_TIMING_CONFIG_I2C_INDEX,
            VL53L1_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS => i2c_index = VL53L1_DYNAMIC_CONFIG_I2C_INDEX,
            _ => i2c_index = VL53L1_SYSTEM_CONTROL_I2C_INDEX,
        }

        /* I2C Buffer size */
        i2c_buffer_size_bytes = VL53L1_SYSTEM_CONTROL_I2C_INDEX
            + VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES as u16
            - i2c_index;
        /* Initialize buffer */

        let pstatic_nvm = &mut pdev.stat_nvm;
        let pcustomer_nvm = &mut pdev.customer;
        let pstatic = &mut pdev.stat_cfg;
        let pgeneral = &mut pdev.gen_cfg;
        let ptiming = &mut pdev.tim_cfg;
        let pdynamic = &mut pdev.dyn_cfg;
        let psystem = &pdev.sys_ctrl;
        let pstate = &pdev.ll_state;

        let mut start: usize;
        let mut end: usize;
        /* Build I2C buffer */
        if device_config_level >= VL53L1_DEVICECONFIGLEVEL_FULL {
            i2c_buffer_offset_bytes = VL53L1_STATIC_NVM_MANAGED_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_STATIC_NVM_MANAGED_I2C_SIZE_BYTES;
            status = VL53L1_i2c_encode_static_nvm_managed(pstatic_nvm, &mut buffer[start..end]);
        }

        if device_config_level >= VL53L1_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS
            && status == VL53L1_ERROR_NONE
        {
            i2c_buffer_offset_bytes = VL53L1_CUSTOMER_NVM_MANAGED_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES;
            status = VL53L1_i2c_encode_customer_nvm_managed(pcustomer_nvm, &mut buffer[start..end]);
        }

        if device_config_level >= VL53L1_DEVICECONFIGLEVEL_STATIC_ONWARDS
            && status == VL53L1_ERROR_NONE
        {
            i2c_buffer_offset_bytes = VL53L1_STATIC_CONFIG_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_STATIC_CONFIG_I2C_SIZE_BYTES;
            status = VL53L1_i2c_encode_static_config(pstatic, &mut buffer[start..end]);
        }

        if device_config_level >= VL53L1_DEVICECONFIGLEVEL_GENERAL_ONWARDS
            && status == VL53L1_ERROR_NONE
        {
            i2c_buffer_offset_bytes = VL53L1_GENERAL_CONFIG_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_GENERAL_CONFIG_I2C_SIZE_BYTES;
            status = VL53L1_i2c_encode_general_config(pgeneral, &mut buffer[start..end]);
        }

        if device_config_level >= VL53L1_DEVICECONFIGLEVEL_TIMING_ONWARDS
            && status == VL53L1_ERROR_NONE
        {
            i2c_buffer_offset_bytes = VL53L1_TIMING_CONFIG_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_TIMING_CONFIG_I2C_SIZE_BYTES;
            status = VL53L1_i2c_encode_timing_config(ptiming, &mut buffer[start..end]);
        }

        if device_config_level >= VL53L1_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS
            && status == VL53L1_ERROR_NONE
        {
            i2c_buffer_offset_bytes = VL53L1_DYNAMIC_CONFIG_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_DYNAMIC_CONFIG_I2C_SIZE_BYTES;
            if psystem.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK
                == VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK
            {
                pdynamic.system__grouped_parameter_hold_0 = pstate.cfg_gph_id | 0x01;
                pdynamic.system__grouped_parameter_hold_1 = pstate.cfg_gph_id | 0x01;
                pdynamic.system__grouped_parameter_hold = pstate.cfg_gph_id;
            }
            status = VL53L1_i2c_encode_dynamic_config(pdynamic, &mut buffer[start..end]);
        }

        if status == VL53L1_ERROR_NONE {
            i2c_buffer_offset_bytes = VL53L1_SYSTEM_CONTROL_I2C_INDEX - i2c_index;
            start = i2c_buffer_offset_bytes as usize;
            end = start + VL53L1_STATIC_CONFIG_I2C_SIZE_BYTES;
            status = VL53L1_i2c_encode_system_control(psystem, &mut buffer[start..end]);
        }

        /* Send I2C Buffer */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WriteMulti(i2c_index, &buffer[0..i2c_buffer_size_bytes as usize]);
        }

        /*
         * Update LL Driver State
         */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_update_ll_driver_rd_state();
        }
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_update_ll_driver_cfg_state();
        }

        return status;
    }

    pub fn VL53L1_update_ll_driver_cfg_state(&mut self) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        let pstate = &mut pdev.ll_state;

        if pdev.sys_ctrl.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_MODE_MASK == 0x00 {
            pstate.cfg_device_state = VL53L1_DEVICESTATE_SW_STANDBY;
            pstate.cfg_stream_count = 0;
            pstate.cfg_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
            pstate.cfg_timing_status = 0;
        } else {
            /*
             * implement configuration stream count
             */
            if pstate.cfg_stream_count == 0xFF {
                pstate.cfg_stream_count = 0x80;
            } else {
                pstate.cfg_stream_count += 1;
            }

            /*
             * Toggle grouped parameter hold ID
             */
            pstate.cfg_gph_id ^= VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;

            /*
             * Implement configuration state machine
             */
            match pstate.cfg_device_state {
                VL53L1_DEVICESTATE_SW_STANDBY => {
                    pstate.cfg_timing_status ^= 0x01;
                    pstate.cfg_stream_count = 1;
                    pstate.cfg_device_state = VL53L1_DEVICESTATE_RANGING_DSS_AUTO;
                }
                VL53L1_DEVICESTATE_RANGING_DSS_AUTO => {
                    pstate.cfg_timing_status ^= 0x01;
                }
                _ => {
                    pstate.cfg_device_state = VL53L1_DEVICESTATE_SW_STANDBY;
                    pstate.cfg_stream_count = 0;
                    pstate.cfg_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
                    pstate.cfg_timing_status = 0;
                }
            }
        }
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_update_ll_driver_rd_state(&mut self) -> VL53L1_Error {
        /*
         * State machine for read device state
         *
         * VL53L1_DEVICESTATE_SW_STANDBY
         * VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC
         * VL53L1_DEVICESTATE_RANGING_GATHER_DATA
         * VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA
         */
        let pdev = &mut self.Data.LLData;
        let pstate = &mut pdev.ll_state;
        if pdev.sys_ctrl.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_MODE_MASK == 0 {
            pstate.rd_device_state = VL53L1_DEVICESTATE_SW_STANDBY;
            pstate.rd_stream_count = 0;
            pstate.rd_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
            pstate.rd_timing_status = 0;
        } else {
            /*
             * implement read stream count
             */
            if pstate.rd_stream_count == 0xFF {
                pstate.rd_stream_count = 0x80;
            } else {
                pstate.rd_stream_count += 1;
            }
            /*
             * Toggle grouped parameter hold ID
             */

            pstate.rd_gph_id ^= VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;

            /* Ok now ranging  */
            match pstate.rd_device_state {
                VL53L1_DEVICESTATE_SW_STANDBY => {
                    if pdev.dyn_cfg.system__grouped_parameter_hold
                        & VL53L1_GROUPEDPARAMETERHOLD_ID_MASK
                        > 0
                    {
                        pstate.rd_device_state = VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC;
                    } else {
                        pstate.rd_device_state = VL53L1_DEVICESTATE_RANGING_GATHER_DATA;
                    }
                    pstate.rd_stream_count = 0;
                    pstate.rd_timing_status = 0;
                }
                VL53L1_DEVICESTATE_RANGING_WAIT_GPH_SYNC => {
                    pstate.rd_stream_count = 0;
                    pstate.rd_device_state = VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
                }
                VL53L1_DEVICESTATE_RANGING_GATHER_DATA => {
                    pstate.rd_device_state = VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
                }
                VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA => {
                    pstate.rd_timing_status ^= 0x01;
                    pstate.rd_device_state = VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA;
                }
                _ => {
                    pstate.rd_device_state = VL53L1_DEVICESTATE_SW_STANDBY;
                    pstate.rd_stream_count = 0;
                    pstate.rd_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
                    pstate.rd_timing_status = 0;
                }
            }
        }
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_GetInterMeasurementPeriodMilliSeconds(
        &mut self,
        pInterMeasurementPeriodMilliSeconds: &mut u32,
    ) -> VL53L1_Error {
        let mut adjustedIMP = 0_u32;
        let status = self.VL53L1_get_inter_measurement_period_ms(&mut adjustedIMP);
        /* Fix for Ticket 468205 actual measurement period shorter than set */
        adjustedIMP -= (adjustedIMP * 64) / 1000;
        *pInterMeasurementPeriodMilliSeconds = adjustedIMP;
        /* End of fix for Ticket 468205 */
        return status;
    }

    pub fn VL53L1_get_inter_measurement_period_ms(
        &self,
        pinter_measurement_period_ms: &mut u32,
    ) -> VL53L1_Error {
        let pdev = &self.Data.LLData;
        if pdev.dbg_results.result__osc_calibrate_val == 0 {
            return VL53L1_ERROR_DIVISION_BY_ZERO;
        }

        *pinter_measurement_period_ms = pdev.tim_cfg.system__intermeasurement_period
            / pdev.dbg_results.result__osc_calibrate_val as u32;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_GetMeasurementTimingBudgetMicroSeconds(
        &mut self,
        pMeasurementTimingBudgetMicroSeconds: &mut u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut Mm1Enabled = 0_u8;
        let mut Mm2Enabled = 0_u8;
        let mut MmTimeoutUs = 0_u32;
        let mut RangeTimeoutUs = 0_u32;
        let mut MeasTimingBdg = 0_u32;
        let mut PhaseCalTimeoutUs = 0_u32;
        let mut PresetMode = 0_u8;
        let mut TimingGuard = 0_u32;
        let mut vhv = 0_u32;
        let mut vhv_loops = 0_i32;
        *pMeasurementTimingBudgetMicroSeconds = 0;
        status = self.VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM1, &mut Mm1Enabled);
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM2, &mut Mm2Enabled);
        }

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_get_timeouts_us(
                &mut PhaseCalTimeoutUs,
                &mut MmTimeoutUs,
                &mut RangeTimeoutUs,
            );
        }

        if status == VL53L1_ERROR_NONE {
            PresetMode = self.Data.CurrentParameters.PresetMode;
            match PresetMode {
                VL53L1_PRESETMODE_LITE_RANGING => {
                    if Mm1Enabled == 1 || Mm2Enabled == 1 {
                        MeasTimingBdg = RangeTimeoutUs + 5000;
                    } else {
                        MeasTimingBdg = RangeTimeoutUs + 1000;
                    }
                }
                VL53L1_PRESETMODE_AUTONOMOUS => {
                    if Mm1Enabled == 1 || Mm2Enabled == 1 {
                        MeasTimingBdg = 2 * RangeTimeoutUs + 26600;
                    } else {
                        MeasTimingBdg = 2 * RangeTimeoutUs + 21600;
                    }
                }
                VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS => {
                    vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
                    self.VL53L1_get_tuning_parm(
                        VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND,
                        &mut vhv_loops,
                    );
                    if vhv_loops > 0 {
                        vhv = (vhv as i32 + vhv_loops * LOWPOWER_AUTO_VHV_LOOP_DURATION_US as i32)
                            as u32;
                    }
                    TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING
                        + LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING
                        + vhv;
                    MeasTimingBdg = 2 * RangeTimeoutUs + TimingGuard;
                }
                _ => status = VL53L1_ERROR_MODE_NOT_SUPPORTED,
            }
        }

        if status == VL53L1_ERROR_NONE {
            *pMeasurementTimingBudgetMicroSeconds = MeasTimingBdg;
        }
        return status;
    }

    pub fn VL53L1_SetDistanceMode(&mut self, DistanceMode: VL53L1_DistanceModes) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let PresetMode = self.Data.CurrentParameters.PresetMode;
        let mut inter_measurement_period_ms = 0_u32;
        let mut TimingBudget = 0_u32;
        let mut MmTimeoutUs = 0_u32;
        let mut PhaseCalTimeoutUs = 0_u32;
        let mut user_zone = VL53L1_user_zone_t::default();
        /* when the distance mode is valid:
         * Manual Mode: all modes
         * AUTO AUTO_LITE : LITE_RANGING, RANGING
         */
        if DistanceMode != VL53L1_DISTANCEMODE_SHORT
            && DistanceMode != VL53L1_DISTANCEMODE_MEDIUM
            && DistanceMode != VL53L1_DISTANCEMODE_LONG
        {
            return VL53L1_ERROR_INVALID_PARAMS;
        }

        status = self.VL53L1_get_user_zone(&mut user_zone);

        inter_measurement_period_ms = self.Data.LLData.inter_measurement_period_ms;
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_get_timeouts_us(
                &mut PhaseCalTimeoutUs,
                &mut MmTimeoutUs,
                &mut TimingBudget,
            );
        }

        if status == VL53L1_ERROR_NONE {
            status = self.SetPresetMode(PresetMode, DistanceMode, inter_measurement_period_ms);
        }

        if status == VL53L1_ERROR_NONE {
            self.Data.CurrentParameters.DistanceMode = DistanceMode;
            status = self.VL53L1_set_timeouts_us(PhaseCalTimeoutUs, MmTimeoutUs, TimingBudget);
        }

        if status == VL53L1_ERROR_NONE {
            self.Data.LLData.range_config_timeout_us = TimingBudget;
            status = self.VL53L1_set_user_zone(&user_zone);
        }
        return status;
    }

    pub fn VL53L1_LoadPatch(&mut self) -> VL53L1_Error {
        let mut status = self.VL53L1_WrByte(VL53L1_FIRMWARE__ENABLE, 0x00);
        let mut patch_power = 0_u8;
        let mut comms_buffer = [0_u8; 6];

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_enable_powerforce();
        }

        let patch_tuning = BDTABLE[VL53L1_Tuning_t::VL53L1_TUNING_PHASECAL_PATCH_POWER as usize];

        match patch_tuning {
            0 => patch_power = 0x00,
            1 => patch_power = 0x10,
            2 => patch_power = 0x20,
            3 => patch_power = 0x40,
            _ => patch_power = 0x00,
        }

        /* Set patch RAM offsets */
        if status == VL53L1_ERROR_NONE {
            /* Package up MultiByte transaction */
            comms_buffer[0] = 0x29;
            comms_buffer[1] = 0xC9;
            comms_buffer[2] = 0x0E;
            comms_buffer[3] = 0x40;
            comms_buffer[4] = 0x28;
            comms_buffer[5] = patch_power;
            /* 0x10 for 60ms, 0x20 for 240ms and 0x40 for 3580ms */
            status = self.VL53L1_WriteMulti(VL53L1_PATCH__ADDRESS_0, &comms_buffer);
        }

        /* Enable patch JMP patches */
        if status == VL53L1_ERROR_NONE {
            comms_buffer[0] = 0x00;
            comms_buffer[1] = 0x07;
            status = self.VL53L1_WriteMulti(VL53L1_PATCH__JMP_ENABLES, &comms_buffer[0..2]);
        }

        /* Enable patch DATA patches */
        if status == VL53L1_ERROR_NONE {
            comms_buffer[0] = 0x00;
            comms_buffer[1] = 0x07;
            status = self.VL53L1_WriteMulti(VL53L1_PATCH__DATA_ENABLES, &comms_buffer[0..2]);
        }

        /* Enable firmware patching */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WrByte(VL53L1_PATCH__CTRL, 0x01);
        }

        /* Enable Firmware */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WrByte(VL53L1_FIRMWARE__ENABLE, 0x01);
        }
        return status;
    }

    pub fn VL53L1_enable_powerforce(&mut self) -> VL53L1_Error {
        self.VL53L1_set_powerforce_register(0x01)
    }

    pub fn VL53L1_set_user_zone(&mut self, puser_zone: &VL53L1_user_zone_t) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        VL53L1_encode_row_col(
            puser_zone.y_centre,
            puser_zone.x_centre,
            &mut pdev.dyn_cfg.roi_config__user_roi_centre_spad,
        );
        VL53L1_encode_zone_size(
            puser_zone.width,
            puser_zone.height,
            &mut pdev.dyn_cfg.roi_config__user_roi_requested_global_xy_size,
        );
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_user_zone(&self, puser_zone: &mut VL53L1_user_zone_t) -> VL53L1_Error {
        let pdev = &self.Data.LLData;
        /* convert SPAD number into (row,col) location*/
        VL53L1_decode_row_col(
            pdev.dyn_cfg.roi_config__user_roi_centre_spad,
            &mut puser_zone.y_centre,
            &mut puser_zone.x_centre,
        );
        /* extract x and y sizes */
        VL53L1_decode_zone_size(
            pdev.dyn_cfg.roi_config__user_roi_requested_global_xy_size,
            &mut puser_zone.width,
            &mut puser_zone.height,
        );
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_stop_range(&mut self) -> VL53L1_Error {
        {
            let pdev = &mut self.Data.LLData;
            /* Merge ABORT mode with mode_start */
            pdev.sys_ctrl.system__mode_start = (pdev.sys_ctrl.system__mode_start
                & VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK)
                | VL53L1_DEVICEMEASUREMENTMODE_ABORT;
        }
        let status = self.VL53L1_set_system_control();
        {
            let pdev = &mut self.Data.LLData;
            /* Abort bit is auto clear so clear register group structure to match */
            pdev.sys_ctrl.system__mode_start =
                pdev.sys_ctrl.system__mode_start & VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK;
        }

        /* reset zone dynamic info */
        self.VL53L1_init_ll_driver_state(VL53L1_DEVICESTATE_SW_STANDBY);

        /* reset low power auto */
        let pdev = &mut self.Data.LLData;
        if pdev.low_power_auto_data.is_low_power_auto_mode == 1 {
            self.VL53L1_low_power_auto_data_stop_range();
        }
        return status;
    }

    pub fn VL53L1_low_power_auto_data_stop_range(&mut self) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        /* doing this ensures stop_range followed by a get_device_results does
         * not mess up the counters */
        pdev.low_power_auto_data.low_power_auto_range_count = 0xFF;

        pdev.low_power_auto_data.first_run_phasecal_result = 0;
        pdev.low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
        pdev.low_power_auto_data.dss__required_spads = 0;

        /* restore vhv configs */
        if pdev.low_power_auto_data.saved_vhv_init != 0 {
            pdev.stat_nvm.vhv_config__init = pdev.low_power_auto_data.saved_vhv_init;
        }

        if pdev.low_power_auto_data.saved_vhv_timeout != 0 {
            pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound =
                pdev.low_power_auto_data.saved_vhv_timeout;
        }

        /* remove phasecal override */
        pdev.gen_cfg.phasecal_config__override = 0x00;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_set_system_control(&mut self) -> VL53L1_Error {
        let mut pdata = &mut self.Data.LLData.sys_ctrl;
        let mut comms_buffer = [0u8; VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES];
        let mut status = VL53L1_i2c_encode_system_control(&pdata, &mut comms_buffer);
        if status == VL53L1_ERROR_NONE {
            status =
                self.VL53L1_WriteMulti(VL53L1_POWER_MANAGEMENT__GO1_POWER_FORCE, &comms_buffer);
        }
        return status;
    }

    pub fn VL53L1_WriteMulti(&mut self, index: u16, pdata: &[u8]) -> VL53L1_Error {
        crate::module::tof::i2c1_write(index, pdata, self.i2c_slave_address as u32)
    }

    pub fn VL53L1_UnloadPatch(&mut self) -> VL53L1_Error {
        /* Disable Firmware (allow full write access) */
        let mut status = self.VL53L1_WrByte(VL53L1_FIRMWARE__ENABLE, 0x00);
        /* Force GO1 off */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_disable_powerforce();
        }
        /* Disable firmware patching */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WrByte(VL53L1_PATCH__CTRL, 0x00);
        }
        /* Enable Firmware */
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_WrByte(VL53L1_FIRMWARE__ENABLE, 0x01);
        }
        return status;
    }

    pub fn VL53L1_disable_powerforce(&mut self) -> VL53L1_Error {
        self.VL53L1_set_powerforce_register(0x00)
    }

    pub fn VL53L1_set_powerforce_register(&mut self, value: u8) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        pdev.sys_ctrl.power_management__go1_power_force = value;
        self.VL53L1_WrByte(VL53L1_POWER_MANAGEMENT__GO1_POWER_FORCE, value)
    }

    pub fn VL53L1_WrByte(&mut self, index: u16, data: u8) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut comms_buffer = [0u8; 1];
        comms_buffer[0] = data;
        status = self.VL53L1_WriteMulti(index, &comms_buffer);
        return status;
    }

    pub fn VL53L1_SetPresetMode(&mut self, PresetMode: VL53L1_PresetModes) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        status = self.VL53L1_low_power_auto_data_init();
        status = self.SetPresetMode(PresetMode, VL53L1_DISTANCEMODE_LONG, 1000);

        if status == VL53L1_ERROR_NONE {
            if PresetMode == VL53L1_PRESETMODE_LITE_RANGING
                || PresetMode == VL53L1_PRESETMODE_AUTONOMOUS
                || PresetMode == VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS
            {
                status = self.VL53L1_SetMeasurementTimingBudgetMicroSeconds(41000);
            } else {
                status = self.VL53L1_SetMeasurementTimingBudgetMicroSeconds(33333);
            }
        }

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_SetInterMeasurementPeriodMilliSeconds(1000);
        }

        return status;
    }

    pub fn VL53L1_SetInterMeasurementPeriodMilliSeconds(
        &mut self,
        InterMeasurementPeriodMilliSeconds: u32,
    ) -> VL53L1_Error {
        /* Fix for Ticket 468205 actual measurement period shorter than set */
        let mut adjustedIMP = InterMeasurementPeriodMilliSeconds;
        adjustedIMP += (adjustedIMP * 64) / 1000;
        self.VL53L1_set_inter_measurement_period_ms(adjustedIMP)
    }

    pub fn VL53L1_SetMeasurementTimingBudgetMicroSeconds(
        &mut self,
        MeasurementTimingBudgetMicroSeconds: u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut MM1Enabled = 0_u8;
        let mut MM2Enabled = 0_u8;
        let mut TimingGuard = 0_u32;
        let mut divisor = 0_u32;
        let mut TimingBudget = 0_u32;
        let mut MMTimeoutUs = 0_u32;
        let mut PresetMode = VL53L1_PRESETMODE_LITE_RANGING;
        let mut PhaseCalTimeoutUs = 0_u32;
        let mut vhv = 0_u32;
        let mut vhv_loops = 0_i32;
        let mut FDAMaxTimingBudgetUs = FDA_MAX_TIMING_BUDGET_US;

        /* Timing budget is limited to 10 seconds */
        if MeasurementTimingBudgetMicroSeconds > 10000000 {
            return VL53L1_ERROR_INVALID_PARAMS;
        }

        status = self.VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM1, &mut MM1Enabled);
        status |= self.VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM2, &mut MM2Enabled);
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_get_timeouts_us(
                &mut PhaseCalTimeoutUs,
                &mut MMTimeoutUs,
                &mut TimingBudget,
            );
        }

        if status == VL53L1_ERROR_NONE {
            PresetMode = self.Data.CurrentParameters.PresetMode;
            divisor = 1;
            match PresetMode {
                VL53L1_PRESETMODE_LITE_RANGING => {
                    if MM1Enabled == 1 || MM2Enabled == 1 {
                        TimingGuard = 5000;
                    } else {
                        TimingGuard = 1000;
                    }
                }
                VL53L1_PRESETMODE_AUTONOMOUS => {
                    divisor = 2;
                    FDAMaxTimingBudgetUs *= 2;
                    if MM1Enabled == 1 || MM2Enabled == 1 {
                        TimingGuard = 26600;
                    } else {
                        TimingGuard = 21600;
                    }
                }
                VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS => {
                    divisor = 2;
                    FDAMaxTimingBudgetUs *= 2;
                    self.VL53L1_get_tuning_parm(
                        VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND,
                        &mut vhv_loops,
                    );
                    if vhv_loops > 0 {
                        vhv = (vhv as i32 + (vhv_loops * LOWPOWER_AUTO_VHV_LOOP_DURATION_US as i32))
                            as u32;
                    }
                    TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING
                        + LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING
                        + vhv;
                }
                _ => {
                    return VL53L1_ERROR_INVALID_PARAMS;
                }
            }

            if MeasurementTimingBudgetMicroSeconds <= TimingGuard {
                return VL53L1_ERROR_INVALID_PARAMS;
            } else {
                TimingBudget = MeasurementTimingBudgetMicroSeconds - TimingGuard;
            }

            if TimingBudget > FDAMaxTimingBudgetUs {
                return VL53L1_ERROR_INVALID_PARAMS;
            } else {
                TimingBudget /= divisor;
                status = self.VL53L1_set_timeouts_us(PhaseCalTimeoutUs, MMTimeoutUs, TimingBudget);
            }

            if status == VL53L1_ERROR_NONE {
                self.Data.LLData.range_config_timeout_us = TimingBudget;
            }
        }
        return status;
    }

    pub fn VL53L1_get_tuning_parm(
        &self,
        tuning_parm_key: VL53L1_TuningParms,
        ptuning_parm_value: &mut i32,
    ) -> VL53L1_Error {
        let pdev = &self.Data.LLData;
        match tuning_parm_key {
            VL53L1_TUNINGPARM_VERSION => {
                *ptuning_parm_value = pdev.tuning_parms.tp_tuning_parm_version as i32
            }
            VL53L1_TUNINGPARM_KEY_TABLE_VERSION => {
                *ptuning_parm_value = pdev.tuning_parms.tp_tuning_parm_key_table_version as i32
            }
            VL53L1_TUNINGPARM_LLD_VERSION => {
                *ptuning_parm_value = pdev.tuning_parms.tp_tuning_parm_lld_version as i32
            }
            VL53L1_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_consistency_lite_phase_tolerance as i32
            }
            VL53L1_TUNINGPARM_PHASECAL_TARGET => {
                *ptuning_parm_value = pdev.tuning_parms.tp_phasecal_target as i32
            }
            VL53L1_TUNINGPARM_LITE_CAL_REPEAT_RATE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_cal_repeat_rate as i32
            }
            VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR => {
                *ptuning_parm_value = pdev.gain_cal.standard_ranging_gain_factor as i32
            }
            VL53L1_TUNINGPARM_LITE_MIN_CLIP_MM => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_min_clip as i32
            }
            VL53L1_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_long_sigma_thresh_mm as i32
            }
            VL53L1_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_med_sigma_thresh_mm as i32
            }
            VL53L1_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_short_sigma_thresh_mm as i32
            }
            VL53L1_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_long_min_count_rate_rtn_mcps as i32
            }
            VL53L1_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_med_min_count_rate_rtn_mcps as i32
            }
            VL53L1_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_short_min_count_rate_rtn_mcps as i32
            }
            VL53L1_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_sigma_est_pulse_width_ns as i32
            }
            VL53L1_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_sigma_est_amb_width_ns as i32
            }
            VL53L1_TUNINGPARM_LITE_SIGMA_REF_MM => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_sigma_ref_mm as i32
            }
            VL53L1_TUNINGPARM_LITE_RIT_MULT => {
                *ptuning_parm_value = pdev.xtalk_cfg.crosstalk_range_ignore_threshold_mult as i32
            }
            VL53L1_TUNINGPARM_LITE_SEED_CONFIG => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_seed_cfg as i32
            }
            VL53L1_TUNINGPARM_LITE_QUANTIFIER => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_quantifier as i32
            }
            VL53L1_TUNINGPARM_LITE_FIRST_ORDER_SELECT => {
                *ptuning_parm_value = pdev.tuning_parms.tp_lite_first_order_select as i32
            }
            VL53L1_TUNINGPARM_LITE_XTALK_MARGIN_KCPS => {
                *ptuning_parm_value = pdev.xtalk_cfg.lite_mode_crosstalk_margin_kcps as i32
            }
            VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_init_phase_rtn_lite_long as i32
            }
            VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_init_phase_rtn_lite_med as i32
            }
            VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_init_phase_rtn_lite_short as i32
            }
            VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_init_phase_ref_lite_long as i32
            }
            VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_init_phase_ref_lite_med as i32
            }
            VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE => {
                *ptuning_parm_value = pdev.tuning_parms.tp_init_phase_ref_lite_short as i32
            }
            VL53L1_TUNINGPARM_TIMED_SEED_CONFIG => {
                *ptuning_parm_value = pdev.tuning_parms.tp_timed_seed_cfg as i32
            }
            VL53L1_TUNINGPARM_VHV_LOOPBOUND => {
                *ptuning_parm_value = pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound as i32
            }
            VL53L1_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE => {
                *ptuning_parm_value = pdev.refspadchar.device_test_mode as i32
            }
            VL53L1_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD => {
                *ptuning_parm_value = pdev.refspadchar.vcsel_period as i32
            }
            VL53L1_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US => {
                *ptuning_parm_value = pdev.refspadchar.timeout_us as i32
            }
            VL53L1_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS => {
                *ptuning_parm_value = pdev.refspadchar.target_count_rate_mcps as i32
            }
            VL53L1_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS => {
                *ptuning_parm_value = pdev.refspadchar.min_count_rate_limit_mcps as i32
            }
            VL53L1_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS => {
                *ptuning_parm_value = pdev.refspadchar.max_count_rate_limit_mcps as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS => {
                *ptuning_parm_value = pdev.offsetcal_cfg.dss_config__target_total_rate_mcps as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US => {
                *ptuning_parm_value = pdev.offsetcal_cfg.phasecal_config_timeout_us as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US => {
                *ptuning_parm_value = pdev.offsetcal_cfg.mm_config_timeout_us as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US => {
                *ptuning_parm_value = pdev.offsetcal_cfg.range_config_timeout_us as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES => {
                *ptuning_parm_value = pdev.offsetcal_cfg.pre_num_of_samples as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES => {
                *ptuning_parm_value = pdev.offsetcal_cfg.mm1_num_of_samples as i32
            }
            VL53L1_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES => {
                *ptuning_parm_value = pdev.offsetcal_cfg.mm2_num_of_samples as i32
            }
            VL53L1_TUNINGPARM_SPADMAP_VCSEL_PERIOD => {
                *ptuning_parm_value = pdev.ssc_cfg.vcsel_period as i32
            }
            VL53L1_TUNINGPARM_SPADMAP_VCSEL_START => {
                *ptuning_parm_value = pdev.ssc_cfg.vcsel_start as i32
            }
            VL53L1_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS => {
                *ptuning_parm_value = pdev.ssc_cfg.rate_limit_mcps as i32
            }
            VL53L1_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS => {
                *ptuning_parm_value = pdev.tuning_parms.tp_dss_target_lite_mcps as i32
            }
            VL53L1_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS => {
                *ptuning_parm_value = pdev.tuning_parms.tp_dss_target_timed_mcps as i32
            }
            VL53L1_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_phasecal_timeout_lite_us as i32
            }
            VL53L1_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_phasecal_timeout_timed_us as i32
            }
            VL53L1_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_mm_timeout_lite_us as i32
            }
            VL53L1_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_mm_timeout_timed_us as i32
            }
            VL53L1_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_range_timeout_lite_us as i32
            }
            VL53L1_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_range_timeout_timed_us as i32
            }
            VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND => {
                *ptuning_parm_value = pdev.low_power_auto_data.vhv_loop_bound as i32
            }
            VL53L1_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_mm_timeout_lpa_us as i32
            }
            VL53L1_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US => {
                *ptuning_parm_value = pdev.tuning_parms.tp_range_timeout_lpa_us as i32
            }
            _ => {
                *ptuning_parm_value = 0x7FFFFFFF;
                return VL53L1_ERROR_INVALID_PARAMS;
            }
        }
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_timeouts_us(
        &mut self,
        phasecal_config_timeout_us: &mut u32,
        mm_config_timeout_us: &mut u32,
        range_config_timeout_us: &mut u32,
    ) -> VL53L1_Error {
        let mut macro_period_us = 0_u32;
        let mut timeout_encoded = 0_u16;
        let pdev = &mut self.Data.LLData;
        if pdev.stat_nvm.osc_measured__fast_osc__frequency == 0 {
            return VL53L1_ERROR_DIVISION_BY_ZERO;
        }
        /* Update Macro Period for Range A VCSEL Period */
        macro_period_us = VL53L1_calc_macro_period_us(
            pdev.stat_nvm.osc_measured__fast_osc__frequency,
            pdev.tim_cfg.range_config__vcsel_period_a,
        );
        /*  Get Phase Cal Timing A timeout */
        *phasecal_config_timeout_us = VL53L1_calc_timeout_us(
            pdev.gen_cfg.phasecal_config__timeout_macrop as u32,
            macro_period_us,
        );
        /*  Get MM Timing A timeout */
        timeout_encoded = pdev.tim_cfg.mm_config__timeout_macrop_a_hi as u16;
        timeout_encoded =
            (timeout_encoded << 8) | pdev.tim_cfg.mm_config__timeout_macrop_a_lo as u16;
        *mm_config_timeout_us = VL53L1_calc_decoded_timeout_us(timeout_encoded, macro_period_us);
        /*  Get Range Timing A timeout */
        timeout_encoded = pdev.tim_cfg.range_config__timeout_macrop_a_hi as u16;
        timeout_encoded =
            (timeout_encoded << 8) | pdev.tim_cfg.range_config__timeout_macrop_a_lo as u16;
        *range_config_timeout_us = VL53L1_calc_decoded_timeout_us(timeout_encoded, macro_period_us);
        pdev.phasecal_config_timeout_us = *phasecal_config_timeout_us;
        pdev.mm_config_timeout_us = *mm_config_timeout_us;
        pdev.range_config_timeout_us = *range_config_timeout_us;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_GetSequenceStepEnable(
        &mut self,
        SequenceStepId: VL53L1_SequenceStepId,
        pSequenceStepEnabled: &mut u8,
    ) -> VL53L1_Error {
        self.VL53L1_get_sequence_config_bit(SequenceStepId, pSequenceStepEnabled)
    }

    pub fn VL53L1_get_sequence_config_bit(
        &mut self,
        bit_id: VL53L1_DeviceSequenceConfig,
        pvalue: &mut u8,
    ) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        let mut bit_mask = 0x01_u8;
        if bit_id <= VL53L1_DEVICESEQUENCECONFIG_RANGE {
            if bit_id > 0 {
                bit_mask = 0x01 << bit_id;
            }
            *pvalue = pdev.dyn_cfg.system__sequence_config & bit_mask;

            if bit_id > 0 {
                *pvalue = *pvalue >> bit_id;
            }
            return VL53L1_ERROR_NONE;
        } else {
            return VL53L1_ERROR_INVALID_PARAMS;
        }
    }

    pub fn SetPresetMode(
        &mut self,
        PresetMode: VL53L1_PresetModes,
        DistanceMode: VL53L1_DistanceModes,
        inter_measurement_period_ms: u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut device_preset_mode: VL53L1_DevicePresetModes = 0;
        let mut measurement_mode = 0_u8;
        let mut dss_config__target_total_rate_mcps = 0_u16;
        let mut phasecal_config_timeout_us = 0_u32;
        let mut mm_config_timeout_us = 0_u32;
        let mut lld_range_config_timeout_us = 0_u32;

        if PresetMode == VL53L1_PRESETMODE_AUTONOMOUS
            || PresetMode == VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS
        {
            measurement_mode = VL53L1_DEVICEMEASUREMENTMODE_TIMED;
        } else {
            measurement_mode = VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK;
        }

        status = ComputeDevicePresetMode(PresetMode, DistanceMode, &mut device_preset_mode);
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_get_preset_mode_timing_cfg(
                device_preset_mode,
                &mut dss_config__target_total_rate_mcps,
                &mut phasecal_config_timeout_us,
                &mut mm_config_timeout_us,
                &mut lld_range_config_timeout_us,
            );
        }

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_set_preset_mode(
                device_preset_mode,
                dss_config__target_total_rate_mcps,
                phasecal_config_timeout_us,
                mm_config_timeout_us,
                lld_range_config_timeout_us,
                inter_measurement_period_ms,
            );
        }

        if status == VL53L1_ERROR_NONE {
            self.Data.LLData.measurement_mode = measurement_mode;
            self.Data.CurrentParameters.PresetMode = PresetMode;
        }
        return status;
    }

    pub fn VL53L1_get_preset_mode_timing_cfg(
        &self,
        device_preset_mode: VL53L1_DevicePresetModes,
        dss_config__target_total_rate_mcps: &mut u16,
        phasecal_config_timeout_us: &mut u32,
        mm_config_timeout_us: &mut u32,
        lld_range_config_timeout_us: &mut u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let pdev = &self.Data.LLData;
        match device_preset_mode {
            VL53L1_DEVICEPRESETMODE_STANDARD_RANGING
            | VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE
            | VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE
            | VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL
            | VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL
            | VL53L1_DEVICEPRESETMODE_OLT => {
                *dss_config__target_total_rate_mcps = pdev.tuning_parms.tp_dss_target_lite_mcps;
                *phasecal_config_timeout_us = pdev.tuning_parms.tp_phasecal_timeout_lite_us;
                *mm_config_timeout_us = pdev.tuning_parms.tp_mm_timeout_lite_us;
                *lld_range_config_timeout_us = pdev.tuning_parms.tp_range_timeout_lite_us;
            }
            VL53L1_DEVICEPRESETMODE_TIMED_RANGING
            | VL53L1_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE
            | VL53L1_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE
            | VL53L1_DEVICEPRESETMODE_SINGLESHOT_RANGING => {
                *dss_config__target_total_rate_mcps = pdev.tuning_parms.tp_dss_target_timed_mcps;
                *phasecal_config_timeout_us = pdev.tuning_parms.tp_phasecal_timeout_timed_us;
                *mm_config_timeout_us = pdev.tuning_parms.tp_mm_timeout_timed_us;
                *lld_range_config_timeout_us = pdev.tuning_parms.tp_range_timeout_timed_us;
            }
            VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE
            | VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE
            | VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE => {
                *dss_config__target_total_rate_mcps = pdev.tuning_parms.tp_dss_target_timed_mcps;
                *phasecal_config_timeout_us = pdev.tuning_parms.tp_phasecal_timeout_timed_us;
                *mm_config_timeout_us = pdev.tuning_parms.tp_mm_timeout_lpa_us;
                *lld_range_config_timeout_us = pdev.tuning_parms.tp_range_timeout_lpa_us;
            }
            _ => {
                status = VL53L1_ERROR_INVALID_PARAMS;
            }
        }
        return status;
    }

    pub fn VL53L1_DataInit(&mut self) -> VL53L1_Error {
        let mut status = self.VL53L1_data_init(1);
        if status == VL53L1_ERROR_NONE {
            self.Data.PalState = VL53L1_STATE_WAIT_STATICINIT;
        }

        for i in 0..VL53L1_CHECKENABLE_NUMBER_OF_CHECKS {
            if status == VL53L1_ERROR_NONE {
                status |= self.VL53L1_SetLimitCheckEnable(i as u16, 1);
            } else {
                break;
            }
        }
        return status;
    }

    pub fn VL53L1_SetLimitCheckEnable(
        &mut self,
        LimitCheckId: u16,
        LimitCheckEnable: u8,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        let mut TempFix1616 = 0_u32;
        if LimitCheckId >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS as u16 {
            status = VL53L1_ERROR_INVALID_PARAMS;
        } else {
            if LimitCheckEnable != 0 {
                TempFix1616 = self.Data.CurrentParameters.LimitChecksValue[LimitCheckId as usize];
            }
            status = self.SetLimitValue(LimitCheckId, TempFix1616);
        }

        if status == VL53L1_ERROR_NONE {
            self.Data.CurrentParameters.LimitChecksEnable[LimitCheckId as usize] =
                if LimitCheckEnable == 0 { 0 } else { 1 };
        }
        return status;
    }

    pub fn SetLimitValue(&mut self, LimitCheckId: u16, value: u32) -> VL53L1_Error {
        match LimitCheckId {
            VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE => {
                self.VL53L1_set_lite_sigma_threshold(VL53L1_FIXPOINT1616TOFIXPOINT142(value))
            }
            VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE => {
                self.VL53L1_set_lite_min_count_rate(VL53L1_FIXPOINT1616TOFIXPOINT97(value))
            }
            _ => VL53L1_ERROR_INVALID_PARAMS,
        }
    }

    pub fn VL53L1_set_lite_min_count_rate(&mut self, lite_mincountrate: u16) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        pdev.tim_cfg.range_config__min_count_rate_rtn_limit_mcps = lite_mincountrate;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_set_lite_sigma_threshold(&mut self, lite_sigma: u16) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        pdev.tim_cfg.range_config__sigma_thresh = lite_sigma;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_data_init(&mut self, read_p2p_data: u8) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;

        self.VL53L1_ll_driver_state(VL53L1_DEVICESTATE_UNKNOWN);
        {
            let pdev = &mut self.Data.LLData;
            pdev.wait_method = VL53L1_WAIT_METHOD_BLOCKING;
            pdev.preset_mode = VL53L1_DEVICEPRESETMODE_STANDARD_RANGING;
            pdev.measurement_mode = VL53L1_DEVICEMEASUREMENTMODE_STOP;

            pdev.offset_calibration_mode = VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD;
            pdev.offset_correction_mode = VL53L1_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;

            pdev.phasecal_config_timeout_us = 1000;
            pdev.mm_config_timeout_us = 2000;
            pdev.range_config_timeout_us = 13000;
            pdev.inter_measurement_period_ms = 100;
            pdev.dss_config__target_total_rate_mcps = 0x0A00;
            pdev.debug_mode = 0;
            /* initialise gain calibration values to tuning parameter values */
            pdev.gain_cal.standard_ranging_gain_factor =
                VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT;
        }

        self.VL53L1_init_version();

        if read_p2p_data > 0 {
            status = self.VL53L1_read_p2p_data();
        }

        /* Initialise Ref SPAD Char configuration structure */
        if !VL53L1_NOCALIB {
            status = self.VL53L1_init_refspadchar_config_struct();
            status = self.VL53L1_init_ssc_config_struct();
        }

        status = self.VL53L1_init_xtalk_config_struct();

        if !VL53L1_NOCALIB {
            status = self.VL53L1_init_offset_cal_config_struct();
        }

        status = self.VL53L1_init_tuning_parm_storage_struct();
        status = self.VL53L1_set_vhv_loopbound(VL53L1_TUNINGPARM_VHV_LOOPBOUND_DEFAULT);

        if status == VL53L1_ERROR_NONE {
            let pdev = &self.Data.LLData;
            self.VL53L1_set_preset_mode(
                pdev.preset_mode,
                pdev.dss_config__target_total_rate_mcps,
                pdev.phasecal_config_timeout_us,
                pdev.mm_config_timeout_us,
                pdev.range_config_timeout_us,
                pdev.inter_measurement_period_ms,
            );
        }
        self.VL53L1_low_power_auto_data_init();
        return status;
    }

    pub fn VL53L1_set_preset_mode(
        &mut self,
        device_preset_mode: VL53L1_DevicePresetModes,
        dss_config__target_total_rate_mcps: u16,
        phasecal_config_timeout_us: u32,
        mm_config_timeout_us: u32,
        range_config_timeout_us: u32,
        inter_measurement_period_ms: u32,
    ) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;
        self.VL53L1_init_ll_driver_state(VL53L1_DEVICESTATE_SW_STANDBY);
        {
            let pdev = &mut self.Data.LLData;
            pdev.preset_mode = device_preset_mode;
            pdev.mm_config_timeout_us = mm_config_timeout_us;
            pdev.range_config_timeout_us = range_config_timeout_us;
            pdev.inter_measurement_period_ms = inter_measurement_period_ms;

            let mut pstatic = &mut pdev.stat_cfg;
            let pgeneral = &mut pdev.gen_cfg;
            let ptiming = &mut pdev.tim_cfg;
            let pdynamic = &mut pdev.dyn_cfg;
            let psystem = &mut pdev.sys_ctrl;
            let ptuning_parms = &mut pdev.tuning_parms;
            let plpadata = &mut pdev.low_power_auto_data;

            status = match pdev.preset_mode {
                VL53L1_DEVICEPRESETMODE_STANDARD_RANGING => VL53L1_preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms,
                ),
                VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE => {
                    VL53L1_preset_mode_standard_ranging_short_range(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                    )
                }
                VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE => {
                    VL53L1_preset_mode_standard_ranging_long_range(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                    )
                }
                VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL => {
                    if !VL53L1_NOCALIB {
                        VL53L1_preset_mode_standard_ranging_mm1_cal(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms,
                        )
                    } else {
                        VL53L1_ERROR_INVALID_PARAMS
                    }
                }
                VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL => {
                    if !VL53L1_NOCALIB {
                        VL53L1_preset_mode_standard_ranging_mm2_cal(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms,
                        )
                    } else {
                        VL53L1_ERROR_INVALID_PARAMS
                    }
                }
                VL53L1_DEVICEPRESETMODE_TIMED_RANGING => VL53L1_preset_mode_timed_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms,
                ),
                VL53L1_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE => {
                    VL53L1_preset_mode_timed_ranging_short_range(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                    )
                }
                VL53L1_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE => {
                    VL53L1_preset_mode_timed_ranging_long_range(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                    )
                }
                VL53L1_DEVICEPRESETMODE_OLT => VL53L1_preset_mode_olt(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms,
                ),
                VL53L1_DEVICEPRESETMODE_SINGLESHOT_RANGING => {
                    VL53L1_preset_mode_singleshot_ranging(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                    )
                }
                VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE => {
                    VL53L1_preset_mode_low_power_auto_short_ranging(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                        plpadata,
                    )
                }
                VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE => {
                    VL53L1_preset_mode_low_power_auto_ranging(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                        plpadata,
                    )
                }
                VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE => {
                    VL53L1_preset_mode_low_power_auto_long_ranging(
                        pstatic,
                        pgeneral,
                        ptiming,
                        pdynamic,
                        psystem,
                        ptuning_parms,
                        plpadata,
                    )
                }
                _ => VL53L1_ERROR_INVALID_PARAMS,
            };
            /* update DSS target */
            if status == VL53L1_ERROR_NONE {
                pstatic.dss_config__target_total_rate_mcps = dss_config__target_total_rate_mcps;
                pdev.dss_config__target_total_rate_mcps = dss_config__target_total_rate_mcps;
            }
        }

        /*
         * Update the register timeout values based on input
         * real time values and preset mode VCSEL periods
         */
        if status == VL53L1_ERROR_NONE {
            let pdev = &self.Data.LLData;
            status = self.VL53L1_set_timeouts_us(
                phasecal_config_timeout_us,
                mm_config_timeout_us,
                range_config_timeout_us,
            );
        }

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_set_inter_measurement_period_ms(inter_measurement_period_ms);
        }
        return status;
    }

    pub fn VL53L1_set_inter_measurement_period_ms(
        &mut self,
        inter_measurement_period_ms: u32,
    ) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        if pdev.dbg_results.result__osc_calibrate_val == 0 {
            return VL53L1_ERROR_DIVISION_BY_ZERO;
        } else {
            pdev.inter_measurement_period_ms = inter_measurement_period_ms;
            pdev.tim_cfg.system__intermeasurement_period =
                inter_measurement_period_ms * pdev.dbg_results.result__osc_calibrate_val as u32;
            return VL53L1_ERROR_NONE;
        }
    }

    pub fn VL53L1_set_timeouts_us(
        &mut self,
        phasecal_config_timeout_us: u32,
        mm_config_timeout_us: u32,
        range_config_timeout_us: u32,
    ) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        if pdev.stat_nvm.osc_measured__fast_osc__frequency == 0 {
            return VL53L1_ERROR_DIVISION_BY_ZERO;
        }

        pdev.phasecal_config_timeout_us = phasecal_config_timeout_us;
        pdev.mm_config_timeout_us = mm_config_timeout_us;
        pdev.range_config_timeout_us = range_config_timeout_us;

        // return VL53L1_calc_timeout_register_values(
        // 	phasecal_config_timeout_us,
        // 	pdev.stat_nvm.osc_measured__fast_osc__frequency,
        // 	&pdev.phasecal_config_timeout_mclks);
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_init_ll_driver_state(&mut self, device_state: VL53L1_DeviceState) {
        let pstate = &mut self.Data.LLData.ll_state;
        pstate.cfg_device_state = device_state;
        pstate.cfg_stream_count = 0;
        pstate.cfg_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
        pstate.cfg_timing_status = 0;
        pstate.rd_device_state = device_state;
        pstate.rd_stream_count = 0;
        pstate.rd_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
        pstate.rd_timing_status = 0;
    }

    pub fn VL53L1_init_refspadchar_config_struct(&mut self) -> VL53L1_Error {
        let mut pdata = &mut self.Data.LLData.refspadchar;
        /* Reference SPAD Char Configuration
         *
         * vcsel_period              = 0x0B   - 24 clock VCSEL period
         * timeout_us                = 1000   - Set 1000us phase cal timeout
         * target_count_rate_mcps    = 0x0A00 - 9.7 -> 20.0 Mcps
         * min_count_rate_limit_mcps = 0x0500 - 9.7 -> 10.0 Mcps
         * max_count_rate_limit_mcps = 0x1400 - 9.7 -> 40.0 Mcps
         */
        pdata.device_test_mode = VL53L1_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT;
        pdata.vcsel_period = VL53L1_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT;
        pdata.timeout_us = VL53L1_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT;
        pdata.target_count_rate_mcps = VL53L1_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT;
        pdata.min_count_rate_limit_mcps =
            VL53L1_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT;
        pdata.max_count_rate_limit_mcps =
            VL53L1_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_init_ssc_config_struct(&mut self) -> VL53L1_Error {
        let mut pdata = &mut self.Data.LLData.ssc_cfg;
        /* SPAD Select Check Configuration */

        /* 0 - store RTN count rates
         * 1 - store REF count rates
         */
        pdata.array_select = VL53L1_DEVICESSCARRAY_RTN;

        /* VCSEL period register value  0x12 (18) -> 38 VCSEL clocks */
        pdata.vcsel_period = VL53L1_TUNINGPARM_SPADMAP_VCSEL_PERIOD_DEFAULT;

        /* VCSEL pulse start */
        pdata.vcsel_start = VL53L1_TUNINGPARM_SPADMAP_VCSEL_START_DEFAULT;

        /* VCSEL pulse width */
        pdata.vcsel_width = 0x02;

        /* SSC timeout [us] */
        pdata.timeout_us = 36000;

        /* SSC rate limit [Mcps]
         * - 9.7 for VCSEL ON
         * - 1.15 for VCSEL OFF
         */
        pdata.rate_limit_mcps = VL53L1_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_init_xtalk_config_struct(&mut self) -> VL53L1_Error {
        let mut pnvm = &mut self.Data.LLData.customer;
        let mut pdata = &mut self.Data.LLData.xtalk_cfg;
        /* Store xtalk data into golden copy */
        pdata.algo__crosstalk_compensation_plane_offset_kcps =
            pnvm.algo__crosstalk_compensation_plane_offset_kcps as u32;
        pdata.algo__crosstalk_compensation_x_plane_gradient_kcps =
            pnvm.algo__crosstalk_compensation_x_plane_gradient_kcps;
        pdata.algo__crosstalk_compensation_y_plane_gradient_kcps =
            pnvm.algo__crosstalk_compensation_y_plane_gradient_kcps;

        /* Store NVM defaults for later use */
        pdata.nvm_default__crosstalk_compensation_plane_offset_kcps =
            pnvm.algo__crosstalk_compensation_plane_offset_kcps as u32;
        pdata.nvm_default__crosstalk_compensation_x_plane_gradient_kcps =
            pnvm.algo__crosstalk_compensation_x_plane_gradient_kcps;
        pdata.nvm_default__crosstalk_compensation_y_plane_gradient_kcps =
            pnvm.algo__crosstalk_compensation_y_plane_gradient_kcps;

        pdata.lite_mode_crosstalk_margin_kcps = VL53L1_TUNINGPARM_LITE_XTALK_MARGIN_KCPS_DEFAULT;

        /* Default for Range Ignore Threshold Mult = 2.0 */
        pdata.crosstalk_range_ignore_threshold_mult = VL53L1_TUNINGPARM_LITE_RIT_MULT_DEFAULT;

        if (pdata.algo__crosstalk_compensation_plane_offset_kcps == 0x00)
            && (pdata.algo__crosstalk_compensation_x_plane_gradient_kcps == 0x00)
            && (pdata.algo__crosstalk_compensation_y_plane_gradient_kcps == 0x00)
        {
            pdata.global_crosstalk_compensation_enable = 0x00;
        } else {
            pdata.global_crosstalk_compensation_enable = 0x01;
        }

        if pdata.global_crosstalk_compensation_enable == 0x01 {
            pdata.crosstalk_range_ignore_threshold_rate_mcps = VL53L1_calc_range_ignore_threshold(
                pdata.algo__crosstalk_compensation_plane_offset_kcps,
                pdata.algo__crosstalk_compensation_x_plane_gradient_kcps,
                pdata.algo__crosstalk_compensation_y_plane_gradient_kcps,
                pdata.crosstalk_range_ignore_threshold_mult,
            );
        } else {
            pdata.crosstalk_range_ignore_threshold_rate_mcps = 0;
        }
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_init_offset_cal_config_struct(&mut self) -> VL53L1_Error {
        let mut pdata = &mut self.Data.LLData.offsetcal_cfg;
        /* Preset Timeout and DSS defaults */
        pdata.dss_config__target_total_rate_mcps =
            VL53L1_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT;
        /* 20.0 Mcps */
        pdata.phasecal_config_timeout_us = VL53L1_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT;
        /* 1000 us */
        pdata.range_config_timeout_us = VL53L1_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT;
        /* 13000 us */
        pdata.mm_config_timeout_us = VL53L1_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT;
        /* 13000 us - Added as part of Patch_AddedOffsetCalMMTuningParm_11791 */

        /* Init number of averaged samples */
        pdata.pre_num_of_samples = VL53L1_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES_DEFAULT;
        pdata.mm1_num_of_samples = VL53L1_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES_DEFAULT;
        pdata.mm2_num_of_samples = VL53L1_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES_DEFAULT;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_init_tuning_parm_storage_struct(&mut self) -> VL53L1_Error {
        let mut pdata = &mut self.Data.LLData.tuning_parms;
        pdata.tp_tuning_parm_version = VL53L1_TUNINGPARM_VERSION_DEFAULT;
        pdata.tp_tuning_parm_key_table_version = VL53L1_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT;
        pdata.tp_tuning_parm_lld_version = VL53L1_TUNINGPARM_LLD_VERSION_DEFAULT;
        pdata.tp_init_phase_rtn_lite_long =
            VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT;
        pdata.tp_init_phase_rtn_lite_med =
            VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT;
        pdata.tp_init_phase_rtn_lite_short =
            VL53L1_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT;
        pdata.tp_init_phase_ref_lite_long =
            VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT;
        pdata.tp_init_phase_ref_lite_med =
            VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT;
        pdata.tp_init_phase_ref_lite_short =
            VL53L1_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT;
        pdata.tp_consistency_lite_phase_tolerance =
            VL53L1_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT;
        pdata.tp_phasecal_target = VL53L1_TUNINGPARM_PHASECAL_TARGET_DEFAULT;
        pdata.tp_cal_repeat_rate = VL53L1_TUNINGPARM_LITE_CAL_REPEAT_RATE_DEFAULT;
        pdata.tp_lite_min_clip = VL53L1_TUNINGPARM_LITE_MIN_CLIP_MM_DEFAULT;
        pdata.tp_lite_long_sigma_thresh_mm = VL53L1_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT;
        pdata.tp_lite_med_sigma_thresh_mm = VL53L1_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM_DEFAULT;
        pdata.tp_lite_short_sigma_thresh_mm = VL53L1_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT;
        pdata.tp_lite_long_min_count_rate_rtn_mcps =
            VL53L1_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
        pdata.tp_lite_med_min_count_rate_rtn_mcps =
            VL53L1_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
        pdata.tp_lite_short_min_count_rate_rtn_mcps =
            VL53L1_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
        pdata.tp_lite_sigma_est_pulse_width_ns =
            VL53L1_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT;
        pdata.tp_lite_sigma_est_amb_width_ns =
            VL53L1_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT;
        pdata.tp_lite_sigma_ref_mm = VL53L1_TUNINGPARM_LITE_SIGMA_REF_MM_DEFAULT;
        pdata.tp_lite_seed_cfg = VL53L1_TUNINGPARM_LITE_SEED_CONFIG_DEFAULT;
        pdata.tp_timed_seed_cfg = VL53L1_TUNINGPARM_TIMED_SEED_CONFIG_DEFAULT;
        pdata.tp_lite_quantifier = VL53L1_TUNINGPARM_LITE_QUANTIFIER_DEFAULT;
        pdata.tp_lite_first_order_select = VL53L1_TUNINGPARM_LITE_FIRST_ORDER_SELECT_DEFAULT;

        /* Preset Mode Configurations */
        /* - New parms added as part of Patch_TuningParmPresetModeAddition_11839 */

        pdata.tp_dss_target_lite_mcps =
            VL53L1_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
        pdata.tp_dss_target_timed_mcps =
            VL53L1_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
        pdata.tp_phasecal_timeout_lite_us =
            VL53L1_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US as u32;
        pdata.tp_phasecal_timeout_timed_us =
            VL53L1_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
        pdata.tp_mm_timeout_lite_us = VL53L1_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT;
        pdata.tp_mm_timeout_timed_us = VL53L1_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT;
        pdata.tp_range_timeout_lite_us = VL53L1_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
        pdata.tp_range_timeout_timed_us = VL53L1_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

        /* Added for Patch_LowPowerAutoMode */

        pdata.tp_mm_timeout_lpa_us = VL53L1_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT;
        pdata.tp_range_timeout_lpa_us =
            VL53L1_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_set_vhv_loopbound(&mut self, vhv_loop_bound: u8) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound =
            (pdev.stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03) + vhv_loop_bound * 4;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_low_power_auto_data_init(&mut self) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        pdev.low_power_auto_data.vhv_loop_bound =
            VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT;
        pdev.low_power_auto_data.is_low_power_auto_mode = 0;
        pdev.low_power_auto_data.low_power_auto_range_count = 0;
        pdev.low_power_auto_data.saved_interrupt_config = 0;
        pdev.low_power_auto_data.saved_vhv_init = 0;
        pdev.low_power_auto_data.saved_vhv_timeout = 0;
        pdev.low_power_auto_data.first_run_phasecal_result = 0;
        pdev.low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
        pdev.low_power_auto_data.dss__required_spads = 0;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_ll_driver_state(&mut self, device_state: VL53L1_DeviceState) {
        let pstate = &mut self.Data.LLData.ll_state;
        pstate.cfg_device_state = device_state;
        pstate.cfg_stream_count = 0;
        pstate.cfg_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
        pstate.cfg_timing_status = 0;

        pstate.rd_device_state = device_state;
        pstate.rd_stream_count = 0;
        pstate.rd_gph_id = VL53L1_GROUPEDPARAMETERHOLD_ID_MASK;
        pstate.rd_timing_status = 0;
    }

    pub fn VL53L1_init_version(&mut self) {
        let pver = &mut self.Data.LLData.version;
        pver.ll_major = VL53L1_LL_API_IMPLEMENTATION_VER_MAJOR;
        pver.ll_minor = VL53L1_LL_API_IMPLEMENTATION_VER_MINOR;
        pver.ll_build = VL53L1_LL_API_IMPLEMENTATION_VER_SUB;
        pver.ll_revision = VL53L1_LL_API_IMPLEMENTATION_VER_REVISION;
    }

    pub fn VL53L1_read_p2p_data(&mut self) -> VL53L1_Error {
        let mut status = VL53L1_ERROR_NONE;

        status = self.VL53L1_get_static_nvm_managed();

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_get_customer_nvm_managed();
        }

        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_get_nvm_copy_data();

            if status == VL53L1_ERROR_NONE {
                let pdev = &mut self.Data.LLData;
                /* copy Return Good SPADs to buffer */
                VL53L1_copy_rtn_good_spads_to_buffer(&pdev.nvm_copy_data, &mut pdev.rtn_good_spads);
            }
        }
        /*
         * read slow osc calibration value
         * counts per ms
         */
        if status == VL53L1_ERROR_NONE {
            let mut rslt: u16 = 0;
            status = self.VL53L1_RdWord(VL53L1_RESULT__OSC_CALIBRATE_VAL, &mut rslt);
            self.Data.LLData.dbg_results.result__osc_calibrate_val = rslt;
        }
        /*
         * Check if there a sensible value for osc_measured__fast_osc__frequency
         */
        {
            let pdev = &mut self.Data.LLData;
            if pdev.stat_nvm.osc_measured__fast_osc__frequency < 0x1000 {
                pdev.stat_nvm.osc_measured__fast_osc__frequency = 0xBCCC;
            }
        }

        if status == VL53L1_ERROR_NONE {
            let mut roi: VL53L1_user_zone_t = VL53L1_user_zone_t::default();
            status = self.VL53L1_get_mode_mitigation_roi(&mut roi);
            self.Data.LLData.mm_roi = roi;
        }

        {
            let pdev = &mut self.Data.LLData;
            if pdev.optical_centre.x_centre == 0 && pdev.optical_centre.y_centre == 0 {
                pdev.optical_centre.x_centre = pdev.mm_roi.x_centre << 4;
                pdev.optical_centre.y_centre = pdev.mm_roi.y_centre << 4;
            }
        }

        return status;
    }

    pub fn VL53L1_get_mode_mitigation_roi(
        &mut self,
        pmm_roi: &mut VL53L1_user_zone_t,
    ) -> VL53L1_Error {
        let pdev = &mut self.Data.LLData;
        let mut x: u8 = 0;
        let mut y: u8 = 0;
        let mut xy_size: u8;

        VL53L1_decode_row_col(
            pdev.nvm_copy_data.roi_config__mode_roi_centre_spad,
            &mut y,
            &mut x,
        );
        pmm_roi.x_centre = x;
        pmm_roi.y_centre = y;
        xy_size = pdev.nvm_copy_data.roi_config__mode_roi_xy_size;
        pmm_roi.height = xy_size >> 4;
        pmm_roi.width = xy_size & 0x0F;
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_nvm_copy_data(&mut self) -> VL53L1_Error {
        let mut comms_buffer = [0_u8; VL53L1_NVM_COPY_DATA_I2C_SIZE_BYTES];
        let mut status = self.VL53L1_ReadMulti(VL53L1_IDENTIFICATION__MODEL_ID, &mut comms_buffer);
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_i2c_decode_nvm_copy_data(&comms_buffer);
        }
        return status;
    }

    pub fn VL53L1_i2c_decode_nvm_copy_data(&mut self, pbuffer: &[u8]) -> VL53L1_Error {
        if VL53L1_NVM_COPY_DATA_I2C_SIZE_BYTES > pbuffer.len() {
            return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
        }
        let mut pdata = &mut self.Data.LLData.nvm_copy_data;
        pdata.identification__model_id = pbuffer[0];
        pdata.identification__module_type = pbuffer[1];
        pdata.identification__revision_id = pbuffer[2];
        pdata.identification__module_id = VL53L1_i2c_decode_uint16_t(&pbuffer[3..5]);
        pdata.ana_config__fast_osc__trim_max = pbuffer[5] & 0x7F;
        pdata.ana_config__fast_osc__freq_set = pbuffer[6] & 0x7;
        pdata.ana_config__vcsel_trim = pbuffer[7] & 0x7;
        pdata.ana_config__vcsel_selion = pbuffer[8] & 0x3F;
        pdata.ana_config__vcsel_selion_max = pbuffer[9] & 0x3F;
        pdata.protected_laser_safety__lock_bit = pbuffer[10] & 0x1;
        pdata.laser_safety__key = pbuffer[11] & 0x7F;
        pdata.laser_safety__key_ro = pbuffer[12] & 0x1;
        pdata.laser_safety__clip = pbuffer[13] & 0x3F;
        pdata.laser_safety__mult = pbuffer[14] & 0x3F;
        pdata.global_config__spad_enables_rtn_0 = pbuffer[15];
        pdata.global_config__spad_enables_rtn_1 = pbuffer[16];
        pdata.global_config__spad_enables_rtn_2 = pbuffer[17];
        pdata.global_config__spad_enables_rtn_3 = pbuffer[18];
        pdata.global_config__spad_enables_rtn_4 = pbuffer[19];
        pdata.global_config__spad_enables_rtn_5 = pbuffer[20];
        pdata.global_config__spad_enables_rtn_6 = pbuffer[21];
        pdata.global_config__spad_enables_rtn_7 = pbuffer[22];
        pdata.global_config__spad_enables_rtn_8 = pbuffer[23];
        pdata.global_config__spad_enables_rtn_9 = pbuffer[24];
        pdata.global_config__spad_enables_rtn_10 = pbuffer[25];
        pdata.global_config__spad_enables_rtn_11 = pbuffer[26];
        pdata.global_config__spad_enables_rtn_12 = pbuffer[27];
        pdata.global_config__spad_enables_rtn_13 = pbuffer[28];
        pdata.global_config__spad_enables_rtn_14 = pbuffer[29];
        pdata.global_config__spad_enables_rtn_15 = pbuffer[30];
        pdata.global_config__spad_enables_rtn_16 = pbuffer[31];
        pdata.global_config__spad_enables_rtn_17 = pbuffer[32];
        pdata.global_config__spad_enables_rtn_18 = pbuffer[33];
        pdata.global_config__spad_enables_rtn_19 = pbuffer[34];
        pdata.global_config__spad_enables_rtn_20 = pbuffer[35];
        pdata.global_config__spad_enables_rtn_21 = pbuffer[36];
        pdata.global_config__spad_enables_rtn_22 = pbuffer[37];
        pdata.global_config__spad_enables_rtn_23 = pbuffer[38];
        pdata.global_config__spad_enables_rtn_24 = pbuffer[39];
        pdata.global_config__spad_enables_rtn_25 = pbuffer[40];
        pdata.global_config__spad_enables_rtn_26 = pbuffer[41];
        pdata.global_config__spad_enables_rtn_27 = pbuffer[42];
        pdata.global_config__spad_enables_rtn_28 = pbuffer[43];
        pdata.global_config__spad_enables_rtn_29 = pbuffer[44];
        pdata.global_config__spad_enables_rtn_30 = pbuffer[45];
        pdata.global_config__spad_enables_rtn_31 = pbuffer[46];
        pdata.roi_config__mode_roi_centre_spad = pbuffer[47];
        pdata.roi_config__mode_roi_xy_size = pbuffer[48];
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_customer_nvm_managed(&mut self) -> VL53L1_Error {
        let mut comms_buffer = [0_u8; VL53L1_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES];
        let mut status =
            self.VL53L1_ReadMulti(VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_0, &mut comms_buffer);
        if status == VL53L1_ERROR_NONE {
            status = self.VL53L1_i2c_decode_customer_nvm_managed(&comms_buffer);
        }
        return status;
    }

    pub fn VL53L1_i2c_decode_customer_nvm_managed(&mut self, pbuffer: &[u8]) -> VL53L1_Error {
        if VL53L1_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES > pbuffer.len() {
            return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
        }
        let mut pdata = &mut self.Data.LLData.customer;
        pdata.global_config__spad_enables_ref_0 = pbuffer[0];
        pdata.global_config__spad_enables_ref_1 = pbuffer[1];
        pdata.global_config__spad_enables_ref_2 = pbuffer[2];
        pdata.global_config__spad_enables_ref_3 = pbuffer[3];
        pdata.global_config__spad_enables_ref_4 = pbuffer[4];
        pdata.global_config__spad_enables_ref_5 = pbuffer[5] & 0xF;
        pdata.global_config__ref_en_start_select = pbuffer[6];
        pdata.ref_spad_man__num_requested_ref_spads = pbuffer[7] & 0x3F;
        pdata.ref_spad_man__ref_location = pbuffer[8] & 0x3;
        pdata.algo__crosstalk_compensation_plane_offset_kcps =
            VL53L1_i2c_decode_uint16_t(&pbuffer[9..11]);
        pdata.algo__crosstalk_compensation_x_plane_gradient_kcps =
            VL53L1_i2c_decode_int16_t(&pbuffer[11..13]);
        pdata.algo__crosstalk_compensation_y_plane_gradient_kcps =
            VL53L1_i2c_decode_int16_t(&pbuffer[13..15]);
        pdata.ref_spad_char__total_rate_target_mcps = VL53L1_i2c_decode_uint16_t(&pbuffer[15..17]);
        pdata.algo__part_to_part_range_offset_mm =
            VL53L1_i2c_decode_int16_t(&pbuffer[17..19]) & 0x1FFF;
        pdata.mm_config__inner_offset_mm = VL53L1_i2c_decode_int16_t(&pbuffer[19..21]);
        pdata.mm_config__outer_offset_mm = VL53L1_i2c_decode_int16_t(&pbuffer[21..23]);
        return VL53L1_ERROR_NONE;
    }

    pub fn VL53L1_get_static_nvm_managed(&mut self) -> VL53L1_Error {
        let mut comms_buffer = [0_u8; VL53L1_STATIC_NVM_MANAGED_I2C_SIZE_BYTES];
        let status = self.VL53L1_ReadMulti(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, &mut comms_buffer);
        if status == VL53L1_ERROR_NONE {
            return self.VL53L1_i2c_decode_static_nvm_managed(&comms_buffer);
        }
        return status;
    }

    pub fn VL53L1_ReadMulti(&self, index: u16, pdata: &mut [u8]) -> VL53L1_Error {
        crate::module::tof::i2c1_read(index, pdata, self.i2c_slave_address as u32)
    }

    pub fn VL53L1_RdWord(&mut self, index: u16, pdata: &mut u16) -> VL53L1_Error {
        let mut comms_buffer = [0_u8; 2];
        let status = self.VL53L1_ReadMulti(index, &mut comms_buffer);
        if status == VL53L1_ERROR_NONE {
            *pdata = VL53L1_i2c_decode_uint16_t(&comms_buffer);
        }
        return status;
    }

    pub fn VL53L1_i2c_decode_static_nvm_managed(&mut self, pbuffer: &[u8]) -> VL53L1_Error {
        if VL53L1_STATIC_NVM_MANAGED_I2C_SIZE_BYTES > pbuffer.len() {
            return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
        }
        let mut pdata = &mut self.Data.LLData.stat_nvm;
        pdata.i2c_slave__device_address = pbuffer[0] & 0x7F;
        pdata.ana_config__vhv_ref_sel_vddpix = pbuffer[1] & 0xF;
        pdata.ana_config__vhv_ref_sel_vquench = pbuffer[2] & 0x7F;
        pdata.ana_config__reg_avdd1v2_sel = pbuffer[3] & 0x3;
        pdata.ana_config__fast_osc__trim = pbuffer[4] & 0x7F;
        pdata.osc_measured__fast_osc__frequency = VL53L1_i2c_decode_uint16_t(&pbuffer[5..7]);
        pdata.vhv_config__timeout_macrop_loop_bound = pbuffer[7];
        pdata.vhv_config__count_thresh = pbuffer[8];
        pdata.vhv_config__offset = pbuffer[9] & 0x3F;
        pdata.vhv_config__init = pbuffer[10];
        return VL53L1_ERROR_NONE;
    }
}
