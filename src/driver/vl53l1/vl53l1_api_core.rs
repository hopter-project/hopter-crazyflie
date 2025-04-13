#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::{
    vl53l1_error_codes::*, vl53l1_ll_def::*, vl53l1_ll_device::*, vl53l1_register_settings::*,
    vl53l1_register_structs::*,
};

pub fn VL53L1_copy_sys_and_core_results_to_range_results(
    gain_factor: i32,
    psys: &mut VL53L1_system_results_t,
    pcore: &mut VL53L1_core_results_t,
    presults: &mut VL53L1_range_results_t,
) -> VL53L1_Error {
    let status = VL53L1_ERROR_NONE;
    let mut tmpu32 = 0_u32;
    let mut range_mm = 0_i32;
    presults.stream_count = psys.result__stream_count;

    for i in 0..2 {
        let mut pdata = &mut presults.data[i];
        pdata.range_id = i as u8;
        pdata.time_stamp = 0;
        if psys.result__stream_count == 0
            && psys.result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK
                == VL53L1_DEVICEERROR_RANGECOMPLETE
        {
            pdata.range_status = VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK;
        } else {
            pdata.range_status = psys.result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK;
        }

        if i == 0 {
            if psys.result__report_status == VL53L1_DEVICEREPORTSTATUS_MM1 {
                pdata.actual_effective_spads = psys.result__mm_inner_actual_effective_spads_sd0;
            } else if psys.result__report_status == VL53L1_DEVICEREPORTSTATUS_MM2 {
                pdata.actual_effective_spads = psys.result__mm_outer_actual_effective_spads_sd0;
            } else {
                pdata.actual_effective_spads = psys.result__dss_actual_effective_spads_sd0;
            }

            pdata.peak_signal_count_rate_mcps =
                psys.result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
            pdata.avg_signal_count_rate_mcps = psys.result__avg_signal_count_rate_mcps_sd0;
            pdata.ambient_count_rate_mcps = psys.result__ambient_count_rate_mcps_sd0;
            /* Start Patch_SigmaEstimateAccuracyImprovement */

            /* shift up sigma estimate to 7 bit fractional and clip to 9 bit int */
            tmpu32 = (psys.result__sigma_sd0 as u32) << 5;
            if tmpu32 > 0xFFFF {
                tmpu32 = 0xFFFF;
            }
            pdata.sigma_mm = tmpu32 as u16;
            /* End Patch_SigmaEstimateAccuracyImprovement */
            pdata.median_phase = psys.result__phase_sd0;

            range_mm = psys.result__final_crosstalk_corrected_range_mm_sd0 as i32;

            /* apply correction gain */
            range_mm *= gain_factor;
            range_mm += 0x0400;
            range_mm /= 0x0800;

            pdata.median_range_mm = range_mm as i16;

            pdata.ranging_total_events = pcore.result_core__ranging_total_events_sd0;
            pdata.signal_total_events = pcore.result_core__signal_total_events_sd0;
            pdata.total_periods_elapsed = pcore.result_core__total_periods_elapsed_sd0;
            pdata.ambient_window_events = pcore.result_core__ambient_window_events_sd0;
        } else {
            pdata.actual_effective_spads = psys.result__dss_actual_effective_spads_sd1;
            pdata.peak_signal_count_rate_mcps = psys.result__peak_signal_count_rate_mcps_sd1;
            pdata.avg_signal_count_rate_mcps = 0xFFFF;
            pdata.ambient_count_rate_mcps = psys.result__ambient_count_rate_mcps_sd1;

            /* Start Patch_SigmaEstimateAccuracyImprovement */

            /* shift up sigma estimate to 7 bit fractional and clip to 9 bit int */
            tmpu32 = (psys.result__sigma_sd1 as u32) << 5;
            if tmpu32 > 0xFFFF {
                tmpu32 = 0xFFFF;
            }
            pdata.sigma_mm = tmpu32 as u16;

            /* End Patch_SigmaEstimateAccuracyImprovement */
            pdata.median_phase = psys.result__phase_sd1;
            range_mm = psys.result__final_crosstalk_corrected_range_mm_sd1 as i32;

            /* apply correction gain */
            range_mm *= gain_factor;
            range_mm += 0x0400;
            range_mm /= 0x0800;

            pdata.median_range_mm = range_mm as i16;

            pdata.ranging_total_events = pcore.result_core__ranging_total_events_sd1;
            pdata.signal_total_events = pcore.result_core__signal_total_events_sd1;
            pdata.total_periods_elapsed = pcore.result_core__total_periods_elapsed_sd1;
            pdata.ambient_window_events = pcore.result_core__ambient_window_events_sd1;
        }
    }

    /* Update Global Device Status for results
     * - Default to no update
     */
    presults.device_status = VL53L1_DEVICEERROR_NOUPDATE;
    /* Check range status
     * - If device error condition, update device status
     * - Remove device status from range status output this should
     * only contain information relating to range data
     */
    match psys.result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK {
        VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE
        | VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE
        | VL53L1_DEVICEERROR_NOVHVVALUEFOUND
        | VL53L1_DEVICEERROR_USERROICLIP
        | VL53L1_DEVICEERROR_MULTCLIPFAIL => {
            presults.device_status =
                psys.result__range_status & VL53L1_RANGE_STATUS__RANGE_STATUS_MASK;
            presults.data[0].range_status = VL53L1_DEVICEERROR_NOUPDATE;
        }
        _ => {}
    }
    return status;
}
