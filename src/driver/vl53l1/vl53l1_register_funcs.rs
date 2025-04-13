#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::{vl53l1_core::*, vl53l1_error_codes::*, vl53l1_register_structs::*};

pub fn VL53L1_i2c_encode_system_control(
    pdata: &VL53L1_system_control_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_SYSTEM_CONTROL_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }

    pbuffer[0] = pdata.power_management__go1_power_force & 0x1;
    pbuffer[1] = pdata.system__stream_count_ctrl & 0x1;
    pbuffer[2] = pdata.firmware__enable & 0x1;
    pbuffer[3] = pdata.system__interrupt_clear & 0x3;
    pbuffer[4] = pdata.system__mode_start;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_encode_static_nvm_managed(
    pdata: &mut VL53L1_static_nvm_managed_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_STATIC_NVM_MANAGED_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }

    pbuffer[0] = pdata.i2c_slave__device_address & 0x7F;
    pbuffer[1] = pdata.ana_config__vhv_ref_sel_vddpix & 0xF;
    pbuffer[2] = pdata.ana_config__vhv_ref_sel_vquench & 0x7F;
    pbuffer[3] = pdata.ana_config__reg_avdd1v2_sel & 0x3;
    pbuffer[4] = pdata.ana_config__fast_osc__trim & 0x7F;
    VL53L1_i2c_encode_uint16_t(pdata.osc_measured__fast_osc__frequency, &mut pbuffer[5..7]);
    pbuffer[7] = pdata.vhv_config__timeout_macrop_loop_bound;
    pbuffer[8] = pdata.vhv_config__count_thresh;
    pbuffer[9] = pdata.vhv_config__offset & 0x3F;
    pbuffer[10] = pdata.vhv_config__init;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_encode_customer_nvm_managed(
    pdata: &mut VL53L1_customer_nvm_managed_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }
    pbuffer[0] = pdata.global_config__spad_enables_ref_0;
    pbuffer[1] = pdata.global_config__spad_enables_ref_1;
    pbuffer[2] = pdata.global_config__spad_enables_ref_2;
    pbuffer[3] = pdata.global_config__spad_enables_ref_3;
    pbuffer[4] = pdata.global_config__spad_enables_ref_4;
    pbuffer[5] = pdata.global_config__spad_enables_ref_5 & 0xF;
    pbuffer[6] = pdata.global_config__ref_en_start_select;
    pbuffer[7] = pdata.ref_spad_man__num_requested_ref_spads & 0x3F;
    pbuffer[8] = pdata.ref_spad_man__ref_location & 0x3;
    VL53L1_i2c_encode_uint16_t(
        pdata.algo__crosstalk_compensation_plane_offset_kcps,
        &mut pbuffer[9..11],
    );
    VL53L1_i2c_encode_int16_t(
        pdata.algo__crosstalk_compensation_x_plane_gradient_kcps,
        &mut pbuffer[11..13],
    );
    VL53L1_i2c_encode_int16_t(
        pdata.algo__crosstalk_compensation_y_plane_gradient_kcps,
        &mut pbuffer[13..15],
    );
    VL53L1_i2c_encode_uint16_t(
        pdata.ref_spad_char__total_rate_target_mcps,
        &mut pbuffer[15..17],
    );
    VL53L1_i2c_encode_int16_t(
        pdata.algo__part_to_part_range_offset_mm & 0x1FFF,
        &mut pbuffer[17..19],
    );
    VL53L1_i2c_encode_int16_t(pdata.mm_config__inner_offset_mm, &mut pbuffer[19..21]);
    VL53L1_i2c_encode_int16_t(pdata.mm_config__outer_offset_mm, &mut pbuffer[21..23]);
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_encode_static_config(
    pdata: &mut VL53L1_static_config_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_STATIC_CONFIG_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }

    VL53L1_i2c_encode_uint16_t(pdata.dss_config__target_total_rate_mcps, &mut pbuffer[0..2]);
    pbuffer[2] = pdata.debug__ctrl & 0x1;
    pbuffer[3] = pdata.test_mode__ctrl & 0xF;
    pbuffer[4] = pdata.clk_gating__ctrl & 0xF;
    pbuffer[5] = pdata.nvm_bist__ctrl & 0x1F;
    pbuffer[6] = pdata.nvm_bist__num_nvm_words & 0x7F;
    pbuffer[7] = pdata.nvm_bist__start_address & 0x7F;
    pbuffer[8] = pdata.host_if__status & 0x1;
    pbuffer[9] = pdata.pad_i2c_hv__config;
    pbuffer[10] = pdata.pad_i2c_hv__extsup_config & 0x1;
    pbuffer[11] = pdata.gpio_hv_pad__ctrl & 0x3;
    pbuffer[12] = pdata.gpio_hv_mux__ctrl & 0x1F;
    pbuffer[13] = pdata.gpio__tio_hv_status & 0x3;
    pbuffer[14] = pdata.gpio__fio_hv_status & 0x3;
    pbuffer[15] = pdata.ana_config__spad_sel_pswidth & 0x7;
    pbuffer[16] = pdata.ana_config__vcsel_pulse_width_offset & 0x1F;
    pbuffer[17] = pdata.ana_config__fast_osc__config_ctrl & 0x1;
    pbuffer[18] = pdata.sigma_estimator__effective_pulse_width_ns;
    pbuffer[19] = pdata.sigma_estimator__effective_ambient_width_ns;
    pbuffer[20] = pdata.sigma_estimator__sigma_ref_mm;
    pbuffer[21] = pdata.algo__crosstalk_compensation_valid_height_mm;
    pbuffer[22] = pdata.spare_host_config__static_config_spare_0;
    pbuffer[23] = pdata.spare_host_config__static_config_spare_1;
    VL53L1_i2c_encode_uint16_t(
        pdata.algo__range_ignore_threshold_mcps,
        &mut pbuffer[24..26],
    );
    pbuffer[26] = pdata.algo__range_ignore_valid_height_mm;
    pbuffer[27] = pdata.algo__range_min_clip;
    pbuffer[28] = pdata.algo__consistency_check__tolerance & 0xF;
    pbuffer[29] = pdata.spare_host_config__static_config_spare_2;
    pbuffer[30] = pdata.sd_config__reset_stages_msb & 0xF;
    pbuffer[31] = pdata.sd_config__reset_stages_lsb;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_encode_general_config(
    pdata: &mut VL53L1_general_config_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_GENERAL_CONFIG_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }

    pbuffer[0] = pdata.gph_config__stream_count_update_value;
    pbuffer[1] = pdata.global_config__stream_divider;
    pbuffer[2] = pdata.system__interrupt_config_gpio;
    pbuffer[3] = pdata.cal_config__vcsel_start & 0x7F;
    VL53L1_i2c_encode_uint16_t(pdata.cal_config__repeat_rate & 0xFFF, &mut pbuffer[4..6]);
    pbuffer[6] = pdata.global_config__vcsel_width & 0x7F;
    pbuffer[7] = pdata.phasecal_config__timeout_macrop;
    pbuffer[8] = pdata.phasecal_config__target;
    pbuffer[9] = pdata.phasecal_config__override & 0x1;
    pbuffer[11] = pdata.dss_config__roi_mode_control & 0x7;
    VL53L1_i2c_encode_uint16_t(pdata.system__thresh_rate_high, &mut pbuffer[12..14]);
    VL53L1_i2c_encode_uint16_t(pdata.system__thresh_rate_low, &mut pbuffer[14..16]);
    VL53L1_i2c_encode_uint16_t(
        pdata.dss_config__manual_effective_spads_select,
        &mut pbuffer[16..18],
    );
    pbuffer[18] = pdata.dss_config__manual_block_select;
    pbuffer[19] = pdata.dss_config__aperture_attenuation;
    pbuffer[20] = pdata.dss_config__max_spads_limit;
    pbuffer[21] = pdata.dss_config__min_spads_limit;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_encode_timing_config(
    pdata: &mut VL53L1_timing_config_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_TIMING_CONFIG_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }
    pbuffer[0] = pdata.mm_config__timeout_macrop_a_hi & 0xF;
    pbuffer[1] = pdata.mm_config__timeout_macrop_a_lo;
    pbuffer[2] = pdata.mm_config__timeout_macrop_b_hi & 0xF;
    pbuffer[3] = pdata.mm_config__timeout_macrop_b_lo;
    pbuffer[4] = pdata.range_config__timeout_macrop_a_hi & 0xF;
    pbuffer[5] = pdata.range_config__timeout_macrop_a_lo;
    pbuffer[6] = pdata.range_config__vcsel_period_a & 0x3F;
    pbuffer[7] = pdata.range_config__timeout_macrop_b_hi & 0xF;
    pbuffer[8] = pdata.range_config__timeout_macrop_b_lo;
    pbuffer[9] = pdata.range_config__vcsel_period_b & 0x3F;
    VL53L1_i2c_encode_uint16_t(pdata.range_config__sigma_thresh, &mut pbuffer[10..12]);
    VL53L1_i2c_encode_uint16_t(
        pdata.range_config__min_count_rate_rtn_limit_mcps,
        &mut pbuffer[12..14],
    );
    pbuffer[14] = pdata.range_config__valid_phase_low;
    pbuffer[15] = pdata.range_config__valid_phase_high;
    VL53L1_i2c_encode_uint32_t(pdata.system__intermeasurement_period, &mut pbuffer[18..22]);
    pbuffer[22] = pdata.system__fractional_enable & 0x1;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_encode_dynamic_config(
    pdata: &mut VL53L1_dynamic_config_t,
    pbuffer: &mut [u8],
) -> VL53L1_Error {
    if VL53L1_DYNAMIC_CONFIG_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }
    pbuffer[0] = pdata.system__grouped_parameter_hold_0 & 0x3;
    VL53L1_i2c_encode_uint16_t(pdata.system__thresh_high, &mut pbuffer[1..3]);
    VL53L1_i2c_encode_uint16_t(pdata.system__thresh_low, &mut pbuffer[3..5]);
    pbuffer[5] = pdata.system__enable_xtalk_per_quadrant & 0x1;
    pbuffer[6] = pdata.system__seed_config & 0x7;
    pbuffer[7] = pdata.sd_config__woi_sd0;
    pbuffer[8] = pdata.sd_config__woi_sd1;
    pbuffer[9] = pdata.sd_config__initial_phase_sd0 & 0x7F;
    pbuffer[10] = pdata.sd_config__initial_phase_sd1 & 0x7F;
    pbuffer[11] = pdata.system__grouped_parameter_hold_1 & 0x3;
    pbuffer[12] = pdata.sd_config__first_order_select & 0x3;
    pbuffer[13] = pdata.sd_config__quantifier & 0xF;
    pbuffer[14] = pdata.roi_config__user_roi_centre_spad;
    pbuffer[15] = pdata.roi_config__user_roi_requested_global_xy_size;
    pbuffer[16] = pdata.system__sequence_config;
    pbuffer[17] = pdata.system__grouped_parameter_hold & 0x3;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_decode_debug_results(
    pbuffer: &[u8],
    pdata: &mut VL53L1_debug_results_t,
) -> VL53L1_Error {
    if VL53L1_DEBUG_RESULTS_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }
    pdata.phasecal_result__reference_phase = VL53L1_i2c_decode_uint16_t(&pbuffer[0..2]);
    pdata.phasecal_result__vcsel_start = pbuffer[2] & 0x7F;
    pdata.ref_spad_char_result__num_actual_ref_spads = pbuffer[3] & 0x3F;
    pdata.ref_spad_char_result__ref_location = pbuffer[4] & 0x3;
    pdata.vhv_result__coldboot_status = pbuffer[5] & 0x1;
    pdata.vhv_result__search_result = pbuffer[6] & 0x3F;
    pdata.vhv_result__latest_setting = pbuffer[7] & 0x3F;
    pdata.result__osc_calibrate_val = VL53L1_i2c_decode_uint16_t(&pbuffer[8..10]) & 0x3FF;
    pdata.ana_config__powerdown_go1 = pbuffer[10] & 0x3;
    pdata.ana_config__ref_bg_ctrl = pbuffer[11] & 0x3;
    pdata.ana_config__regdvdd1v2_ctrl = pbuffer[12] & 0xF;
    pdata.ana_config__osc_slow_ctrl = pbuffer[13] & 0x7;
    pdata.test_mode__status = pbuffer[14] & 0x1;
    pdata.firmware__system_status = pbuffer[15] & 0x3;
    pdata.firmware__mode_status = pbuffer[16];
    pdata.firmware__secondary_mode_status = pbuffer[17];
    pdata.firmware__cal_repeat_rate_counter = VL53L1_i2c_decode_uint16_t(&pbuffer[18..20]) & 0xFFF;
    pdata.gph__system__thresh_high = VL53L1_i2c_decode_uint16_t(&pbuffer[22..24]);
    pdata.gph__system__thresh_low = VL53L1_i2c_decode_uint16_t(&pbuffer[24..26]);
    pdata.gph__system__enable_xtalk_per_quadrant = pbuffer[26] & 0x1;
    pdata.gph__spare_0 = pbuffer[27] & 0x7;
    pdata.gph__sd_config__woi_sd0 = pbuffer[28];
    pdata.gph__sd_config__woi_sd1 = pbuffer[29];
    pdata.gph__sd_config__initial_phase_sd0 = pbuffer[30] & 0x7F;
    pdata.gph__sd_config__initial_phase_sd1 = pbuffer[31] & 0x7F;
    pdata.gph__sd_config__first_order_select = pbuffer[32] & 0x3;
    pdata.gph__sd_config__quantifier = pbuffer[33] & 0xF;
    pdata.gph__roi_config__user_roi_centre_spad = pbuffer[34];
    pdata.gph__roi_config__user_roi_requested_global_xy_size = pbuffer[35];
    pdata.gph__system__sequence_config = pbuffer[36];
    pdata.gph__gph_id = pbuffer[37] & 0x1;
    pdata.system__interrupt_set = pbuffer[38] & 0x3;
    pdata.interrupt_manager__enables = pbuffer[39] & 0x1F;
    pdata.interrupt_manager__clear = pbuffer[40] & 0x1F;
    pdata.interrupt_manager__status = pbuffer[41] & 0x1F;
    pdata.mcu_to_host_bank__wr_access_en = pbuffer[42] & 0x1;
    pdata.power_management__go1_reset_status = pbuffer[43] & 0x1;
    pdata.pad_startup_mode__value_ro = pbuffer[44] & 0x3;
    pdata.pad_startup_mode__value_ctrl = pbuffer[45] & 0x3F;
    pdata.pll_period_us = VL53L1_i2c_decode_uint32_t(&pbuffer[46..50]) & 0x3FFFF;
    pdata.interrupt_scheduler__data_out = VL53L1_i2c_decode_uint32_t(&pbuffer[50..54]);
    pdata.nvm_bist__complete = pbuffer[54] & 0x1;
    pdata.nvm_bist__status = pbuffer[55] & 0x1;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_decode_core_results(
    pbuffer: &[u8],
    pdata: &mut VL53L1_core_results_t,
) -> VL53L1_Error {
    if VL53L1_CORE_RESULTS_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }
    pdata.result_core__ambient_window_events_sd0 = VL53L1_i2c_decode_uint32_t(&pbuffer[0..4]);
    pdata.result_core__ranging_total_events_sd0 = VL53L1_i2c_decode_uint32_t(&pbuffer[4..8]);
    pdata.result_core__signal_total_events_sd0 = VL53L1_i2c_decode_int32_t(&pbuffer[8..12]);
    pdata.result_core__total_periods_elapsed_sd0 = VL53L1_i2c_decode_uint32_t(&pbuffer[12..16]);
    pdata.result_core__ambient_window_events_sd1 = VL53L1_i2c_decode_uint32_t(&pbuffer[16..20]);
    pdata.result_core__ranging_total_events_sd1 = VL53L1_i2c_decode_uint32_t(&pbuffer[20..24]);
    pdata.result_core__signal_total_events_sd1 = VL53L1_i2c_decode_int32_t(&pbuffer[24..28]);
    pdata.result_core__total_periods_elapsed_sd1 = VL53L1_i2c_decode_uint32_t(&pbuffer[28..32]);
    pdata.result_core__spare_0 = pbuffer[32];
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_i2c_decode_system_results(
    pbuffer: &[u8],
    pdata: &mut VL53L1_system_results_t,
) -> VL53L1_Error {
    if VL53L1_SYSTEM_RESULTS_I2C_SIZE_BYTES > pbuffer.len() {
        return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;
    }
    pdata.result__interrupt_status = pbuffer[0] & 0x3F;
    pdata.result__range_status = pbuffer[1];
    pdata.result__report_status = pbuffer[2] & 0xF;
    pdata.result__stream_count = pbuffer[3];
    pdata.result__dss_actual_effective_spads_sd0 = VL53L1_i2c_decode_uint16_t(&pbuffer[4..6]);
    pdata.result__peak_signal_count_rate_mcps_sd0 = VL53L1_i2c_decode_uint16_t(&pbuffer[6..8]);
    pdata.result__ambient_count_rate_mcps_sd0 = VL53L1_i2c_decode_uint16_t(&pbuffer[8..10]);
    pdata.result__sigma_sd0 = VL53L1_i2c_decode_uint16_t(&pbuffer[10..12]);
    pdata.result__phase_sd0 = VL53L1_i2c_decode_uint16_t(&pbuffer[12..14]);
    pdata.result__final_crosstalk_corrected_range_mm_sd0 =
        VL53L1_i2c_decode_uint16_t(&pbuffer[14..16]);
    pdata.result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
        VL53L1_i2c_decode_uint16_t(&pbuffer[16..18]);
    pdata.result__mm_inner_actual_effective_spads_sd0 =
        VL53L1_i2c_decode_uint16_t(&pbuffer[18..20]);
    pdata.result__mm_outer_actual_effective_spads_sd0 =
        VL53L1_i2c_decode_uint16_t(&pbuffer[20..22]);
    pdata.result__avg_signal_count_rate_mcps_sd0 = VL53L1_i2c_decode_uint16_t(&pbuffer[22..24]);
    pdata.result__dss_actual_effective_spads_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[24..26]);
    pdata.result__peak_signal_count_rate_mcps_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[26..28]);
    pdata.result__ambient_count_rate_mcps_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[28..30]);
    pdata.result__sigma_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[30..32]);
    pdata.result__phase_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[32..34]);
    pdata.result__final_crosstalk_corrected_range_mm_sd1 =
        VL53L1_i2c_decode_uint16_t(&pbuffer[34..36]);
    pdata.result__spare_0_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[36..38]);
    pdata.result__spare_1_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[38..40]);
    pdata.result__spare_2_sd1 = VL53L1_i2c_decode_uint16_t(&pbuffer[40..42]);
    pdata.result__spare_3_sd1 = pbuffer[42];
    pdata.result__thresh_info = pbuffer[43];
    return VL53L1_ERROR_NONE;
}
