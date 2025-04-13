#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::{
    vl53l1_core::VL53L1_config_low_power_auto_mode, vl53l1_error_codes::*, vl53l1_ll_def::*,
    vl53l1_ll_device::*, vl53l1_register_settings::*, vl53l1_register_structs::*,
};

pub fn VL53L1_preset_mode_standard_ranging(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Static Configuration */
    /* dss_config__target_total_rate_mcps = 20.0 Mcps 9.7 fp */
    pstatic.dss_config__target_total_rate_mcps = 0x0A00;
    pstatic.debug__ctrl = 0x00;
    pstatic.test_mode__ctrl = 0x00;
    pstatic.clk_gating__ctrl = 0x00;
    pstatic.nvm_bist__ctrl = 0x00;
    pstatic.nvm_bist__num_nvm_words = 0x00;
    pstatic.nvm_bist__start_address = 0x00;
    pstatic.host_if__status = 0x00;
    pstatic.pad_i2c_hv__config = 0x00;
    pstatic.pad_i2c_hv__extsup_config = 0x00;

    /*
     *  0 - gpio__extsup_hv
     *  1 - gpio__vmodeint_hv
     */
    pstatic.gpio_hv_pad__ctrl = 0x00;

    /*
     * Set interrupt active low
     *
     *  3:0 - gpio__mux_select_hv
     *    4 - gpio__mux_active_high_hv
     */
    pstatic.gpio_hv_mux__ctrl = VL53L1_DEVICEINTERRUPTPOLARITY_ACTIVE_LOW
        | VL53L1_DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS;

    pstatic.gpio__tio_hv_status = 0x02;
    pstatic.gpio__fio_hv_status = 0x00;
    pstatic.ana_config__spad_sel_pswidth = 0x02;
    pstatic.ana_config__vcsel_pulse_width_offset = 0x08;
    pstatic.ana_config__fast_osc__config_ctrl = 0x00;

    pstatic.sigma_estimator__effective_pulse_width_ns =
        ptuning_parms.tp_lite_sigma_est_pulse_width_ns;
    pstatic.sigma_estimator__effective_ambient_width_ns =
        ptuning_parms.tp_lite_sigma_est_amb_width_ns;
    pstatic.sigma_estimator__sigma_ref_mm = ptuning_parms.tp_lite_sigma_ref_mm;
    /* Minimum allowable value of 1 - 0 disables the feature */
    pstatic.algo__crosstalk_compensation_valid_height_mm = 0x01;
    pstatic.spare_host_config__static_config_spare_0 = 0x00;
    pstatic.spare_host_config__static_config_spare_1 = 0x00;

    pstatic.algo__range_ignore_threshold_mcps = 0x0000;

    /* set RIT distance to 20 mm */
    pstatic.algo__range_ignore_valid_height_mm = 0xff;
    pstatic.algo__range_min_clip = ptuning_parms.tp_lite_min_clip;
    /*
     * Phase consistency check limit - format 1.3 fp
     * 0x02 . 0.25
     * 0x08 . 1.00
     */
    pstatic.algo__consistency_check__tolerance = ptuning_parms.tp_consistency_lite_phase_tolerance;
    pstatic.spare_host_config__static_config_spare_2 = 0x00;
    pstatic.sd_config__reset_stages_msb = 0x00;
    pstatic.sd_config__reset_stages_lsb = 0x00;

    pgeneral.gph_config__stream_count_update_value = 0x00;
    pgeneral.global_config__stream_divider = 0x00;
    pgeneral.system__interrupt_config_gpio = VL53L1_INTERRUPT_CONFIG_NEW_SAMPLE_READY;
    pgeneral.cal_config__vcsel_start = 0x0B;

    /*
     * Set VHV / Phase Cal repeat rate to 1 every
     * 60 * 60 ranges (once every minute @ 60Hz)
     * 0 - disables
     * 12-bit value . 4095 max
     */
    pgeneral.cal_config__repeat_rate = ptuning_parms.tp_cal_repeat_rate;
    pgeneral.global_config__vcsel_width = 0x02;
    /* 13 macro periods gives a timeout of 1ms */
    pgeneral.phasecal_config__timeout_macrop = 0x0D;
    /* Phase cal target phase 2.0625 - 4.4 fp . 0x21*/
    pgeneral.phasecal_config__target = ptuning_parms.tp_phasecal_target;
    pgeneral.phasecal_config__override = 0x00;
    pgeneral.dss_config__roi_mode_control = VL53L1_DEVICEDSSMODE__TARGET_RATE;
    /* format for threshold high and low is 9.7 fp */
    pgeneral.system__thresh_rate_high = 0x0000;
    pgeneral.system__thresh_rate_low = 0x0000;
    /* The format for manual effective spads is 8.8 . 0x8C00 = 140.00 */
    pgeneral.dss_config__manual_effective_spads_select = 0x8C00;
    pgeneral.dss_config__manual_block_select = 0x00;

    /*
     * Aperture attenuation value - format 0.8
     *
     * Nominal:  5x   . 0.200000 * 256 = 51 = 0x33
     * Measured: 4.6x . 0.217391 * 256 = 56 = 0x38
     */
    pgeneral.dss_config__aperture_attenuation = 0x38;
    pgeneral.dss_config__max_spads_limit = 0xFF;
    pgeneral.dss_config__min_spads_limit = 0x01;

    /* Timing Configuration */

    /* Default timing of 2ms */
    ptiming.mm_config__timeout_macrop_a_hi = 0x00;
    ptiming.mm_config__timeout_macrop_a_lo = 0x1a;
    ptiming.mm_config__timeout_macrop_b_hi = 0x00;
    ptiming.mm_config__timeout_macrop_b_lo = 0x20;
    /* Setup for 30ms default */
    ptiming.range_config__timeout_macrop_a_hi = 0x01;
    ptiming.range_config__timeout_macrop_a_lo = 0xCC;
    /* register value 11 gives a 24 VCSEL period */
    ptiming.range_config__vcsel_period_a = 0x0B;
    /* Setup for 30ms default */
    ptiming.range_config__timeout_macrop_b_hi = 0x01;
    ptiming.range_config__timeout_macrop_b_lo = 0xF5;
    /* register value  09 gives a 20 VCSEL period */
    ptiming.range_config__vcsel_period_b = 0x09;
    /*
     * Sigma thresh register - format 14.2
     *
     * 0x003C . 15.0 mm
     * 0x0050 . 20.0 mm
     */
    ptiming.range_config__sigma_thresh = ptuning_parms.tp_lite_med_sigma_thresh_mm;
    /*
     *  Rate Limit - format 9.7fp
     *  0x0020 . 0.250 Mcps
     *  0x0080 . 1.000 Mcps
     */
    ptiming.range_config__min_count_rate_rtn_limit_mcps =
        ptuning_parms.tp_lite_med_min_count_rate_rtn_mcps;

    /* Phase limit register formats = 5.3
     * low   = 0x08 .  1.0
     * high  = 0x78 . 15.0 . 3.0m
     */
    ptiming.range_config__valid_phase_low = 0x08;
    ptiming.range_config__valid_phase_high = 0x78;
    ptiming.system__intermeasurement_period = 0x00000000;
    ptiming.system__fractional_enable = 0x00;

    /* Dynamic Configuration */

    pdynamic.system__grouped_parameter_hold_0 = 0x01;

    pdynamic.system__thresh_high = 0x0000;
    pdynamic.system__thresh_low = 0x0000;
    pdynamic.system__enable_xtalk_per_quadrant = 0x00;
    pdynamic.system__seed_config = ptuning_parms.tp_lite_seed_cfg;

    /* Timing A */
    pdynamic.sd_config__woi_sd0 = 0x0B;
    /* Timing B */
    pdynamic.sd_config__woi_sd1 = 0x09;

    pdynamic.sd_config__initial_phase_sd0 = ptuning_parms.tp_init_phase_rtn_lite_med;
    pdynamic.sd_config__initial_phase_sd1 = ptuning_parms.tp_init_phase_ref_lite_med;

    pdynamic.system__grouped_parameter_hold_1 = 0x01;

    /*
     *  Quantifier settings
     *
     *  sd_config__first_order_select
     *     bit 0 - return sigma delta
     *     bit 1 - reference sigma delta
     *
     *  sd_config__first_order_select = 0x03 (1st order)
     *
     *      sd_config__quantifier options
     *        0
     *        1 .   64
     *        2 .  128
     *        3 .  256
     *
     *  sd_config__first_order_select = 0x00 (2nd order)
     *
     *      sd_config__quantifier options
     *        0
     *        1  .  256
     *        2  . 1024
     *        3  . 4095
     *
     *  Setting below 2nd order, Quantifier = 1024
     */

    pdynamic.sd_config__first_order_select = ptuning_parms.tp_lite_first_order_select;
    pdynamic.sd_config__quantifier = ptuning_parms.tp_lite_quantifier;

    /* Below defaults will be overwritten by zone_cfg
     * Spad no = 199 (0xC7)
     * Spad no =  63 (0x3F)
     */
    pdynamic.roi_config__user_roi_centre_spad = 0xC7;
    /* 16x16 ROI */
    pdynamic.roi_config__user_roi_requested_global_xy_size = 0xFF;

    pdynamic.system__sequence_config = VL53L1_SEQUENCE_VHV_EN
        | VL53L1_SEQUENCE_PHASECAL_EN
        | VL53L1_SEQUENCE_DSS1_EN
        | VL53L1_SEQUENCE_DSS2_EN
        | VL53L1_SEQUENCE_MM2_EN
        | VL53L1_SEQUENCE_RANGE_EN;

    pdynamic.system__grouped_parameter_hold = 0x02;

    /* System control */

    psystem.system__stream_count_ctrl = 0x00;
    psystem.firmware__enable = 0x01;
    psystem.system__interrupt_clear = VL53L1_CLEAR_RANGE_INT;

    psystem.system__mode_start = VL53L1_DEVICESCHEDULERMODE_STREAMING
        | VL53L1_DEVICEREADOUTMODE_SINGLE_SD
        | VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK;

    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_standard_ranging_short_range(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration followed by
     * overrides for the short range configuration
     */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* Timing Configuration
     *
     * vcsel_period_a    = 7 -> 16 period
     * vcsel_period_b    = 5 -> 12 period
     * sigma_thresh                  = 0x003C -> 14.2fp -> 15.0 mm
     * min_count_rate_rtn_limit_mcps = 0x0080 ->  9.7fp ->  1.0 Mcps
     * valid_phase_low               = 0x08 -> 5.3fp -> 1.0
     * valid_phase_high              = 0x38 -> 5.3fp -> 7.0 -> 1.4m
     */
    ptiming.range_config__vcsel_period_a = 0x07;
    ptiming.range_config__vcsel_period_b = 0x05;
    ptiming.range_config__sigma_thresh = ptuning_parms.tp_lite_short_sigma_thresh_mm;
    ptiming.range_config__min_count_rate_rtn_limit_mcps =
        ptuning_parms.tp_lite_short_min_count_rate_rtn_mcps;
    ptiming.range_config__valid_phase_low = 0x08;
    ptiming.range_config__valid_phase_high = 0x38;

    /* Dynamic Configuration
     * SD0 -> Timing A
     * SD1 -> Timing B
     */
    pdynamic.sd_config__woi_sd0 = 0x07;
    pdynamic.sd_config__woi_sd1 = 0x05;
    pdynamic.sd_config__initial_phase_sd0 = ptuning_parms.tp_init_phase_rtn_lite_short;
    pdynamic.sd_config__initial_phase_sd1 = ptuning_parms.tp_init_phase_ref_lite_short;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_standard_ranging_long_range(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration with
     * overrides for long range configuration
     */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* Timing Configuration
     *
     * vcsel_period_a    = 15 -> 32 period
     * vcsel_period_b    = 13 -> 28 period
     * sigma_thresh                  = 0x003C -> 14.2fp -> 15.0 mm
     * min_count_rate_rtn_limit_mcps = 0x0080 ->  9.7fp ->  1.0 Mcps
     * valid_phase_low               = 0x08 -> 5.3fp ->  1.0
     * valid_phase_high              = 0xB8 -> 5.3fp -> 23.0 -> 4.6m
     */
    ptiming.range_config__vcsel_period_a = 0x0F;
    ptiming.range_config__vcsel_period_b = 0x0D;
    ptiming.range_config__sigma_thresh = ptuning_parms.tp_lite_long_sigma_thresh_mm;
    ptiming.range_config__min_count_rate_rtn_limit_mcps =
        ptuning_parms.tp_lite_long_min_count_rate_rtn_mcps;
    ptiming.range_config__valid_phase_low = 0x08;
    ptiming.range_config__valid_phase_high = 0xB8;

    /* Dynamic Configuration
     * SD0 -> Timing A
     * SD1 -> Timing B
     */
    pdynamic.sd_config__woi_sd0 = 0x0F;
    pdynamic.sd_config__woi_sd1 = 0x0D;
    pdynamic.sd_config__initial_phase_sd0 = ptuning_parms.tp_init_phase_rtn_lite_long;
    pdynamic.sd_config__initial_phase_sd1 = ptuning_parms.tp_init_phase_ref_lite_long;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_standard_ranging_mm1_cal(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration with
     * overrides for long range configuration
     */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    pgeneral.dss_config__roi_mode_control = VL53L1_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
    pdynamic.system__sequence_config = VL53L1_SEQUENCE_VHV_EN
        | VL53L1_SEQUENCE_PHASECAL_EN
        | VL53L1_SEQUENCE_DSS1_EN
        | VL53L1_SEQUENCE_DSS2_EN
        | VL53L1_SEQUENCE_MM1_EN;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_standard_ranging_mm2_cal(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration with
     * overrides for long range configuration
     */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    pgeneral.dss_config__roi_mode_control = VL53L1_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
    pdynamic.system__sequence_config = VL53L1_SEQUENCE_VHV_EN
        | VL53L1_SEQUENCE_PHASECAL_EN
        | VL53L1_SEQUENCE_DSS1_EN
        | VL53L1_SEQUENCE_DSS2_EN
        | VL53L1_SEQUENCE_MM2_EN;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_timed_ranging(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* now override standard ranging specific registers */
    /* Dynamic Configuration */
    /* Disable GPH  */
    pdynamic.system__grouped_parameter_hold = 0x00;

    /* Re-Configure timing budget default for 13ms */
    ptiming.range_config__timeout_macrop_a_hi = 0x00;
    ptiming.range_config__timeout_macrop_a_lo = 0xB1;
    /* Setup for 13ms default */
    ptiming.range_config__timeout_macrop_b_hi = 0x00;
    ptiming.range_config__timeout_macrop_b_lo = 0xD4;

    /* Timing Configuration */
    ptiming.system__intermeasurement_period = 0x00000600;
    pdynamic.system__seed_config = ptuning_parms.tp_timed_seed_cfg;

    /* System control */
    /* Configure Timed/Psuedo-solo mode */
    psystem.system__mode_start = VL53L1_DEVICESCHEDULERMODE_PSEUDO_SOLO
        | VL53L1_DEVICEREADOUTMODE_SINGLE_SD
        | VL53L1_DEVICEMEASUREMENTMODE_TIMED;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_timed_ranging_short_range(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* Dynamic Configuration */
    /* Disable GPH  */
    pdynamic.system__grouped_parameter_hold = 0x00;

    /* Timing Configuration */
    /* Re-Configure timing budget default for 13ms */
    ptiming.range_config__timeout_macrop_a_hi = 0x01;
    ptiming.range_config__timeout_macrop_a_lo = 0x84;
    /* Setup for 13ms default */
    ptiming.range_config__timeout_macrop_b_hi = 0x01;
    ptiming.range_config__timeout_macrop_b_lo = 0xB1;

    ptiming.system__intermeasurement_period = 0x00000600;
    pdynamic.system__seed_config = ptuning_parms.tp_timed_seed_cfg;

    /* System control */
    /* Configure Timed/Psuedo-solo mode */
    psystem.system__mode_start = VL53L1_DEVICESCHEDULERMODE_PSEUDO_SOLO
        | VL53L1_DEVICEREADOUTMODE_SINGLE_SD
        | VL53L1_DEVICEMEASUREMENTMODE_TIMED;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_timed_ranging_long_range(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_standard_ranging_long_range(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* Dynamic Configuration */
    /* Disable GPH  */
    pdynamic.system__grouped_parameter_hold = 0x00;

    /* Timing Configuration */
    /* Re-Configure timing budget default for 13ms */
    ptiming.range_config__timeout_macrop_a_hi = 0x00;
    ptiming.range_config__timeout_macrop_a_lo = 0x97;
    /* Setup for 13ms default */
    ptiming.range_config__timeout_macrop_b_hi = 0x00;
    ptiming.range_config__timeout_macrop_b_lo = 0xB1;

    ptiming.system__intermeasurement_period = 0x00000600;
    pdynamic.system__seed_config = ptuning_parms.tp_timed_seed_cfg;

    /* System control */
    /* Configure Timed/Psuedo-solo mode */
    psystem.system__mode_start = VL53L1_DEVICESCHEDULERMODE_PSEUDO_SOLO
        | VL53L1_DEVICEREADOUTMODE_SINGLE_SD
        | VL53L1_DEVICEMEASUREMENTMODE_TIMED;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_low_power_auto_ranging(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
    plpadata: &mut VL53L1_low_power_auto_data_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_timed_ranging(pstatic, pgeneral, ptiming, pdynamic, psystem, ptuning_parms);
    return VL53L1_config_low_power_auto_mode(pgeneral, pdynamic, plpadata);
}

pub fn VL53L1_preset_mode_low_power_auto_short_ranging(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
    plpadata: &mut VL53L1_low_power_auto_data_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_timed_ranging_short_range(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    return VL53L1_config_low_power_auto_mode(pgeneral, pdynamic, plpadata);
}

pub fn VL53L1_preset_mode_low_power_auto_long_ranging(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
    plpadata: &mut VL53L1_low_power_auto_data_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_timed_ranging_long_range(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    return VL53L1_config_low_power_auto_mode(pgeneral, pdynamic, plpadata);
}

pub fn VL53L1_preset_mode_singleshot_ranging(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* Dynamic Configuration */
    /* Disable GPH  */
    pdynamic.system__grouped_parameter_hold = 0x00;

    /* Timing Configuration */
    /* Re-Configure timing budget default for 13ms */
    ptiming.range_config__timeout_macrop_a_hi = 0x00;
    ptiming.range_config__timeout_macrop_a_lo = 0xB1;
    /* Setup for 13ms default */
    ptiming.range_config__timeout_macrop_b_hi = 0x00;
    ptiming.range_config__timeout_macrop_b_lo = 0xD4;

    pdynamic.system__seed_config = ptuning_parms.tp_timed_seed_cfg;

    /* System control */
    /* Configure Timed/Psuedo-solo mode */
    psystem.system__mode_start = VL53L1_DEVICESCHEDULERMODE_PSEUDO_SOLO
        | VL53L1_DEVICEREADOUTMODE_SINGLE_SD
        | VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_preset_mode_olt(
    pstatic: &mut VL53L1_static_config_t,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    psystem: &mut VL53L1_system_control_t,
    ptuning_parms: &mut VL53L1_tuning_parm_storage_t,
) -> VL53L1_Error {
    /* Call standard ranging configuration */
    VL53L1_preset_mode_standard_ranging(
        pstatic,
        pgeneral,
        ptiming,
        pdynamic,
        psystem,
        ptuning_parms,
    );
    /* now override OLT specific registers */
    psystem.system__stream_count_ctrl = 0x01;
    return VL53L1_ERROR_NONE;
}
