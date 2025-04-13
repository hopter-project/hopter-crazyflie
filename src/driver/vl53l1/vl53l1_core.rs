#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::{
    vl53l1_core_support::*, vl53l1_error_codes::*, vl53l1_ll_def::VL53L1_low_power_auto_data_t,
    vl53l1_ll_device::*, vl53l1_register_settings::*, vl53l1_register_structs::*,
};

pub fn VL53L1_i2c_decode_uint16_t(pbuffer: &[u8]) -> u16 {
    /*
     * Decodes a uint16_t from the input I2C read buffer
     * (MS byte first order)
     */
    let mut value = 0_u16;

    for i in 0..pbuffer.len() {
        value = (value << 8) | pbuffer[i] as u16;
    }
    return value;
}

pub fn VL53L1_i2c_decode_int16_t(pbuffer: &[u8]) -> i16 {
    /*
     * Decodes a uint16_t from the input I2C read buffer
     * (MS byte first order)
     */
    let mut value = 0_i16;

    for i in 0..pbuffer.len() {
        value = (value << 8) | pbuffer[i] as i16;
    }
    return value;
}

pub fn VL53L1_copy_rtn_good_spads_to_buffer(pdata: &VL53L1_nvm_copy_data_t, pbuffer: &mut [u8]) {
    pbuffer[0] = pdata.global_config__spad_enables_rtn_0;
    pbuffer[1] = pdata.global_config__spad_enables_rtn_1;
    pbuffer[2] = pdata.global_config__spad_enables_rtn_2;
    pbuffer[3] = pdata.global_config__spad_enables_rtn_3;
    pbuffer[4] = pdata.global_config__spad_enables_rtn_4;
    pbuffer[5] = pdata.global_config__spad_enables_rtn_5;
    pbuffer[6] = pdata.global_config__spad_enables_rtn_6;
    pbuffer[7] = pdata.global_config__spad_enables_rtn_7;
    pbuffer[8] = pdata.global_config__spad_enables_rtn_8;
    pbuffer[9] = pdata.global_config__spad_enables_rtn_9;
    pbuffer[10] = pdata.global_config__spad_enables_rtn_10;
    pbuffer[11] = pdata.global_config__spad_enables_rtn_11;
    pbuffer[12] = pdata.global_config__spad_enables_rtn_12;
    pbuffer[13] = pdata.global_config__spad_enables_rtn_13;
    pbuffer[14] = pdata.global_config__spad_enables_rtn_14;
    pbuffer[15] = pdata.global_config__spad_enables_rtn_15;
    pbuffer[16] = pdata.global_config__spad_enables_rtn_16;
    pbuffer[17] = pdata.global_config__spad_enables_rtn_17;
    pbuffer[18] = pdata.global_config__spad_enables_rtn_18;
    pbuffer[19] = pdata.global_config__spad_enables_rtn_19;
    pbuffer[20] = pdata.global_config__spad_enables_rtn_20;
    pbuffer[21] = pdata.global_config__spad_enables_rtn_21;
    pbuffer[22] = pdata.global_config__spad_enables_rtn_22;
    pbuffer[23] = pdata.global_config__spad_enables_rtn_23;
    pbuffer[24] = pdata.global_config__spad_enables_rtn_24;
    pbuffer[25] = pdata.global_config__spad_enables_rtn_25;
    pbuffer[26] = pdata.global_config__spad_enables_rtn_26;
    pbuffer[27] = pdata.global_config__spad_enables_rtn_27;
    pbuffer[28] = pdata.global_config__spad_enables_rtn_28;
    pbuffer[29] = pdata.global_config__spad_enables_rtn_29;
    pbuffer[30] = pdata.global_config__spad_enables_rtn_30;
    pbuffer[31] = pdata.global_config__spad_enables_rtn_31;
}

pub fn VL53L1_calc_range_ignore_threshold(
    central_rate: u32,
    x_gradient: i16,
    y_gradient: i16,
    rate_mult: u8,
) -> u16 {
    let mut range_ignore_thresh_int = 0_i32;
    let mut range_ignore_thresh_kcps = 0_u16;
    let mut central_rate_int = 0_i32;
    let mut x_gradient_int = 0_i16;
    let mut y_gradient_int = 0_i16;

    /* Shift central_rate to .13 fractional for simple addition */
    central_rate_int = central_rate as i32 * (1 << 4) / 1000;
    if x_gradient < 0 {
        x_gradient_int = -x_gradient;
    }

    if y_gradient < 0 {
        y_gradient_int = -y_gradient;
    }

    /* Calculate full rate per spad - worst case from measured xtalk */
    /* Generated here from .11 fractional kcps */
    /* Additional factor of 4 applied to bring fractional precision to .13 */
    range_ignore_thresh_int = 8 * x_gradient_int as i32 * 4 + 8 * y_gradient_int as i32 * 4;

    /* Convert Kcps to Mcps */

    range_ignore_thresh_int = range_ignore_thresh_int / 1000;

    /* Combine with Central Rate - Mcps .13 format*/

    range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;
    /* Mult by user input */

    range_ignore_thresh_int = rate_mult as i32 * range_ignore_thresh_int;

    range_ignore_thresh_int = (range_ignore_thresh_int + (1 << 4)) / (1 << 5);

    /* Finally clip and output in correct format */

    if range_ignore_thresh_int > 0xFFFF {
        range_ignore_thresh_kcps = 0xFFFF;
    } else {
        range_ignore_thresh_kcps = range_ignore_thresh_int as u16;
    }

    return range_ignore_thresh_kcps;
}

pub fn VL53L1_config_low_power_auto_mode(
    pgeneral: &mut VL53L1_general_config_t,
    pdynamic: &mut VL53L1_dynamic_config_t,
    plpadata: &mut VL53L1_low_power_auto_data_t,
) -> VL53L1_Error {
    /* set low power auto mode */
    plpadata.is_low_power_auto_mode = 1;

    /* set low power range count to 0 */
    plpadata.low_power_auto_range_count = 0;

    /* Turn off MM1/MM2 and DSS2 */
    pdynamic.system__sequence_config = VL53L1_SEQUENCE_VHV_EN |
			VL53L1_SEQUENCE_PHASECAL_EN |
			VL53L1_SEQUENCE_DSS1_EN |
			VL53L1_SEQUENCE_DSS2_EN |
			/* VL53L1_SEQUENCE_MM1_EN |*/
			/* VL53L1_SEQUENCE_MM2_EN |*/
			VL53L1_SEQUENCE_RANGE_EN;
    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_calc_timeout_register_values(
    phasecal_config_timout_us: u32,
    mm_config_timeout_us: u32,
    range_config_timeout_us: u32,
    fast_osc_frequency: u16,
    pgeneral: &mut VL53L1_general_config_t,
    ptiming: &mut VL53L1_timing_config_t,
) -> VL53L1_Error {
    /*
     * Converts the input MM and range timeouts in [us]
     * into the appropriate register values
     *
     * Must also be run after the VCSEL period settings are changed
     */

    let mut macro_period_us = 0_u32;
    let mut timeout_mclks = 0_u32;
    let mut timeout_encoded = 0_u16;
    if fast_osc_frequency == 0 {
        return VL53L1_ERROR_DIVISION_BY_ZERO;
    } else {
        /* Update Macro Period for Range A VCSEL Period */
        macro_period_us =
            VL53L1_calc_macro_period_us(fast_osc_frequency, ptiming.range_config__vcsel_period_a);
        /*  Update Phase timeout - uses Timing A */
        timeout_mclks = VL53L1_calc_timeout_mclks(phasecal_config_timout_us, macro_period_us);
        /* clip as the phase cal timeout register is only 8-bits */
        if timeout_mclks > 0xFF {
            timeout_mclks = 0xFF;
        }
        pgeneral.phasecal_config__timeout_macrop = timeout_mclks as u8;
        /*  Update MM Timing A timeout */
        timeout_encoded = VL53L1_calc_encoded_timeout(mm_config_timeout_us, macro_period_us);

        ptiming.mm_config__timeout_macrop_a_hi = ((timeout_encoded & 0xFF00) >> 8) as u8;
        ptiming.mm_config__timeout_macrop_a_lo = (timeout_encoded & 0x00FF) as u8;
        /* Update Range Timing A timeout */
        timeout_encoded = VL53L1_calc_encoded_timeout(range_config_timeout_us, macro_period_us);
        ptiming.range_config__timeout_macrop_a_hi = ((timeout_encoded & 0xFF00) >> 8) as u8;
        ptiming.range_config__timeout_macrop_a_lo = (timeout_encoded & 0x00FF) as u8;

        /* Update Macro Period for Range B VCSEL Period */
        macro_period_us =
            VL53L1_calc_macro_period_us(fast_osc_frequency, ptiming.range_config__vcsel_period_b);
        /* Update MM Timing B timeout */
        timeout_encoded = VL53L1_calc_encoded_timeout(mm_config_timeout_us, macro_period_us);
        ptiming.mm_config__timeout_macrop_b_hi = ((timeout_encoded & 0xFF00) >> 8) as u8;
        ptiming.mm_config__timeout_macrop_b_lo = (timeout_encoded & 0x00FF) as u8;

        /* Update Range Timing B timeout */
        timeout_encoded = VL53L1_calc_encoded_timeout(range_config_timeout_us, macro_period_us);
        ptiming.range_config__timeout_macrop_b_hi = ((timeout_encoded & 0xFF00) >> 8) as u8;
        ptiming.range_config__timeout_macrop_b_lo = (timeout_encoded & 0x00FF) as u8;
    }

    return VL53L1_ERROR_NONE;
}

pub fn VL53L1_calc_encoded_timeout(timeout_us: u32, macro_period_us: u32) -> u16 {
    /*  Calculates the encoded timeout register value based on the input
     *  timeout period in milliseconds and the macro period in [us]
     *
     *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
     *  Max timeout in 20.12 format = 32-bits
     *
     *  Macro period [us] = 12.12 format
     */
    let mut timeout_encoded = 0_u16;
    let mut timeout_mclks = 0_u32;
    timeout_mclks = VL53L1_calc_timeout_mclks(timeout_us, macro_period_us);
    timeout_encoded = VL53L1_encode_timeout(timeout_mclks);
    return timeout_encoded;
}

pub fn VL53L1_encode_timeout(timeout_mclks: u32) -> u16 {
    /*
     * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
     */
    let mut encoded_timeout = 0_u16;
    let mut ls_byte = 0_u32;
    let mut ms_byte = 0_u16;

    if timeout_mclks > 0 {
        ls_byte = timeout_mclks - 1;

        while (ls_byte & 0xFFFFFF00) > 0 {
            ls_byte = ls_byte >> 1;
            ms_byte += 1;
        }

        encoded_timeout = (ms_byte << 8) + (ls_byte & 0x000000FF) as u16;
    }

    return encoded_timeout;
}

pub fn VL53L1_calc_timeout_mclks(timeout_us: u32, macro_period_us: u32) -> u32 {
    /*  Calculates the timeout value in macro periods based on the input
     *  timeout period in milliseconds and the macro period in [us]
     *
     *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
     *  Max timeout in 20.12 format = 32-bits
     *
     *  Macro period [us] = 12.12 format
     */
    ((timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us
}

pub fn VL53L1_calc_macro_period_us(fast_osc_frequency: u16, vcsel_period: u8) -> u32 {
    let mut pll_period_us = 0_u32;
    let mut vcsel_period_pclks = 0_u8;
    let mut macro_period_us = 0_u32;
    /*  Calculate PLL period in [us] from the  fast_osc_frequency
     *  Fast osc frequency fixed point format = unsigned 4.12
     */
    pll_period_us = VL53L1_calc_pll_period_us(fast_osc_frequency);
    /*  VCSEL period
     *  - the real VCSEL period in PLL clocks = 2*(VCSEL_PERIOD+1)
     */
    vcsel_period_pclks = VL53L1_decode_vcsel_period(vcsel_period);

    /*  Macro period
     *  - PLL period [us]      = 0.24 format
     *      - for 1.0 MHz fast oscillator freq
     *      - max PLL period = 1/64 (6-bits)
     *      - i.e only the lower 18-bits of PLL Period value are used
     *  - Macro period [vclks] = 2304 (12-bits)
     *
     *  Max bits (24 - 6) + 12 = 30-bits usage
     *
     *  Downshift by 6 before multiplying by the VCSEL Period
     */
    macro_period_us = VL53L1_MACRO_PERIOD_VCSEL_PERIODS * pll_period_us;
    macro_period_us = macro_period_us >> 6;

    macro_period_us = macro_period_us * vcsel_period_pclks as u32;
    macro_period_us = macro_period_us >> 6;

    return macro_period_us;
}

pub fn VL53L1_calc_timeout_us(timeout_mclks: u32, macro_period_us: u32) -> u32 {
    /*  Calculates the timeout value in microseconds based on the input
     *  timeout period in macro periods and the macro period in [us]
     *
     *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
     *  Max timeout in 20.12 format = 32-bits
     *
     *  Macro period [us] = 12.12 format
     */
    let mut tmp = 0_u64;
    tmp = timeout_mclks as u64 * macro_period_us as u64;
    return ((tmp + 0x800) >> 12) as u32;
}

pub fn VL53L1_calc_decoded_timeout_us(timeout_encoded: u16, macro_period_us: u32) -> u32 {
    /*  Calculates the timeout value in microseconds based on the input
     *  timeout period in macro periods and the macro period in [us]
     *
     *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
     *  Max timeout in 20.12 format = 32-bits
     *
     *  Macro period [us] = 12.12 format
     */
    let mut timeout_mclks = 0_u32;
    timeout_mclks = VL53L1_decode_timeout(timeout_encoded);
    return VL53L1_calc_timeout_us(timeout_mclks, macro_period_us);
}

pub fn VL53L1_decode_timeout(encoded_timeout: u16) -> u32 {
    /*
     * Decode 16-bit timeout register value
     * format (LSByte * 2^MSByte) + 1
     */
    (((encoded_timeout & 0x00FF) as u32) << ((encoded_timeout & 0xFF00) >> 8) as u32) + 1
}

pub fn VL53L1_decode_zone_size(encoded_xy_size: u8, pwidth: &mut u8, pheight: &mut u8) {
    /* extract x and y sizes
     *
     * Important: the sense of the device width and height is swapped
     * versus the API sense
     *
     * MS Nibble = height
     * LS Nibble = width
     */
    *pheight = encoded_xy_size >> 4;
    *pwidth = encoded_xy_size & 0x0F;
}

pub fn VL53L1_encode_row_col(row: u8, col: u8, pspad_number: &mut u8) {
    /*
     *  Encodes the input array(row,col) location as SPAD number.
     */
    if row > 7 {
        *pspad_number = 128 + (col << 3) + (15 - row);
    } else {
        *pspad_number = ((15 - row) << 3) + row;
    }
}

pub fn VL53L1_encode_zone_size(width: u8, height: u8, pencoded_xy_size: &mut u8) {
    /* encode x and y sizes
     *
     * Important: the sense of the device width and height is swapped
     * versus the API sense
     *
     * MS Nibble = height
     * LS Nibble = width
     */
    *pencoded_xy_size = (height << 4) + width;
}

pub fn VL53L1_i2c_encode_uint16_t(ip_value: u16, pbuffer: &mut [u8]) {
    pbuffer[1] = (ip_value & 0x00FF) as u8;
    pbuffer[0] = ((ip_value & 0xFF00) >> 8) as u8;
}

pub fn VL53L1_i2c_encode_int16_t(ip_value: i16, pbuffer: &mut [u8]) {
    pbuffer[1] = (ip_value & 0x00FF) as u8;
    pbuffer[0] = ((ip_value >> 8) & 0x00FF) as u8;
}

pub fn VL53L1_i2c_encode_uint32_t(ip_value: u32, pbuffer: &mut [u8]) {
    pbuffer[3] = (ip_value & 0x000000FF) as u8;
    pbuffer[2] = ((ip_value & 0x0000FF00) >> 8) as u8;
    pbuffer[1] = ((ip_value & 0x00FF0000) >> 16) as u8;
    pbuffer[0] = ((ip_value & 0xFF000000) >> 24) as u8;
}

pub fn VL53L1_i2c_decode_uint32_t(pbuffer: &[u8]) -> u32 {
    let mut value = 0_u32;
    for i in 0..4 {
        value = (value << 8) | pbuffer[i] as u32;
    }
    return value;
}

pub fn VL53L1_i2c_decode_int32_t(pbuffer: &[u8]) -> i32 {
    let mut value = 0_i32;
    for i in 0..4 {
        value = (value << 8) | pbuffer[i] as i32;
    }
    return value;
}
