#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
pub fn VL53L1_decode_row_col(spad_number: u8, prow: &mut u8, pcol: &mut u8) {
    if spad_number > 127 {
        *prow = 8 + ((255 - spad_number) & 0x07);
        *pcol = (spad_number - 128) >> 3;
    } else {
        *prow = spad_number & 0x07;
        *pcol = (127 - spad_number) >> 3;
    }
}

pub fn VL53L1_calc_pll_period_us(fast_osc_frequency: u16) -> u32 {
    /*  Calculates PLL frequency using NVM fast_osc_frequency
     *  Fast osc frequency fixed point format = unsigned 4.12
     *
     *  PLL period fixed point format = unsigned 0.24
     *  Min input fast osc frequency  = 1 MHz
     *  PLL Multiplier = 64 (fixed)
     *  Min PLL freq = 64.0MHz
     *  -> max PLL period = 1/ 64
     *  ->  only the 18 LS bits are used
     *
     *  2^30 = (2^24) (1.0us) * 4096 (2^12) / 64 (PLL Multiplier)
     */
    (0x01 << 30) / (fast_osc_frequency as u32)
}

pub fn VL53L1_decode_vcsel_period(vcsel_period_reg: u8) -> u8 {
    /*
     * Converts the encoded VCSEL period register value into
     * the real period in PLL clocks
     */
    (vcsel_period_reg + 1) << 1
}
