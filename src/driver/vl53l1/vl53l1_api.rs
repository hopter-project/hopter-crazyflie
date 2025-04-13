#![allow(non_snake_case)]
#![allow(unused)]
#![allow(non_camel_case_types)]
use super::{
    vl53l1_def::*, vl53l1_error_codes::*, vl53l1_ll_def::*, vl53l1_ll_device::*,
    vl53l1_preset_setup::*,
};

pub const FDA_MAX_TIMING_BUDGET_US: u32 = 550000;
pub const LOWPOWER_AUTO_VHV_LOOP_DURATION_US: u32 = 245;
pub const LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING: u32 = 1448;
pub const LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING: u32 = 2100;

/* Bare Driver Tuning parameter table indexed with VL53L1_Tuning_t */
pub const BDTABLE: [i32; VL53L1_Tuning_t::VL53L1_TUNING_MAX_TUNABLE_KEY as usize] = [
    TUNING_VERSION,
    TUNING_PROXY_MIN,
    TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
    TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
    TUNING_MIN_AMBIENT_DMAX_VALID,
    TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
    TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
    TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
    TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
    TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
    TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT,
    TUNING_PHASECAL_PATCH_POWER,
];

pub fn ComputeDevicePresetMode(
    PresetMode: VL53L1_PresetModes,
    DistanceMode: VL53L1_DistanceModes,
    pDevicePresetMode: &mut VL53L1_DevicePresetModes,
) -> VL53L1_Error {
    let mut DistIdx: usize = 0;
    let LightModes = [
        VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE,
        VL53L1_DEVICEPRESETMODE_STANDARD_RANGING,
        VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE,
    ];

    let TimedModes = [
        VL53L1_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE,
        VL53L1_DEVICEPRESETMODE_TIMED_RANGING,
        VL53L1_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE,
    ];

    let LowPowerTimedModes = [
        VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE,
        VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE,
        VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE,
    ];

    *pDevicePresetMode = VL53L1_DEVICEPRESETMODE_STANDARD_RANGING;
    match DistanceMode {
        VL53L1_DISTANCEMODE_SHORT => DistIdx = 0,
        VL53L1_DISTANCEMODE_MEDIUM => DistIdx = 1,
        VL53L1_DISTANCEMODE_LONG => DistIdx = 2,
        _ => return VL53L1_ERROR_MODE_NOT_SUPPORTED,
    }

    match PresetMode {
        VL53L1_PRESETMODE_LITE_RANGING => *pDevicePresetMode = LightModes[DistIdx],
        VL53L1_PRESETMODE_AUTONOMOUS => *pDevicePresetMode = TimedModes[DistIdx],
        VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS => *pDevicePresetMode = LowPowerTimedModes[DistIdx],
        _ => return VL53L1_ERROR_MODE_NOT_SUPPORTED,
    }
    return VL53L1_ERROR_NONE;
}

pub fn ComputeRQL(
    active_results: u8,
    FilteredRangeStatus: u8,
    presults_data: &VL53L1_range_data_t,
) -> u8 {
    let SRL = 300_i16;
    let SRAS = 30_u16;
    let mut RAS = 0_u32;
    let mut SRQL = 0_u32;
    let GI = 7713587_u32; /* 117.7 * 65536 */
    let GGm = 3198157_u32; /* 48.8 * 65536 */
    let LRAP = 6554_u32; /* 0.1 * 65536 */
    let mut partial = 0_u32;
    if active_results == 0 {
        return 0;
    } else if FilteredRangeStatus == VL53L1_DEVICEERROR_PHASECONSISTENCY {
        return 50;
    } else {
        if presults_data.median_range_mm < SRL {
            RAS = SRAS as u32 * 65536;
        } else {
            RAS = LRAP * presults_data.median_range_mm as u32;
        }

        /* Fix1616 + (fix1616 * uint16_t / fix1616) * 65536 = fix1616 */
        if RAS != 0 {
            partial = GGm * presults_data.sigma_mm as u32;
            partial = partial + (RAS >> 1);
            partial = partial / RAS;
            partial = partial * 65536;
            if partial <= GI {
                SRQL = GI - partial;
            } else {
                SRQL = 50 * 65536;
            }
        } else {
            SRQL = 100 * 65536;
        }
        return 50.max(100.min((SRQL >> 16) as u8));
    }
}

pub fn ConvertStatusLite(FilteredRangeStatus: u8) -> u8 {
    match FilteredRangeStatus {
        VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY => VL53L1_RANGESTATUS_SYNCRONISATION_INT,
        VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK => {
            VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL
        }
        VL53L1_DEVICEERROR_RANGEPHASECHECK => VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL,
        VL53L1_DEVICEERROR_MSRCNOTARGET => VL53L1_RANGESTATUS_SIGNAL_FAIL,
        VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK => VL53L1_RANGESTATUS_SIGMA_FAIL,
        VL53L1_DEVICEERROR_PHASECONSISTENCY => VL53L1_RANGESTATUS_WRAP_TARGET_FAIL,
        VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD => VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL,
        VL53L1_DEVICEERROR_MINCLIP => VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED,
        VL53L1_DEVICEERROR_RANGECOMPLETE => VL53L1_RANGESTATUS_RANGE_VALID,
        _ => VL53L1_RANGESTATUS_NONE,
    }
}
