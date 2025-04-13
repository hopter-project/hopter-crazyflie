use crate::module::{acquire_spi1_cs, acquire_spi1_handle};

pub fn spi1_enable_cs() {
    // reset GPIOB pin 4
    let mut pin = acquire_spi1_cs();
    pin.set_low();
}

pub fn spi1_disable_cs() {
    // set GPIOB pin 4
    let mut pin = acquire_spi1_cs();
    pin.set_high();
}

pub fn spi1_read(reg_addr: u8, reg_data: &mut [u8], _intf: u32) -> i8 {
    let mut handle = acquire_spi1_handle();

    #[cfg(feature = "flow_panic")]
    {
        use crate::flight_tasks::FLOW_PANIC_REQUIRED;
        use core::sync::atomic::Ordering;
        if FLOW_PANIC_REQUIRED.load(Ordering::Relaxed) {
            panic!()
        }
    }

    let mut rslt = 0_i8;
    spi1_enable_cs();
    handle.write(&[reg_addr & !0x80]).unwrap();
    if let Err(_) = handle.read(reg_data) {
        rslt = -1;
    }
    spi1_disable_cs();
    return rslt;
}

pub fn spi1_write(reg_addr: u8, reg_data: &[u8], _intf: u32) -> i8 {
    let mut handle = acquire_spi1_handle();
    let mut rslt = 0_i8;
    spi1_enable_cs();
    let buffer = [&[reg_addr | 0x80], reg_data].concat();
    if let Err(_) = handle.write(&buffer) {
        rslt = -1;
    }
    spi1_disable_cs();
    return rslt;
}
