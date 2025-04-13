use crate::driver::serial::spi;

#[allow(unused)]
#[derive(Default)]
pub struct PMW3901MotionData {
    pub motion: u8,
    pub delta_x: i16,
    pub delta_y: i16,
    pub squal: u8,
    pub raw_dat_sum: u8,
    pub max_raw_data: u8,
    pub min_raw_data: u8,
    pub shutter: u16,
}

pub struct PMW3901Dev {}

impl PMW3901Dev {
    pub fn init(&mut self) -> i8 {
        spi::spi1_disable_cs();
        hopter::time::sleep_ms(2).unwrap();
        spi::spi1_enable_cs();
        hopter::time::sleep_ms(2).unwrap();
        spi::spi1_disable_cs();
        hopter::time::sleep_ms(2).unwrap();
        let mut chip_id = [0u8; 1];
        let mut inv_chip_id = [0u8; 1];
        let mut data = [0u8; 1];
        super::serial::spi::spi1_read(0x00, &mut chip_id, 0);
        super::serial::spi::spi1_read(0x5F, &mut inv_chip_id, 0);

        if chip_id[0] == 0x49 && inv_chip_id[0] == 0xB6 {
            // power on reset
            super::serial::spi::spi1_write(0x3A, &[0x5A], 0);
            hopter::time::sleep_ms(5).unwrap();

            // read the motion register to clear it
            super::serial::spi::spi1_read(0x02, &mut data, 0);
            super::serial::spi::spi1_read(0x03, &mut data, 0);
            super::serial::spi::spi1_read(0x04, &mut data, 0);
            super::serial::spi::spi1_read(0x05, &mut data, 0);
            super::serial::spi::spi1_read(0x06, &mut data, 0);
            hopter::time::sleep_ms(1).unwrap();

            self.init_registers();
            dbg_println!("PMW3901 init [OK]");
            return 0;
        } else {
            dbg_println!("PMW3901 init [FAIL]");
            return -1;
        }
    }

    fn init_registers(&mut self) {
        let mut data = [0u8; 1];
        super::serial::spi::spi1_write(0x7F, &[0x00], 0);
        super::serial::spi::spi1_write(0x55, &[0x01], 0);
        super::serial::spi::spi1_write(0x50, &[0x07], 0);
        super::serial::spi::spi1_write(0x7F, &[0x0E], 0);
        super::serial::spi::spi1_write(0x43, &[0x10], 0);

        super::serial::spi::spi1_read(0x67, &mut data, 0);
        if data[0] & 0b10000000 != 0 {
            super::serial::spi::spi1_write(0x48, &[0x04], 0);
        } else {
            super::serial::spi::spi1_write(0x48, &[0x02], 0);
        }

        super::serial::spi::spi1_write(0x7F, &[0x00], 0);
        super::serial::spi::spi1_write(0x51, &[0x7B], 0);
        super::serial::spi::spi1_write(0x50, &[0x00], 0);
        super::serial::spi::spi1_write(0x55, &[0x00], 0);
        super::serial::spi::spi1_write(0x7F, &[0x0E], 0);

        super::serial::spi::spi1_read(0x73, &mut data, 0);
        if data[0] == 0x00 {
            let mut c1 = [0u8; 1];
            let mut c2 = [0u8; 1];
            super::serial::spi::spi1_read(0x70, &mut c1, 0);
            super::serial::spi::spi1_read(0x71, &mut c2, 0);
            if c1[0] < 28 {
                c1[0] += 14;
            }
            if c1[0] > 28 {
                c1[0] += 11;
            }
            c1[0] = 0.max(c1[0].min(0x3F));
            c2[0] = (((c2[0] as u32) * 45) / 100) as u8;
            super::serial::spi::spi1_write(0x7F, &[0x00], 0);
            super::serial::spi::spi1_write(0x61, &[0xAD], 0);
            super::serial::spi::spi1_write(0x51, &[0x70], 0);
            super::serial::spi::spi1_write(0x7F, &[0x0E], 0);
            super::serial::spi::spi1_write(0x70, &c1, 0);
            super::serial::spi::spi1_write(0x71, &c2, 0);
        }

        super::serial::spi::spi1_write(0x7F, &[0x00], 0);
        super::serial::spi::spi1_write(0x61, &[0xAD], 0);
        super::serial::spi::spi1_write(0x7F, &[0x03], 0);
        super::serial::spi::spi1_write(0x40, &[0x00], 0);
        super::serial::spi::spi1_write(0x7F, &[0x05], 0);
        super::serial::spi::spi1_write(0x41, &[0xB3], 0);
        super::serial::spi::spi1_write(0x43, &[0xF1], 0);
        super::serial::spi::spi1_write(0x45, &[0x14], 0);
        super::serial::spi::spi1_write(0x5B, &[0x32], 0);
        super::serial::spi::spi1_write(0x5F, &[0x34], 0);
        super::serial::spi::spi1_write(0x7B, &[0x08], 0);
        super::serial::spi::spi1_write(0x7F, &[0x06], 0);
        super::serial::spi::spi1_write(0x44, &[0x1B], 0);
        super::serial::spi::spi1_write(0x40, &[0xBF], 0);
        super::serial::spi::spi1_write(0x4E, &[0x3F], 0);
        super::serial::spi::spi1_write(0x7F, &[0x08], 0);
        super::serial::spi::spi1_write(0x65, &[0x20], 0);
        super::serial::spi::spi1_write(0x6A, &[0x18], 0);
        super::serial::spi::spi1_write(0x7F, &[0x09], 0);
        super::serial::spi::spi1_write(0x4F, &[0xAF], 0);
        super::serial::spi::spi1_write(0x5F, &[0x40], 0);
        super::serial::spi::spi1_write(0x48, &[0x80], 0);
        super::serial::spi::spi1_write(0x49, &[0x80], 0);
        super::serial::spi::spi1_write(0x57, &[0x77], 0);
        super::serial::spi::spi1_write(0x60, &[0x78], 0);
        super::serial::spi::spi1_write(0x61, &[0x78], 0);
        super::serial::spi::spi1_write(0x62, &[0x08], 0);
        super::serial::spi::spi1_write(0x63, &[0x50], 0);
        super::serial::spi::spi1_write(0x7F, &[0x0A], 0);
        super::serial::spi::spi1_write(0x45, &[0x60], 0);
        super::serial::spi::spi1_write(0x7F, &[0x00], 0);
        super::serial::spi::spi1_write(0x4D, &[0x11], 0);
        super::serial::spi::spi1_write(0x55, &[0x80], 0);
        super::serial::spi::spi1_write(0x74, &[0x1F], 0);
        super::serial::spi::spi1_write(0x75, &[0x1F], 0);
        super::serial::spi::spi1_write(0x4A, &[0x78], 0);
        super::serial::spi::spi1_write(0x4B, &[0x78], 0);
        super::serial::spi::spi1_write(0x44, &[0x08], 0);
        super::serial::spi::spi1_write(0x45, &[0x50], 0);
        super::serial::spi::spi1_write(0x64, &[0xFF], 0);
        super::serial::spi::spi1_write(0x65, &[0x1F], 0);
        super::serial::spi::spi1_write(0x7F, &[0x14], 0);
        super::serial::spi::spi1_write(0x65, &[0x67], 0);
        super::serial::spi::spi1_write(0x66, &[0x08], 0);
        super::serial::spi::spi1_write(0x63, &[0x70], 0);
        super::serial::spi::spi1_write(0x7F, &[0x15], 0);
        super::serial::spi::spi1_write(0x48, &[0x48], 0);
        super::serial::spi::spi1_write(0x7F, &[0x07], 0);
        super::serial::spi::spi1_write(0x41, &[0x0D], 0);
        super::serial::spi::spi1_write(0x43, &[0x14], 0);
        super::serial::spi::spi1_write(0x4B, &[0x0E], 0);
        super::serial::spi::spi1_write(0x45, &[0x0F], 0);
        super::serial::spi::spi1_write(0x44, &[0x42], 0);
        super::serial::spi::spi1_write(0x4C, &[0x80], 0);
        super::serial::spi::spi1_write(0x7F, &[0x10], 0);
        super::serial::spi::spi1_write(0x5B, &[0x02], 0);
        super::serial::spi::spi1_write(0x7F, &[0x07], 0);
        super::serial::spi::spi1_write(0x40, &[0x41], 0);
        super::serial::spi::spi1_write(0x70, &[0x00], 0);

        hopter::time::sleep_ms(10).unwrap();

        super::serial::spi::spi1_write(0x32, &[0x44], 0);
        super::serial::spi::spi1_write(0x7F, &[0x07], 0);
        super::serial::spi::spi1_write(0x40, &[0x40], 0);
        super::serial::spi::spi1_write(0x7F, &[0x06], 0);
        super::serial::spi::spi1_write(0x62, &[0xF0], 0);
        super::serial::spi::spi1_write(0x63, &[0x00], 0);
        super::serial::spi::spi1_write(0x7F, &[0x0D], 0);
        super::serial::spi::spi1_write(0x48, &[0xC0], 0);
        super::serial::spi::spi1_write(0x6F, &[0xD5], 0);
        super::serial::spi::spi1_write(0x7F, &[0x00], 0);
        super::serial::spi::spi1_write(0x5B, &[0xA0], 0);
        super::serial::spi::spi1_write(0x4E, &[0xA8], 0);
        super::serial::spi::spi1_write(0x5A, &[0x50], 0);
        super::serial::spi::spi1_write(0x40, &[0x80], 0);
    }

    pub fn pmw3901_get_motion_data(&mut self, motion_data: &mut PMW3901MotionData) -> i8 {
        let mut motion = [0_u8; 1];
        let mut status = super::serial::spi::spi1_read(0x02, &mut motion, 0);
        motion_data.motion = motion[0];
        if status == 0 {
            let mut delta_x_l = [0_u8; 1];
            let mut delta_x_h = [0_u8; 1];
            let mut delta_y_l = [0_u8; 1];
            let mut delta_y_h = [0_u8; 1];
            let mut squal = [0_u8; 1];
            let mut shutter_l = [0_u8; 1];
            let mut shutter_h = [0_u8; 1];
            status = super::serial::spi::spi1_read(0x04, &mut delta_x_h, 0)
                | super::serial::spi::spi1_read(0x03, &mut delta_x_l, 0)
                | super::serial::spi::spi1_read(0x06, &mut delta_y_h, 0)
                | super::serial::spi::spi1_read(0x05, &mut delta_y_l, 0)
                | super::serial::spi::spi1_read(0x07, &mut squal, 0)
                | super::serial::spi::spi1_read(0x0C, &mut shutter_h, 0)
                | super::serial::spi::spi1_read(0x0B, &mut shutter_l, 0);
            motion_data.delta_x = ((delta_x_h[0] as i16) << 8) | delta_x_l[0] as i16;
            motion_data.delta_y = ((delta_y_h[0] as i16) << 8) | delta_y_l[0] as i16;
            motion_data.squal = squal[0];
            motion_data.shutter = ((shutter_h[0] as u16) << 8) | shutter_l[0] as u16;
        }
        return status;
    }
}
