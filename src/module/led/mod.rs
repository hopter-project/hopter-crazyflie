use core::cell::UnsafeCell;
use stm32f4xx_hal::gpio::{Output, Pin};

#[allow(unused)]
pub struct LEDs {
    internal: UnsafeCell<Internal>,
}

struct Internal {
    left_red_led: Pin<'C', 0, Output>,
    right_red_led: Pin<'C', 3, Output>,
    left_green_led: Pin<'C', 1, Output>,
    right_green_led: Pin<'C', 2, Output>,
    left_blue_led: Pin<'D', 2, Output>,
}

#[allow(unused)]
impl LEDs {
    pub fn new(
        mut left_red_led: Pin<'C', 0, Output>,
        mut right_red_led: Pin<'C', 3, Output>,
        mut left_green_led: Pin<'C', 1, Output>,
        mut right_green_led: Pin<'C', 2, Output>,
        mut left_blue_led: Pin<'D', 2, Output>,
    ) -> Self {
        left_red_led.set_high();
        right_red_led.set_high();
        left_green_led.set_high();
        right_green_led.set_high();
        left_blue_led.set_low();

        let internal = UnsafeCell::new(Internal {
            left_red_led,
            right_red_led,
            left_green_led,
            right_green_led,
            left_blue_led,
        });

        Self { internal }
    }

    pub fn left_red_on(&self) {
        let internal = Self::get_internal(self);
        internal.left_red_led.set_low()
    }

    pub fn right_red_on(&self) {
        let internal = Self::get_internal(self);
        internal.right_red_led.set_low()
    }

    pub fn left_red_off(&self) {
        let internal = Self::get_internal(self);
        internal.left_red_led.set_high()
    }

    pub fn right_red_off(&self) {
        let internal = Self::get_internal(self);
        internal.right_red_led.set_high()
    }

    pub fn left_red_toggle(&self) {
        let internal = Self::get_internal(self);
        internal.left_red_led.toggle()
    }

    pub fn right_red_toggle(&self) {
        let internal = Self::get_internal(self);
        internal.right_red_led.toggle()
    }

    pub fn left_green_on(&self) {
        let internal = Self::get_internal(self);
        internal.left_green_led.set_low()
    }

    pub fn right_green_on(&self) {
        let internal = Self::get_internal(self);
        internal.right_green_led.set_low()
    }

    pub fn left_green_off(&self) {
        let internal = Self::get_internal(self);
        internal.left_green_led.set_high()
    }

    pub fn right_green_off(&self) {
        let internal = Self::get_internal(self);
        internal.right_green_led.set_high()
    }

    pub fn left_green_toggle(&self) {
        let internal = Self::get_internal(self);
        internal.left_green_led.toggle()
    }

    pub fn right_green_toggle(&self) {
        let internal = Self::get_internal(self);
        internal.right_green_led.toggle()
    }

    pub fn left_blue_on(&self) {
        let internal = Self::get_internal(self);
        internal.left_blue_led.set_high()
    }

    pub fn left_blue_off(&self) {
        let internal = Self::get_internal(self);
        internal.left_blue_led.set_low()
    }

    pub fn left_blue_toggle(&self) {
        let internal = Self::get_internal(self);
        internal.left_blue_led.toggle()
    }

    fn get_internal(&self) -> &mut Internal {
        unsafe { &mut *self.internal.get() }
    }
}

unsafe impl Sync for LEDs {}
