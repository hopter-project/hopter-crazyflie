use atomic_float::AtomicF32;
use core::sync::atomic::Ordering;

#[derive(Debug)]
pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    rate: f32,
    dt: f32,
    prev_error: AtomicF32,
    integral: AtomicF32,

    i_limit: Option<f32>, // integral limit
    o_limit: Option<f32>, // output limit
}

impl PID {
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        rate: f32,
        i_limit: Option<f32>,
        o_limit: Option<f32>,
    ) -> Self {
        assert!(rate > 0.0);
        Self {
            kp,
            ki,
            kd,
            rate,
            dt: 1.0 / rate,
            prev_error: AtomicF32::new(0.0),
            integral: AtomicF32::new(0.0),
            i_limit,
            o_limit,
        }
    }

    /*
     * error = target - measure
     */
    pub fn update(&self, error: f32) -> f32 {
        #[cfg(feature = "stabilizer_panic")]
        {
            use crate::flight_tasks::STABILIZER_PANIC_BEGIN_TS;
            use crate::flight_tasks::STABILIZER_PANIC_REQUIRED;
            use crate::flight_tasks::STABILIZER_PANIC_RESPOND;
            use crate::module::timestamp;
            use core::sync::atomic::Ordering;
            if STABILIZER_PANIC_REQUIRED.load(Ordering::Relaxed) {
                STABILIZER_PANIC_REQUIRED.store(false, Ordering::Relaxed);
                STABILIZER_PANIC_RESPOND.store(true, Ordering::Relaxed);
                STABILIZER_PANIC_BEGIN_TS
                    .store(timestamp::usec_timestamp() as usize, Ordering::Relaxed);
                panic!()
            }
        }

        #[cfg(feature = "stabilizer_overflow")]
        {
            use crate::flight_tasks::STABILIZER_PANIC_REQUIRED;
            if STABILIZER_PANIC_REQUIRED.load(Ordering::Relaxed) {
                recursive(4);
            }
        }

        let mut output: f32 = 0.0;
        output += self.kp * error;

        let derivative = (error - self.prev_error.load(Ordering::Relaxed)) * self.rate;
        output += self.kd * derivative;

        let mut integral = self.integral.load(Ordering::Relaxed);
        integral += error * self.dt;

        if let Some(i_limit) = self.i_limit {
            integral = integral.clamp(-i_limit, i_limit);
        }

        output += self.ki * integral;
        self.integral.store(integral, Ordering::Relaxed);

        self.prev_error.store(error, Ordering::Relaxed);
        if let Some(o_limit) = self.o_limit {
            output = output.clamp(-o_limit, o_limit);
        }
        output
    }

    pub fn reset(&self) {
        self.prev_error.store(0.0, Ordering::Relaxed);
        self.integral.store(0.0, Ordering::Relaxed);
    }
}

#[cfg(feature = "stabilizer_overflow")]
#[inline(never)]
fn recursive(remain: u32) {
    use crate::flight_tasks::STABILIZER_PANIC_BEGIN_TS;
    use crate::flight_tasks::STABILIZER_PANIC_REQUIRED;
    use crate::flight_tasks::STABILIZER_PANIC_RESPOND;
    use crate::module::timestamp;
    use core::sync::atomic::Ordering;

    if remain == 0 {
        STABILIZER_PANIC_REQUIRED.store(false, Ordering::Relaxed);
        STABILIZER_PANIC_RESPOND.store(true, Ordering::Relaxed);
        STABILIZER_PANIC_BEGIN_TS.store(timestamp::usec_timestamp() as usize, Ordering::Relaxed);
        overflow();
        return;
    }
    let _has_drop = HasDrop;
    recursive(remain - 1);
}

#[cfg(feature = "stabilizer_overflow")]
struct HasDrop;

#[cfg(feature = "stabilizer_overflow")]
impl Drop for HasDrop {
    fn drop(&mut self) {
        cortex_m::asm::nop();
    }
}

#[cfg(feature = "stabilizer_overflow")]
#[inline(never)]
fn overflow() {
    let _padding = StackFramePadding::new();
}

#[cfg(feature = "stabilizer_overflow")]
struct StackFramePadding {
    _padding: [u8; 32768],
}

#[cfg(feature = "stabilizer_overflow")]
impl StackFramePadding {
    fn new() -> Self {
        let mut padding = core::mem::MaybeUninit::<[u8; 32768]>::uninit();
        let mut ptr = unsafe { (*padding.as_mut_ptr()).as_mut_ptr() };
        for _ in 0..4096 {
            unsafe {
                ptr.write_volatile(0);
                ptr = ptr.offset(1);
            }
        }
        let padding = unsafe { padding.assume_init() };
        Self { _padding: padding }
    }
}
