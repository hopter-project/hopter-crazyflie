use core::f32::consts::PI;
use libm::tanf;

#[derive(Default, Debug, Copy, Clone)]
pub struct Lpf2d {
    a1: f32,
    a2: f32,
    b0: f32,
    b1: f32,
    b2: f32,
    delay_element_1: f32,
    delay_element_2: f32,
}

impl Lpf2d {
    pub fn new(sample_freq: f32, cutoff_freq: f32) -> Self {
        assert!(sample_freq > 0.0 && cutoff_freq > 0.0);
        let fr = sample_freq / cutoff_freq;
        let ohm = tanf(PI / fr);
        let c = 1.0 + 2.0 * libm::cosf(PI / 4.0) * ohm + ohm * ohm;
        let mut lpf = Self::default();
        lpf.b0 = ohm * ohm / c;
        lpf.b1 = 2.0 * lpf.b0;
        lpf.b2 = lpf.b0;
        lpf.a1 = 2.0 * (ohm * ohm - 1.0) / c;
        lpf.a2 = (1.0 - 2.0 * libm::cosf(PI / 4.0) * ohm + ohm * ohm) / c;
        lpf
    }

    pub fn update(&mut self, sample: f32) -> f32 {
        let mut delay_element_0 =
            sample - self.delay_element_1 * self.a1 - self.delay_element_2 * self.a2;
        if delay_element_0.is_nan() {
            // don't allow bad values to propagate via the filter
            delay_element_0 = sample;
        }
        let output = delay_element_0 * self.b0
            + self.delay_element_1 * self.b1
            + self.delay_element_2 * self.b2;
        self.delay_element_2 = self.delay_element_1;
        self.delay_element_1 = delay_element_0;
        output
    }

    #[allow(dead_code)]
    pub fn reset(&mut self, sample: f32) -> f32 {
        let d_val = sample / (self.b0 + self.b1 + self.b2);
        self.delay_element_1 = d_val;
        self.delay_element_2 = d_val;
        return self.update(sample);
    }
}
