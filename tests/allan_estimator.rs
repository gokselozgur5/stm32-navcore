use stm32_navcore::sensors::allan::AllanEstimator;

pub struct Rng(u64);

impl Rng {
    pub fn next_u(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    pub fn gauss(&mut self) -> f64 {
        let (a, b) = (self.next_u(), self.next_u());
        libm::sqrt(-2.0 * libm::log(a)) * libm::cos(2.0 * core::f64::consts::PI * b)
    }
}

