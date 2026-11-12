//! Streaming overlapping Allan variance estimator for IMU noise.
//!
//! Rate samples are integrated into a phase record kept in a fixed ring
//! buffer. Averaging factors are octave spaced (m = 1, 2, 4, ...).

/// Overlapping Allan variance over `L` octave levels, with a phase ring of `N`.
pub struct AllanEstimator<const N: usize, const L: usize> {
    tau0: f64,
    phase: [f64; N],
    acc: f64,
    samples: u64,
    sum: [f64; L],
    count: [u64; L],
}

impl<const N: usize, const L: usize> AllanEstimator<N, L> {
    /// Create an estimator for samples spaced `tau0` seconds apart.
    ///
    /// Panics if the ring cannot hold `2 * 2^(L-1) + 1` phase values.
    pub fn new(tau0: f64) -> Self {
        assert!(L > 0 && L < 32 && tau0 > 0.0);
        assert!(N > (2usize << (L - 1)));
        Self {
            tau0,
            phase: [0.0; N],
            acc: 0.0,
            samples: 0,
            sum: [0.0; L],
            count: [0; L],
        }
    }
}

impl<const N: usize, const L: usize> AllanEstimator<N, L> {
    /// Feed one rate sample, updating every level that has enough history.
    pub fn push(&mut self, rate: f64) {
        self.acc += rate * self.tau0;
        self.samples += 1;
        let k = self.samples as usize;
        self.phase[k % N] = self.acc;
        for i in 0..L {
            let m = 1usize << i;
            if k >= 2 * m {
                let d = self.acc - 2.0 * self.phase[(k - m) % N] + self.phase[(k - 2 * m) % N];
                self.sum[i] += d * d;
                self.count[i] += 1;
            }
        }
    }
}

impl<const N: usize, const L: usize> AllanEstimator<N, L> {
    /// Cluster time in seconds for `level`.
    pub fn tau(&self, level: usize) -> f64 {
        self.tau0 * (1u64 << level) as f64
    }

    /// Number of second differences accumulated at `level`.
    pub fn count(&self, level: usize) -> u64 {
        self.count[level]
    }

    /// Overlapping Allan variance at `level`, or `None` without data.
    pub fn avar(&self, level: usize) -> Option<f64> {
        let n = self.count[level];
        let tau = self.tau(level);
        (n > 0).then(|| self.sum[level] / (2.0 * tau * tau * n as f64))
    }

    /// Allan deviation at `level`, or `None` without data.
    pub fn adev(&self, level: usize) -> Option<f64> {
        self.avar(level).map(libm::sqrt)
    }
}

impl<const N: usize, const L: usize> AllanEstimator<N, L> {
    /// Mean of `ln(adev) - slope * ln(tau)` over `lo..=hi`, if all levels have data.
    fn mean_intercept(&self, lo: usize, hi: usize, slope: f64) -> Option<f64> {
        if lo > hi || hi >= L {
            return None;
        }
        let mut total = 0.0;
        for level in lo..=hi {
            let adev = self.adev(level).filter(|a| *a > 0.0)?;
            total += libm::log(adev) - slope * libm::log(self.tau(level));
        }
        Some(total / (hi - lo + 1) as f64)
    }
}
