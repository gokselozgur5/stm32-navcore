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
