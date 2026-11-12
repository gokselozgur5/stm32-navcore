//! Magnetometer hard-iron and diagonal soft-iron calibration.

/// Streaming per-axis minimum and maximum tracker.
#[derive(Clone, Copy, Debug)]
pub struct MagMinMax {
    min: [f32; 3],
    max: [f32; 3],
    count: u32,
}

impl MagMinMax {
    /// Creates an empty tracker.
    pub const fn new() -> Self {
        Self {
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            count: 0,
        }
    }
}

impl Default for MagMinMax {
    fn default() -> Self {
        Self::new()
    }
}

impl MagMinMax {
    /// Folds one raw sample into the running extremes.
    #[allow(clippy::needless_range_loop)]
    pub fn update(&mut self, s: [f32; 3]) {
        for i in 0..3 {
            self.min[i] = self.min[i].min(s[i]);
            self.max[i] = self.max[i].max(s[i]);
        }
        self.count = self.count.saturating_add(1);
    }

    /// Number of samples seen so far.
    pub fn count(&self) -> u32 {
        self.count
    }
}
