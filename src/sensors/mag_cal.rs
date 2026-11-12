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

impl MagMinMax {
    /// Hard-iron offset: the per-axis midpoint of the observed range.
    pub fn offset(&self) -> [f32; 3] {
        let mut o = [0.0; 3];
        for (i, v) in o.iter_mut().enumerate() {
            *v = if self.count == 0 {
                0.0
            } else {
                0.5 * (self.min[i] + self.max[i])
            };
        }
        o
    }
}

impl MagMinMax {
    /// Diagonal soft-iron gains that equalise half-ranges to their mean.
    ///
    /// A degenerate axis (no range) gets a gain of 1.0.
    pub fn scale(&self) -> [f32; 3] {
        let mut half = [0.0f32; 3];
        for (i, h) in half.iter_mut().enumerate() {
            if self.count > 0 {
                *h = 0.5 * (self.max[i] - self.min[i]);
            }
        }
        let avg = (half[0] + half[1] + half[2]) / 3.0;
        half.map(|h| if h > 1e-6 { avg / h } else { 1.0 })
    }
}
