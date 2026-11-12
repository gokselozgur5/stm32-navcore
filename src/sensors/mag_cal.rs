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

/// Hard-iron offset and diagonal soft-iron scale applied to raw samples.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagCal {
    pub offset: [f32; 3],
    pub scale: [f32; 3],
}

impl MagCal {
    /// Identity calibration: no offset, unit scale.
    pub const IDENTITY: Self = Self {
        offset: [0.0; 3],
        scale: [1.0; 3],
    };

    /// Builds a calibration from an accumulator.
    pub fn from_min_max(mm: &MagMinMax) -> Self {
        Self {
            offset: mm.offset(),
            scale: mm.scale(),
        }
    }
}

impl MagCal {
    /// Returns `(raw - offset) * scale` per axis.
    pub fn apply(&self, raw: [f32; 3]) -> [f32; 3] {
        [
            (raw[0] - self.offset[0]) * self.scale[0],
            (raw[1] - self.offset[1]) * self.scale[1],
            (raw[2] - self.offset[2]) * self.scale[2],
        ]
    }
}

/// Streaming least-squares sphere fit using normal-equation sums.
///
/// Model: `x^2 + y^2 + z^2 = 2ax + 2by + 2cz + d`.
#[derive(Clone, Copy, Debug)]
pub struct SphereFit {
    ata: [[f64; 4]; 4],
    aty: [f64; 4],
    count: u32,
}

impl SphereFit {
    /// Creates an empty fit.
    pub const fn new() -> Self {
        Self {
            ata: [[0.0; 4]; 4],
            aty: [0.0; 4],
            count: 0,
        }
    }
}

impl Default for SphereFit {
    fn default() -> Self {
        Self::new()
    }
}

impl SphereFit {
    /// Adds one raw sample to the normal equations.
    #[allow(clippy::needless_range_loop)]
    pub fn add(&mut self, s: [f32; 3]) {
        let (x, y, z) = (s[0] as f64, s[1] as f64, s[2] as f64);
        let row = [x, y, z, 1.0];
        let t = x * x + y * y + z * z;
        for i in 0..4 {
            for j in 0..4 {
                self.ata[i][j] += row[i] * row[j];
            }
            self.aty[i] += row[i] * t;
        }
        self.count = self.count.saturating_add(1);
    }
}

/// Gauss-Jordan solve of a 4x4 augmented system; `None` if singular.
#[allow(clippy::needless_range_loop)]
fn solve4(mut a: [[f64; 5]; 4]) -> Option<[f64; 4]> {
    for i in 0..4 {
        let mut p = i;
        for r in i + 1..4 {
            if libm::fabs(a[r][i]) > libm::fabs(a[p][i]) {
                p = r;
            }
        }
        if libm::fabs(a[p][i]) < 1e-9 {
            return None;
        }
        a.swap(i, p);
        let pivot = a[i];
        for (r, row) in a.iter_mut().enumerate() {
            if r != i {
                let f = row[i] / pivot[i];
                for (c, v) in row.iter_mut().enumerate() {
                    *v -= f * pivot[c];
                }
            }
        }
    }
    Some([0, 1, 2, 3].map(|i| a[i][4] / a[i][i]))
}

impl SphereFit {
    /// Solves for `(centre, radius)`; `None` with fewer than four samples
    /// or a singular system.
    #[allow(clippy::needless_range_loop)]
    pub fn solve(&self) -> Option<([f32; 3], f32)> {
        if self.count < 4 {
            return None;
        }
        let mut aug = [[0.0; 5]; 4];
        for i in 0..4 {
            aug[i][..4].copy_from_slice(&self.ata[i]);
            aug[i][4] = self.aty[i];
        }
        let p = solve4(aug)?;
        let (a, b, c) = (0.5 * p[0], 0.5 * p[1], 0.5 * p[2]);
        let r2 = p[3] + a * a + b * b + c * c;
        if r2 <= 0.0 {
            return None;
        }
        Some(([a as f32, b as f32, c as f32], libm::sqrt(r2) as f32))
    }
}

/// Tracks which of the eight octants around a centre have been visited.
#[derive(Clone, Copy, Debug, Default)]
pub struct MagCoverage {
    mask: u8,
}

impl MagCoverage {
    /// Creates an empty tracker.
    pub const fn new() -> Self {
        Self { mask: 0 }
    }

    /// Marks the octant of `s` relative to `centre`.
    #[allow(clippy::needless_range_loop)]
    pub fn add(&mut self, s: [f32; 3], centre: [f32; 3]) {
        let mut bit = 0;
        for i in 0..3 {
            if s[i] >= centre[i] {
                bit |= 1 << i;
            }
        }
        self.mask |= 1 << bit;
    }
}

impl MagCoverage {
    /// Number of octants visited, 0 to 8.
    pub fn octants(&self) -> u32 {
        self.mask.count_ones()
    }

    /// Fraction of octants visited, 0.0 to 1.0.
    pub fn fraction(&self) -> f32 {
        self.octants() as f32 / 8.0
    }

    /// True when every octant has been visited.
    pub fn is_full(&self) -> bool {
        self.mask == 0xFF
    }
}

/// Accumulates deviation of corrected magnitudes from a reference radius.
#[derive(Clone, Copy, Debug, Default)]
pub struct MagResidual {
    sum_sq: f32,
    count: u32,
}

impl MagResidual {
    /// Creates an empty accumulator.
    pub const fn new() -> Self {
        Self {
            sum_sq: 0.0,
            count: 0,
        }
    }

    /// Adds one corrected sample measured against `radius`.
    pub fn add(&mut self, corrected: [f32; 3], radius: f32) {
        let m = libm::sqrtf(corrected.iter().map(|v| v * v).sum::<f32>());
        let d = m - radius;
        self.sum_sq += d * d;
        self.count = self.count.saturating_add(1);
    }
}

impl MagResidual {
    /// Root-mean-square deviation, or 0.0 with no samples.
    pub fn rms(&self) -> f32 {
        if self.count == 0 {
            return 0.0;
        }
        libm::sqrtf(self.sum_sq / self.count as f32)
    }

    /// RMS deviation relative to `radius`, or infinity for a non-positive radius.
    pub fn relative(&self, radius: f32) -> f32 {
        if radius > 0.0 {
            self.rms() / radius
        } else {
            f32::INFINITY
        }
    }
}

/// A calibration is valid with full octant coverage and a small residual.
pub fn is_valid(cov: &MagCoverage, res: &MagResidual, radius: f32, max_rel: f32) -> bool {
    cov.is_full() && res.relative(radius) <= max_rel
}
