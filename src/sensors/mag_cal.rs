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
