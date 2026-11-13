//! Fixed-window sliding median filter.

/// Fixed-capacity sliding window median filter with const odd window size N.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MedianFilter<const N: usize> {
    buffer: [f32; N],
    count: usize,
    head: usize,
}

impl<const N: usize> MedianFilter<N> {
    /// Creates a new median filter with window size N.
    pub const fn new() -> Self {
        assert!(N > 0 && N % 2 == 1, "Window size must be odd");
        Self { buffer: [0.0; N], count: 0, head: 0 }
    }

    /// Resets internal buffer state.
    pub fn reset(&mut self) {
        self.count = 0;
        self.head = 0;
    }
}
