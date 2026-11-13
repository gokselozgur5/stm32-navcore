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

impl<const N: usize> MedianFilter<N> {
    /// Pre-fills the filter window with a constant value.
    pub fn init_with(&mut self, val: f32) {
        self.buffer = [val; N];
        self.count = N;
        self.head = 0;
    }

    /// Pushes one sample and returns the sliding window median.
    pub fn update(&mut self, sample: f32) -> f32 {
        if self.count == 0 { self.init_with(sample); return sample; }
        self.buffer[self.head] = sample;
        self.head = (self.head + 1) % N;
        let mut sorted = self.buffer;
        for i in 1..N {
            let key = sorted[i];
            let mut j = i;
            while j > 0 && sorted[j - 1] > key {
                sorted[j] = sorted[j - 1];
                j -= 1;
            }
            sorted[j] = key;
        }
        sorted[N / 2]
    }
}
