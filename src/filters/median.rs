//! Fixed-window median filter for noisy sensor rejection (NC-020).

#[derive(Debug, Clone)]
pub struct MedianFilter<const N: usize> {
    buffer: [f32; N],
    count: usize,
    head: usize,
}

impl<const N: usize> MedianFilter<N> {
    pub const fn new() -> Self {
        Self { buffer: [0.0; N], count: 0, head: 0 }
    }

    pub fn reset(&mut self) {
        self.count = 0;
        self.head = 0;
    }

    pub fn is_full(&self) -> bool {
        self.count == N
    }
}
