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

    pub fn update(&mut self, sample: f32) -> f32 {
        self.buffer[self.head] = sample;
        self.head = (self.head + 1) % N;
        if self.count < N {
            self.count += 1;
        }
        self.current_median()
    }

    pub fn current_median(&self) -> f32 {
        if self.count == 0 { return 0.0; }
        let mut sorted = [0.0f32; N];
        let len = self.count;
        sorted[..len].copy_from_slice(&self.buffer[..len]);
        for i in 1..len {
            let mut j = i;
            while j > 0 && (sorted[j - 1] > sorted[j] || sorted[j - 1].is_nan()) {
                sorted.swap(j - 1, j);
                j -= 1;
            }
        }
        sorted[len / 2]
    }
}
