//! Fault Detection, Isolation and Recovery (FDIR) for dual IMU sensors.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuId {
    Primary,
    Secondary,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
}

#[derive(Debug, Clone)]
pub struct DualImuManager {
    pub active_imu: ImuId,
    pub chi_square_threshold: f32,
    pub consecutive_faults: u32,
    pub max_consecutive_faults: u32,
    pub primary_healthy: bool,
    pub secondary_healthy: bool,
}

impl DualImuManager {
    pub fn new(chi_square_threshold: f32, max_consecutive_faults: u32) -> Self {
        Self {
            active_imu: ImuId::Primary,
            chi_square_threshold,
            consecutive_faults: 0,
            max_consecutive_faults,
            primary_healthy: true,
            secondary_healthy: true,
        }
    }

    pub fn default_config() -> Self {
        Self::new(12.59, 3)
    }
}

impl DualImuManager {
    pub fn compute_chi_square(diff: [f32; 3], var: [f32; 3]) -> f32 {
        let mut sum = 0.0f32;
        for i in 0..3 {
            let v = if var[i] > 1e-6 { var[i] } else { 1e-6 };
            sum += (diff[i] * diff[i]) / v;
        }
        sum
    }
}

impl DualImuManager {
    pub fn evaluate_sensors(&mut self, p: &ImuReading, s: &ImuReading, gyro_var: [f32; 3]) -> bool {
        let d_gyro = [
            p.gyro_rps[0] - s.gyro_rps[0],
            p.gyro_rps[1] - s.gyro_rps[1],
            p.gyro_rps[2] - s.gyro_rps[2],
        ];
        let chi2 = Self::compute_chi_square(d_gyro, gyro_var);
        chi2 > self.chi_square_threshold
    }
}
