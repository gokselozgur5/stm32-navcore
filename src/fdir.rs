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
    pub clean_epochs: u32,
    pub recovery_epochs: u32,
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
            clean_epochs: 0,
            recovery_epochs: 0,
        }
    }

    /// Enables primary recovery after `epochs` consecutive clean epochs (0 disables).
    pub fn with_recovery(mut self, epochs: u32) -> Self {
        self.recovery_epochs = epochs;
        self
    }

    pub fn default_config() -> Self {
        Self::new(12.59, 3)
    }
}

/// Returns true when every gyro and accelerometer component is finite.
fn reading_is_finite(r: &ImuReading) -> bool {
    let all = r.gyro_rps.iter().chain(r.accel_mps2.iter());
    all.into_iter().all(|v| v.is_finite())
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
        let all = p.gyro_rps.iter().chain(s.gyro_rps.iter());
        if all.into_iter().any(|g| !g.is_finite()) {
            return true;
        }
        let d_gyro = [
            p.gyro_rps[0] - s.gyro_rps[0],
            p.gyro_rps[1] - s.gyro_rps[1],
            p.gyro_rps[2] - s.gyro_rps[2],
        ];
        let chi2 = Self::compute_chi_square(d_gyro, gyro_var);
        chi2 > self.chi_square_threshold
    }
}

impl DualImuManager {
    /// Runs one FDIR epoch and returns the reading of the active IMU.
    ///
    /// A non-finite secondary sample marks `secondary_healthy` false for that
    /// epoch and is never counted against the primary.
    ///
    /// A finite gyro disagreement cannot be attributed by a two-IMU comparison,
    /// so it is counted against the primary while the secondary is healthy.
    ///
    /// The manager only switches to the secondary while `secondary_healthy` is
    /// true. When both IMUs are faulty it holds the active IMU, and callers must
    /// check both health flags before using the returned reading.
    pub fn update_and_select(
        &mut self,
        p: &ImuReading,
        s: &ImuReading,
        gyro_var: [f32; 3],
    ) -> ImuReading {
        let faulty = self.evaluate_sensors(p, s, gyro_var);
        self.secondary_healthy = reading_is_finite(s);
        let primary_fault = faulty && (!reading_is_finite(p) || self.secondary_healthy);
        if primary_fault && self.primary_healthy {
            self.consecutive_faults += 1;
            if self.consecutive_faults >= self.max_consecutive_faults {
                self.primary_healthy = false;
            }
        } else if !primary_fault {
            self.consecutive_faults = 0;
        }
        if !self.primary_healthy && self.secondary_healthy {
            self.active_imu = ImuId::Secondary;
        }
        if !self.primary_healthy && self.recovery_epochs > 0 {
            self.clean_epochs = if faulty { 0 } else { self.clean_epochs + 1 };
            if self.clean_epochs >= self.recovery_epochs {
                self.primary_healthy = true;
                self.active_imu = ImuId::Primary;
                self.consecutive_faults = 0;
                self.clean_epochs = 0;
            }
        }
        match self.active_imu {
            ImuId::Primary => *p,
            ImuId::Secondary => *s,
        }
    }
}

/// Detects a frozen sensor that repeats the exact same 3-axis sample.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StuckDetector {
    pub limit: u32,
    pub repeats: u32,
    pub last: Option<[f32; 3]>,
}

impl StuckDetector {
    pub fn new(limit: u32) -> Self {
        Self {
            limit,
            repeats: 0,
            last: None,
        }
    }
}

impl StuckDetector {
    /// Feeds one sample and returns true once it has repeated `limit` times.
    pub fn update(&mut self, sample: [f32; 3]) -> bool {
        if self.last == Some(sample) {
            self.repeats = self.repeats.saturating_add(1);
        } else {
            self.repeats = 0;
        }
        self.last = Some(sample);
        self.repeats >= self.limit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dual_imu_normal_tracking() {
        let mut mgr = DualImuManager::default_config();
        let imu1 = ImuReading {
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.01, -0.01, 0.0],
        };
        let imu2 = ImuReading {
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.012, -0.009, 0.001],
        };
        let sel = mgr.update_and_select(&imu1, &imu2, [0.01; 3]);
        assert_eq!(mgr.active_imu, ImuId::Primary);
        assert_eq!(sel, imu1);
    }
}

#[test]
fn test_primary_gyro_runaway_failover_within_3_epochs() {
    let mut mgr = DualImuManager::default_config();
    let good = ImuReading {
        accel_mps2: [0.0, 0.0, -9.81],
        gyro_rps: [0.0, 0.0, 0.0],
    };
    let runaway = ImuReading {
        accel_mps2: [0.0, 0.0, -9.81],
        gyro_rps: [2.5, -3.0, 1.8],
    };
    // Epoch 1
    mgr.update_and_select(&runaway, &good, [0.01; 3]);
    assert_eq!(mgr.consecutive_faults, 1);
    assert_eq!(mgr.active_imu, ImuId::Primary);

    // Epoch 2
    mgr.update_and_select(&runaway, &good, [0.01; 3]);
    assert_eq!(mgr.consecutive_faults, 2);
    assert_eq!(mgr.active_imu, ImuId::Primary);

    // Epoch 3 -> failover!
    let sel = mgr.update_and_select(&runaway, &good, [0.01; 3]);
    assert_eq!(mgr.consecutive_faults, 3);
    assert_eq!(mgr.active_imu, ImuId::Secondary);
    assert!(!mgr.primary_healthy);
    assert_eq!(sel, good);
}
