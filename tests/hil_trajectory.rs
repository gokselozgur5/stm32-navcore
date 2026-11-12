use core::f32::consts::PI;
use stm32_navcore::ekf::{NavEkf, StationaryDetector};
use stm32_navcore::math::{pressure_hpa_to_altitude_m, tilt_compensated_yaw, Mat, Quaternion};
use stm32_navcore::protocol::NavTelemetry;

struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0xDEAD_BEEF_CAFE_BABE
            } else {
                seed
            },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / 16777216.0
    }

    fn next_gaussian_approx(&mut self, std_dev: f32) -> f32 {
        let mut sum = 0.0f32;
        for _ in 0..12 {
            sum += self.next_f32();
        }
        (sum - 6.0) * std_dev
    }
}

fn wrap_pi(mut a: f32) -> f32 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

fn altitude_to_pressure_hpa(alt_m: f32, p0_hpa: f32) -> f32 {
    let base = 1.0f32 - (alt_m / 44330.0f32);
    p0_hpa * libm::powf(base, 5.255f32)
}

#[test]
fn test_cortex_m4f_stack_and_state_footprint_invariants() {
    assert!(core::mem::size_of::<NavEkf>() <= 1024);
    assert!(core::mem::size_of::<StationaryDetector>() <= 64);
    assert!(core::mem::size_of::<NavTelemetry>() <= 128);
}

#[test]
fn test_multimodal_flight_hil_trajectory_with_baro_zupt_and_outage() {
    let mut rng = XorShift64::new(1337);
    let mut ekf = NavEkf::new();
    let mut zupt_detector = StationaryDetector::new(10, 0.05, 0.03);

    let total_steps = 4000;
    let dt = 0.01f32;
    let p0_hpa = 1013.25f32;

    let hard_iron = [20.0f32, -15.0, 5.0];
    let soft_iron = Mat::<3, 3>([
        [1.05f32, 0.02, 0.0],
        [0.02, 0.98f32, 0.0],
        [0.0, 0.0, 1.0f32],
    ]);
    let inv_soft = soft_iron.inv_3x3().unwrap();
    let (bn, bd) = (20.0f32, 40.0f32);
    let true_gyro_bias_z = 0.005f32;

    let mut uart_buf = [0u8; NavTelemetry::MAX_UART_FRAME_LEN];
    let mut stationary_zupt_count = 0usize;
    let mut max_vertical_drift_outage = 0.0f32;
    let mut pre_outage_pos_cov = 0.0f32;
    let mut post_outage_pos_cov = 0.0f32;

    for step in 0..total_steps {
        let t = step as f32 * dt;

        let (p_n, p_e, p_d, v_n, v_e, v_d, a_n, a_e, a_d, yaw) = if t < 5.0 {
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        } else if t < 10.0 {
            let tau = t - 5.0;
            (0.0, 0.0, -2.0 * tau, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0)
        } else if t < 30.0 {
            let tau = t - 10.0;
            let omega = 2.0 * PI / 20.0;
            let r_traj = 20.0f32;
            let pn = r_traj * libm::sinf(omega * tau);
            let pe = r_traj * libm::sinf(2.0 * omega * tau);
            let vn = r_traj * omega * libm::cosf(omega * tau);
            let ve = 2.0 * r_traj * omega * libm::cosf(2.0 * omega * tau);
            let an = -r_traj * omega * omega * libm::sinf(omega * tau);
            let ae = -4.0 * r_traj * omega * omega * libm::sinf(2.0 * omega * tau);
            let y = libm::atan2f(ve, vn);
            (pn, pe, -10.0, vn, ve, 0.0, an, ae, 0.0, y)
        } else {
            let tau = t - 30.0;
            let p_down = -10.0 + 1.0 * tau;
            (
                0.0,
                0.0,
                p_down.min(0.0),
                0.0,
                0.0,
                if p_down < 0.0 { 1.0 } else { 0.0 },
                0.0,
                0.0,
                0.0,
                0.0,
            )
        };

        let next_t = (step + 1) as f32 * dt;
        let next_yaw = if next_t < 10.0 {
            0.0
        } else if next_t < 30.0 {
            let tau = next_t - 10.0;
            let omega = 2.0 * PI / 20.0;
            let r_traj = 20.0f32;
            let vn = r_traj * omega * libm::cosf(omega * tau);
            let ve = 2.0 * r_traj * omega * libm::cosf(2.0 * omega * tau);
            libm::atan2f(ve, vn)
        } else {
            0.0
        };
        let yaw_rate = wrap_pi(next_yaw - yaw) / dt;

        let half_yaw = yaw * 0.5;
        let q_true = Quaternion::new(libm::cosf(half_yaw), 0.0, 0.0, libm::sinf(half_yaw));
        let q_conj = Quaternion::new(q_true.w, -q_true.x, -q_true.y, -q_true.z);

        let a_grav_ned = [a_n, a_e, a_d - 9.80665];
        let true_accel_b = q_conj.rotate_vec(a_grav_ned);
        let true_gyro_b = [0.0, 0.0, yaw_rate];

        let meas_accel_b = [
            true_accel_b[0] + rng.next_gaussian_approx(0.01),
            true_accel_b[1] + rng.next_gaussian_approx(0.01),
            true_accel_b[2] + rng.next_gaussian_approx(0.01),
        ];

        let meas_gyro_b = [
            true_gyro_b[0] + rng.next_gaussian_approx(0.001),
            true_gyro_b[1] + rng.next_gaussian_approx(0.001),
            true_gyro_b[2] + true_gyro_bias_z + rng.next_gaussian_approx(0.001),
        ];

        if step == 0 {
            ekf.p_ned_m = [p_n, p_e, p_d];
            ekf.v_ned_mps = [v_n, v_e, v_d];
            ekf.att = q_true;
        }

        ekf.predict(meas_accel_b, meas_gyro_b, dt)
            .expect("EKF predict must succeed");

        let is_stationary = zupt_detector.update(meas_accel_b, meas_gyro_b);
        if is_stationary && t < 4.5 {
            let _ = ekf.update_zupt(0.001, 25.0);
            stationary_zupt_count += 1;
        }

        if step % 10 == 0 {
            let true_alt = -p_d;
            let p_meas =
                altitude_to_pressure_hpa(true_alt, p0_hpa) + rng.next_gaussian_approx(0.05);
            if let Ok(alt_calc) = pressure_hpa_to_altitude_m(p_meas, p0_hpa) {
                let _ = ekf.update_baro_altitude(alt_calc, 0.05, 16.0);
            }
        }

        if step % 10 == 0 {
            let (r, p) = (0.0f32, 0.0f32);
            let (sr, cr, sp, cp) = (libm::sinf(r), libm::cosf(r), libm::sinf(p), libm::cosf(p));
            let (cy, sy) = (libm::cosf(yaw), libm::sinf(yaw));
            let bx = bn * cy * cp - bd * sp;
            let by = -bn * sy * cr + (bn * cy * sp + bd * cp) * sr;
            let bz = bn * sy * sr + (bn * cy * sp + bd * cp) * cr;
            let uncal = inv_soft.matmul(&Mat([[bx], [by], [bz]]));
            let raw = [
                uncal.0[0][0] + hard_iron[0] + rng.next_gaussian_approx(0.05),
                uncal.0[1][0] + hard_iron[1] + rng.next_gaussian_approx(0.05),
                uncal.0[2][0] + hard_iron[2] + rng.next_gaussian_approx(0.05),
            ];
            let (roll, pitch, _) = ekf.att.to_euler_zyx();
            if let Ok(heading) = tilt_compensated_yaw(raw, hard_iron, soft_iron, roll, pitch) {
                let _ = ekf.update_mag_yaw(heading, 0.02, 9.0);
            }
        }

        let is_gnss_outage = (20.0..30.0).contains(&t);

        if !is_gnss_outage && step % 20 == 0 {
            let gnss_p = [
                p_n + rng.next_gaussian_approx(0.05),
                p_e + rng.next_gaussian_approx(0.05),
                p_d + rng.next_gaussian_approx(0.05),
            ];
            let gnss_v = [
                v_n + rng.next_gaussian_approx(0.02),
                v_e + rng.next_gaussian_approx(0.02),
                v_d + rng.next_gaussian_approx(0.02),
            ];
            let _ = ekf.update_gnss_pos(gnss_p, 0.02, 16.0);
            let _ = ekf.update_gnss_vel(gnss_v, 0.005, 16.0);
        }

        if (t - 19.99).abs() < 0.005 {
            pre_outage_pos_cov = ekf.cov.0[0][0] + ekf.cov.0[1][1];
        }

        if is_gnss_outage {
            let vertical_err = (ekf.p_ned_m[2] - p_d).abs();
            if vertical_err > max_vertical_drift_outage {
                max_vertical_drift_outage = vertical_err;
            }
        }

        if (t - 29.99).abs() < 0.005 {
            post_outage_pos_cov = ekf.cov.0[0][0] + ekf.cov.0[1][1];
        }

        let (roll, pitch, ekf_yaw) = ekf.att.to_euler_zyx();
        let telem = NavTelemetry {
            timestamp_ms: (t * 1000.0) as u32,
            pos_ned_m: ekf.p_ned_m,
            vel_ned_mps: ekf.v_ned_mps,
            euler_rad: [roll, pitch, ekf_yaw],
            status: if is_gnss_outage { 0x02 } else { 0x01 },
        };

        let uart_bytes = telem
            .pack_uart(&mut uart_buf)
            .expect("UART pack must succeed");
        let unpacked_uart =
            NavTelemetry::unpack_uart(&uart_buf[..uart_bytes]).expect("UART unpack must succeed");
        assert_eq!(unpacked_uart, telem);
    }

    assert!(
        stationary_zupt_count > 0,
        "ZUPT should engage during stationary phase"
    );
    assert!(
        post_outage_pos_cov > pre_outage_pos_cov,
        "Horizontal position covariance must increase during 10s GNSS outage"
    );
    assert!(
        max_vertical_drift_outage < 1.0,
        "Vertical position drift aided by baro must be bounded <1.0m during 10s outage, got {max_vertical_drift_outage}"
    );
    assert!(
        (ekf.bg_z - true_gyro_bias_z).abs() < 0.005,
        "End-of-flight gyro bias error must be < 0.005 rad/s, got {}",
        (ekf.bg_z - true_gyro_bias_z).abs()
    );
}
