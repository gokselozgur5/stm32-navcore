//! Deterministic INS strapdown integration and Coriolis compensator.
//!
//! Note: Computations are carried out in double-precision (`f64`) using software
//! floating point on single-precision targets to prevent numerical drift.

/// Errors occurring during strapdown mechanization and Coriolis compensation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InsStrapdownError {
    /// Non-finite coordinate, rate, or acceleration encountered.
    NonFinite,
    /// Latitude outside valid range [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Delta time dt is non-positive or non-finite.
    InvalidDeltaTime,
    /// Altitude outside supported range [-10_000 m, 100_000 m].
    InvalidAltitude,
    /// Quaternion norm is zero or degenerate.
    SingularQuaternion,
}

impl core::fmt::Display for InsStrapdownError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NonFinite => write!(f, "non-finite input encountered"),
            Self::InvalidLatitude => write!(f, "latitude must be within [-pi/2, pi/2]"),
            Self::InvalidDeltaTime => write!(f, "delta time must be positive and finite"),
            Self::InvalidAltitude => write!(f, "altitude must be between -10000 and 100000 m"),
            Self::SingularQuaternion => write!(f, "attitude quaternion is singular"),
        }
    }
}

pub use crate::geo::earth_rate::principal_radii_of_curvature;
pub use crate::geo::{WGS84_A, WGS84_E2, WGS84_OMEGA_EARTH};

/// Earth rotation angular velocity in rad/s.
pub const EARTH_ROTATION_RATE: f64 = WGS84_OMEGA_EARTH;

/// Maps GeoError to InsStrapdownError.
fn map_geo_err(e: crate::geo::GeoError) -> InsStrapdownError {
    match e {
        crate::geo::GeoError::NonFinite => InsStrapdownError::NonFinite,
        crate::geo::GeoError::InvalidLatitude | crate::geo::GeoError::InvalidLongitude => {
            InsStrapdownError::InvalidLatitude
        }
    }
}

/// Computes WGS-84 meridian radius of curvature R_M in meters.
pub fn meridian_radius(lat_rad: f64) -> Result<f64, InsStrapdownError> {
    principal_radii_of_curvature(lat_rad)
        .map(|(rm, _)| rm)
        .map_err(map_geo_err)
}

/// Computes WGS-84 prime vertical (transverse) radius of curvature R_N in meters.
pub fn transverse_radius(lat_rad: f64) -> Result<f64, InsStrapdownError> {
    principal_radii_of_curvature(lat_rad)
        .map(|(_, rn)| rn)
        .map_err(map_geo_err)
}

/// Computes Earth rotation rate vector in local NED navigation frame.
pub fn earth_rate_ned(lat_rad: f64) -> Result<[f64; 3], InsStrapdownError> {
    crate::geo::earth_rate::earth_rotation_rate(lat_rad)
        .map(|ned| [ned.north, ned.east, ned.down])
        .map_err(map_geo_err)
}

/// Computes 3D vector cross product u x v.
#[inline]
pub fn cross_product(u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
}

/// Computes craft transport rate vector in local NED navigation frame.
pub fn transport_rate_ned(
    lat_rad: f64,
    alt_m: f64,
    vel_ned: [f64; 3],
) -> Result<[f64; 3], InsStrapdownError> {
    if !alt_m.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    if alt_m < -10_000.0 || alt_m > 100_000.0 {
        return Err(InsStrapdownError::InvalidAltitude);
    }
    crate::geo::earth_rate::transport_rate(lat_rad, alt_m, vel_ned[0], vel_ned[1])
        .map(|ned| [ned.north, ned.east, ned.down])
        .map_err(map_geo_err)
}

/// Computes Coriolis and transport acceleration in NED frame.
pub fn coriolis_acceleration(
    lat_rad: f64,
    alt_m: f64,
    vel_ned: [f64; 3],
) -> Result<[f64; 3], InsStrapdownError> {
    let w_ie = earth_rate_ned(lat_rad)?;
    let w_en = transport_rate_ned(lat_rad, alt_m, vel_ned)?;
    let w_sum = [
        2.0 * w_ie[0] + w_en[0],
        2.0 * w_ie[1] + w_en[1],
        2.0 * w_ie[2] + w_en[2],
    ];
    Ok(cross_product(w_sum, vel_ned))
}

/// Computes Somigliana normal gravity with elevation correction.
pub fn somigliana_gravity(lat_rad: f64, alt_m: f64) -> Result<f64, InsStrapdownError> {
    crate::geo::gravity_wgs84::normal_gravity_elevation(lat_rad, alt_m).map_err(|e| match e {
        crate::geo::gravity_wgs84::Wgs84GravityError::NonFinite => InsStrapdownError::NonFinite,
        crate::geo::gravity_wgs84::Wgs84GravityError::InvalidLatitude => {
            InsStrapdownError::InvalidLatitude
        }
        crate::geo::gravity_wgs84::Wgs84GravityError::InvalidAltitude => {
            InsStrapdownError::InvalidAltitude
        }
        crate::geo::gravity_wgs84::Wgs84GravityError::SingularRadius => {
            InsStrapdownError::NonFinite
        }
    })
}

/// Navigation state for INS strapdown integration in local navigation frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StrapdownState {
    /// Geodetic latitude in radians.
    pub lat_rad: f64,
    /// Geodetic longitude in radians.
    pub lon_rad: f64,
    /// Geodetic altitude above reference ellipsoid in meters.
    pub alt_m: f64,
    /// Velocity in NED frame [v_North, v_East, v_Down] in m/s.
    pub vel_ned: [f64; 3],
    /// Integrated relative displacement in NED frame in meters.
    pub pos_ned: [f64; 3],
    /// Body-to-NED attitude quaternion [w, x, y, z].
    pub quat: [f64; 4],
}

impl StrapdownState {
    /// Creates a new StrapdownState with initial geodetic position.
    pub fn new(lat_rad: f64, lon_rad: f64, alt_m: f64) -> Self {
        Self {
            lat_rad,
            lon_rad,
            alt_m,
            vel_ned: [0.0, 0.0, 0.0],
            pos_ned: [0.0, 0.0, 0.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}

/// Configuration options for INS strapdown integrator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InsStrapdownConfig {
    /// Enable Coriolis acceleration compensation.
    pub enable_coriolis: bool,
    /// Enable Earth rate attitude compensation.
    pub enable_earth_rate: bool,
    /// Enable transport rate attitude compensation.
    pub enable_transport_rate: bool,
    /// Constant gravity override in m/s^2 (None uses WGS-84 Somigliana).
    pub gravity_override: Option<f64>,
}

impl Default for InsStrapdownConfig {
    fn default() -> Self {
        Self {
            enable_coriolis: true,
            enable_earth_rate: true,
            enable_transport_rate: true,
            gravity_override: None,
        }
    }
}

/// Normalizes a 4D quaternion [w, x, y, z].
pub fn quat_normalize(q: [f64; 4]) -> Result<[f64; 4], InsStrapdownError> {
    let norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if !norm_sq.is_finite() || norm_sq < 1e-12 {
        return Err(InsStrapdownError::SingularQuaternion);
    }
    let inv_norm = 1.0 / libm::sqrt(norm_sq);
    Ok([
        q[0] * inv_norm,
        q[1] * inv_norm,
        q[2] * inv_norm,
        q[3] * inv_norm,
    ])
}

/// Rotates 3D vector v from body frame to navigation frame via quaternion.
pub fn quat_rotate_vec(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let qv = [q[1], q[2], q[3]];
    let t = [
        2.0 * (qv[1] * v[2] - qv[2] * v[1]),
        2.0 * (qv[2] * v[0] - qv[0] * v[2]),
        2.0 * (qv[0] * v[1] - qv[1] * v[0]),
    ];
    [
        v[0] + q[0] * t[0] + (qv[1] * t[2] - qv[2] * t[1]),
        v[1] + q[0] * t[1] + (qv[2] * t[0] - qv[0] * t[2]),
        v[2] + q[0] * t[2] + (qv[0] * t[1] - qv[1] * t[0]),
    ]
}

/// Multiplies two quaternions p * q.
pub fn quat_mul(p: [f64; 4], q: [f64; 4]) -> [f64; 4] {
    [
        p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
        p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
        p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
        p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0],
    ]
}

/// Integrates attitude quaternion with angular rate vector omega_b over dt.
pub fn quat_integrate(
    q: [f64; 4],
    omega: [f64; 3],
    dt_s: f64,
) -> Result<[f64; 4], InsStrapdownError> {
    let theta_sq = (omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]) * dt_s * dt_s;
    let theta = libm::sqrt(theta_sq);
    let (s, c) = if theta < 1e-8 {
        (0.5 * dt_s, 1.0 - 0.125 * theta_sq)
    } else {
        let half_th = 0.5 * theta;
        (libm::sin(half_th) / (theta / dt_s), libm::cos(half_th))
    };
    let dq = [c, omega[0] * s, omega[1] * s, omega[2] * s];
    quat_normalize(quat_mul(q, dq))
}

/// INS strapdown mechanization and Coriolis compensator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InsStrapdownIntegrator {
    state: StrapdownState,
    config: InsStrapdownConfig,
}

impl InsStrapdownIntegrator {
    /// Creates a new INS strapdown integrator with initial state and default config.
    pub fn new(initial_state: StrapdownState) -> Self {
        Self::with_config(initial_state, InsStrapdownConfig::default())
    }

    /// Creates a new INS strapdown integrator with custom config.
    pub fn with_config(initial_state: StrapdownState, config: InsStrapdownConfig) -> Self {
        Self {
            state: initial_state,
            config,
        }
    }
}

impl InsStrapdownIntegrator {
    /// Returns current navigation state.
    pub fn state(&self) -> &StrapdownState {
        &self.state
    }

    /// Returns mutable reference to navigation state.
    pub fn state_mut(&mut self) -> &mut StrapdownState {
        &mut self.state
    }

    /// Returns reference to integrator config.
    pub fn config(&self) -> &InsStrapdownConfig {
        &self.config
    }
}

/// Computes total navigation frame rotation rate in NED frame.
pub fn nav_frame_rate(
    lat_rad: f64,
    alt_m: f64,
    vel_ned: [f64; 3],
    enable_earth: bool,
    enable_transport: bool,
) -> Result<[f64; 3], InsStrapdownError> {
    let mut w = [0.0; 3];
    if enable_earth {
        let w_ie = earth_rate_ned(lat_rad)?;
        w[0] += w_ie[0];
        w[2] += w_ie[2];
    }
    if enable_transport {
        let w_en = transport_rate_ned(lat_rad, alt_m, vel_ned)?;
        w[0] += w_en[0];
        w[1] += w_en[1];
        w[2] += w_en[2];
    }
    Ok(w)
}

/// Rotates 3D vector from navigation to body frame via conjugate quaternion.
#[inline]
pub fn quat_rotate_vec_inv(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    quat_rotate_vec([q[0], -q[1], -q[2], -q[3]], v)
}

impl InsStrapdownIntegrator {
    /// Integrates craft attitude with gyro reading and Earth/transport rate compensation.
    pub fn update_attitude(
        &mut self,
        gyro_b: [f64; 3],
        dt_s: f64,
    ) -> Result<[f64; 4], InsStrapdownError> {
        let w_in_n = nav_frame_rate(
            self.state.lat_rad,
            self.state.alt_m,
            self.state.vel_ned,
            self.config.enable_earth_rate,
            self.config.enable_transport_rate,
        )?;
        let w_in_b = quat_rotate_vec_inv(self.state.quat, w_in_n);
        let w_nb_b = [
            gyro_b[0] - w_in_b[0],
            gyro_b[1] - w_in_b[1],
            gyro_b[2] - w_in_b[2],
        ];
        self.state.quat = quat_integrate(self.state.quat, w_nb_b, dt_s)?;
        Ok(self.state.quat)
    }
}

impl InsStrapdownIntegrator {
    /// Computes kinematic acceleration in NED frame from body specific force.
    pub fn kinematic_accel(&self, accel_b: [f64; 3]) -> Result<[f64; 3], InsStrapdownError> {
        let f_n = quat_rotate_vec(self.state.quat, accel_b);
        let g = match self.config.gravity_override {
            Some(g_val) => g_val,
            None => somigliana_gravity(self.state.lat_rad, self.state.alt_m)?,
        };
        let a_cor = if self.config.enable_coriolis {
            coriolis_acceleration(self.state.lat_rad, self.state.alt_m, self.state.vel_ned)?
        } else {
            [0.0, 0.0, 0.0]
        };
        Ok([f_n[0] - a_cor[0], f_n[1] - a_cor[1], f_n[2] + g - a_cor[2]])
    }
}

impl InsStrapdownIntegrator {
    /// Integrates velocity using kinematic acceleration and returns average velocity over step.
    pub fn update_velocity(&mut self, a_n: [f64; 3], dt_s: f64) -> [f64; 3] {
        let v_old = self.state.vel_ned;
        let v_new = [
            v_old[0] + a_n[0] * dt_s,
            v_old[1] + a_n[1] * dt_s,
            v_old[2] + a_n[2] * dt_s,
        ];
        self.state.vel_ned = v_new;
        [
            0.5 * (v_old[0] + v_new[0]),
            0.5 * (v_old[1] + v_new[1]),
            0.5 * (v_old[2] + v_new[2]),
        ]
    }
}

impl InsStrapdownIntegrator {
    /// Integrates relative NED position and geodetic coordinates using average velocity.
    pub fn update_position(&mut self, v_avg: [f64; 3], dt_s: f64) -> Result<(), InsStrapdownError> {
        self.state.pos_ned[0] += v_avg[0] * dt_s;
        self.state.pos_ned[1] += v_avg[1] * dt_s;
        self.state.pos_ned[2] += v_avg[2] * dt_s;
        let r_m = meridian_radius(self.state.lat_rad)?;
        let r_n = transverse_radius(self.state.lat_rad)?;
        let new_lat = self.state.lat_rad + (v_avg[0] * dt_s) / (r_m + self.state.alt_m);
        if libm::fabs(new_lat) > core::f64::consts::FRAC_PI_2 {
            return Err(InsStrapdownError::InvalidLatitude);
        }
        self.state.lat_rad = new_lat;
        let cos_lat = libm::cos(self.state.lat_rad);
        if libm::fabs(cos_lat) > 1e-6 {
            let two_pi = 2.0 * core::f64::consts::PI;
            let lon = self.state.lon_rad + (v_avg[1] * dt_s) / ((r_n + self.state.alt_m) * cos_lat);
            self.state.lon_rad = lon - two_pi * libm::round(lon / two_pi);
        }
        self.state.alt_m -= v_avg[2] * dt_s;
        Ok(())
    }
}

impl InsStrapdownIntegrator {
    /// Steps strapdown mechanization forward by dt_s with IMU measurements.
    pub fn step(
        &mut self,
        accel_b: [f64; 3],
        gyro_b: [f64; 3],
        dt_s: f64,
    ) -> Result<StrapdownState, InsStrapdownError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(InsStrapdownError::InvalidDeltaTime);
        }
        if !accel_b[0].is_finite() || !accel_b[1].is_finite() || !accel_b[2].is_finite() {
            return Err(InsStrapdownError::NonFinite);
        }
        if !gyro_b[0].is_finite() || !gyro_b[1].is_finite() || !gyro_b[2].is_finite() {
            return Err(InsStrapdownError::NonFinite);
        }
        self.update_attitude(gyro_b, dt_s)?;
        let a_n = self.kinematic_accel(accel_b)?;
        let v_avg = self.update_velocity(a_n, dt_s);
        self.update_position(v_avg, dt_s)?;
        Ok(self.state)
    }
}

impl InsStrapdownIntegrator {
    /// Steps strapdown mechanization forward using single-precision f32 IMU inputs.
    pub fn step_f32(
        &mut self,
        accel_b: [f32; 3],
        gyro_b: [f32; 3],
        dt_s: f32,
    ) -> Result<StrapdownState, InsStrapdownError> {
        self.step(
            [accel_b[0] as f64, accel_b[1] as f64, accel_b[2] as f64],
            [gyro_b[0] as f64, gyro_b[1] as f64, gyro_b[2] as f64],
            dt_s as f64,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constants_and_radii() {
        assert_eq!(WGS84_A, 6_378_137.0);
        let rm_eq = meridian_radius(0.0).expect("meridian eq");
        let rn_eq = transverse_radius(0.0).expect("transverse eq");
        assert!((rn_eq - WGS84_A).abs() < 1e-6);
        assert!(rm_eq < rn_eq);
        assert!((rm_eq - WGS84_A * (1.0 - WGS84_E2)).abs() < 1e-4);
        assert_eq!(
            meridian_radius(2.0),
            Err(InsStrapdownError::InvalidLatitude)
        );
        assert_eq!(
            transverse_radius(f64::NAN),
            Err(InsStrapdownError::NonFinite)
        );
    }

    #[test]
    fn test_earth_and_transport_rate() {
        let w_eq = earth_rate_ned(0.0).expect("earth rate eq");
        assert!((w_eq[0] - EARTH_ROTATION_RATE).abs() < 1e-10);
        assert_eq!(w_eq[1], 0.0);
        assert!((w_eq[2] - 0.0).abs() < 1e-10);
        let half_pi = core::f64::consts::FRAC_PI_2;
        let w_np = earth_rate_ned(half_pi).expect("earth rate np");
        assert!(w_np[0].abs() < 1e-10);
        assert!((w_np[2] + EARTH_ROTATION_RATE).abs() < 1e-10);
        let w_trans = transport_rate_ned(0.0, 0.0, [0.0, 100.0, 0.0]).expect("transport");
        assert!((w_trans[0] - 100.0 / WGS84_A).abs() < 1e-8);
        assert_eq!(w_trans[1], 0.0);
    }

    #[test]
    fn test_coriolis_and_eotvos() {
        let a_stat = coriolis_acceleration(0.0, 0.0, [0.0, 0.0, 0.0]).expect("stationary");
        assert_eq!(a_stat, [0.0, 0.0, 0.0]);
        let a_east = coriolis_acceleration(0.0, 0.0, [0.0, 100.0, 0.0]).expect("east");
        assert!(a_east[2] > 0.0);
        let a_west = coriolis_acceleration(0.0, 0.0, [0.0, -100.0, 0.0]).expect("west");
        assert!(a_west[2] < 0.0);
        let a_north = coriolis_acceleration(0.0, 0.0, [100.0, 0.0, 0.0]).expect("north");
        assert!(a_north[2] > 0.0);
        assert_eq!(a_north[0], 0.0);
    }

    #[test]
    fn test_quaternion_rotations_and_integration() {
        let q_id = [1.0, 0.0, 0.0, 0.0];
        let v = [1.0, 2.0, 3.0];
        assert_eq!(quat_rotate_vec(q_id, v), v);
        assert_eq!(quat_rotate_vec_inv(q_id, v), v);
        let q_rot = quat_integrate(q_id, [0.0, 0.0, 0.0], 0.1).expect("int zero");
        assert_eq!(q_rot, q_id);
        let q_deg = [0.0, 0.0, 0.0, 0.0];
        assert_eq!(
            quat_normalize(q_deg),
            Err(InsStrapdownError::SingularQuaternion)
        );
    }

    #[test]
    fn test_stationary_holding_and_free_fall() {
        let initial_state = StrapdownState::new(0.0, 0.0, 0.0);
        let mut ins = InsStrapdownIntegrator::new(initial_state);
        let g = somigliana_gravity(0.0, 0.0).expect("gravity");
        let st = ins
            .step([0.0, 0.0, -g], [0.0, 0.0, 0.0], 0.01)
            .expect("step");
        assert!(st.vel_ned[0].abs() < 1e-6);
        assert!(st.vel_ned[1].abs() < 1e-6);
        assert!(st.vel_ned[2].abs() < 1e-6);
        let mut ff_ins = InsStrapdownIntegrator::new(initial_state);
        let ff_st = ff_ins
            .step([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0)
            .expect("ff");
        assert!((ff_st.vel_ned[2] - g).abs() < 1e-4);
    }

    #[test]
    fn test_input_validation_and_errors() {
        let mut ins = InsStrapdownIntegrator::new(StrapdownState::new(0.0, 0.0, 0.0));
        assert_eq!(
            ins.step([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0),
            Err(InsStrapdownError::InvalidDeltaTime)
        );
        assert_eq!(
            ins.step([f64::NAN, 0.0, 0.0], [0.0, 0.0, 0.0], 0.1),
            Err(InsStrapdownError::NonFinite)
        );
        assert_eq!(
            ins.step([0.0, 0.0, 0.0], [f64::INFINITY, 0.0, 0.0], 0.1),
            Err(InsStrapdownError::NonFinite)
        );
        let err = InsStrapdownError::InvalidLatitude;
        assert_eq!(format!("{}", err), "latitude must be within [-pi/2, pi/2]");
    }

    #[test]
    fn test_trajectory_and_step_f32() {
        let mut ins = InsStrapdownIntegrator::new(StrapdownState::new(0.0, 0.0, 0.0));
        let g = somigliana_gravity(0.0, 0.0).expect("gravity");
        let st = ins
            .step([10.0, 0.0, -g], [0.0, 0.0, 0.0], 1.0)
            .expect("step");
        assert!((st.vel_ned[0] - 10.0).abs() < 1e-4);
        assert!((st.pos_ned[0] - 5.0).abs() < 1e-4);
        assert!(st.lat_rad > 0.0);
        let st_f32 = ins
            .step_f32([0.0f32, 0.0f32, -g as f32], [0.0f32; 3], 0.1f32)
            .expect("f32");
        assert!(st_f32.pos_ned[0] > 5.0);
    }

    #[test]
    fn test_strapdown_matches_wgs84_reference_at_60_degrees() {
        let lat = 60.0f64.to_radians();
        let state = StrapdownState::new(lat, 0.0, 0.0);
        let mut ins = InsStrapdownIntegrator::new(state);
        ins.update_position([100.0, 100.0, 0.0], 1.0).unwrap();
        let dlat = ins.state().lat_rad - lat;
        assert!((ins.state().lon_rad - 3.127_914_98e-5).abs() < 1e-12);
        assert!((dlat - 1.566_550_06e-5).abs() < 1e-12);
    }
}
