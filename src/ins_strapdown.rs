//! Deterministic INS strapdown integration and Coriolis compensator.

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

/// WGS-84 semi-major axis in meters.
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 first eccentricity squared.
pub const WGS84_E2: f64 = 0.006_694_379_990_14;

/// Earth rotation angular velocity in rad/s.
pub const EARTH_ROTATION_RATE: f64 = 7.292_115e-5;

/// Somigliana formula equatorial normal gravity in m/s^2.
pub const GAMMA_E: f64 = 9.780_325_335_9;

/// Somigliana formula parameter k.
pub const SOMIGLIANA_K: f64 = 0.001_931_852_652_41;

/// Computes WGS-84 meridian radius of curvature R_M in meters.
pub fn meridian_radius(lat_rad: f64) -> Result<f64, InsStrapdownError> {
    if !lat_rad.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(InsStrapdownError::InvalidLatitude);
    }
    let s = libm::sin(lat_rad);
    let denom = 1.0 - WGS84_E2 * s * s;
    let sqrt_denom = libm::sqrt(denom);
    Ok(WGS84_A * (1.0 - WGS84_E2) / (denom * sqrt_denom))
}

/// Computes WGS-84 prime vertical (transverse) radius of curvature R_N in meters.
pub fn transverse_radius(lat_rad: f64) -> Result<f64, InsStrapdownError> {
    if !lat_rad.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(InsStrapdownError::InvalidLatitude);
    }
    let s = libm::sin(lat_rad);
    let denom = 1.0 - WGS84_E2 * s * s;
    Ok(WGS84_A / libm::sqrt(denom))
}

/// Computes Earth rotation rate vector in local NED navigation frame.
pub fn earth_rate_ned(lat_rad: f64) -> Result<[f64; 3], InsStrapdownError> {
    if !lat_rad.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(InsStrapdownError::InvalidLatitude);
    }
    let c = libm::cos(lat_rad);
    let s = libm::sin(lat_rad);
    Ok([EARTH_ROTATION_RATE * c, 0.0, -EARTH_ROTATION_RATE * s])
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
    if !alt_m.is_finite()
        || !vel_ned[0].is_finite()
        || !vel_ned[1].is_finite()
        || !vel_ned[2].is_finite()
    {
        return Err(InsStrapdownError::NonFinite);
    }
    if alt_m < -10_000.0 || alt_m > 100_000.0 {
        return Err(InsStrapdownError::InvalidAltitude);
    }
    let r_m = meridian_radius(lat_rad)?;
    let r_n = transverse_radius(lat_rad)?;
    let tan_lat = libm::tan(lat_rad);
    let v_n = vel_ned[0];
    let v_e = vel_ned[1];
    Ok([v_e / (r_n + alt_m), -v_n / (r_m + alt_m), -v_e * tan_lat / (r_n + alt_m)])
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
    crate::gravity_wgs84::normal_gravity_elevation(lat_rad, alt_m).map_err(|e| match e {
        crate::gravity_wgs84::Wgs84GravityError::NonFinite => InsStrapdownError::NonFinite,
        crate::gravity_wgs84::Wgs84GravityError::InvalidLatitude => InsStrapdownError::InvalidLatitude,
        crate::gravity_wgs84::Wgs84GravityError::InvalidAltitude => InsStrapdownError::InvalidAltitude,
        crate::gravity_wgs84::Wgs84GravityError::SingularRadius => InsStrapdownError::NonFinite,
    })
}

/// Computes gravity vector in NED frame [0, 0, g].
pub fn gravity_ned(lat_rad: f64, alt_m: f64) -> Result<[f64; 3], InsStrapdownError> {
    let g = somigliana_gravity(lat_rad, alt_m)?;
    Ok([0.0, 0.0, g])
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
    Ok([q[0] * inv_norm, q[1] * inv_norm, q[2] * inv_norm, q[3] * inv_norm])
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
pub fn quat_integrate(q: [f64; 4], omega: [f64; 3], dt_s: f64) -> Result<[f64; 4], InsStrapdownError> {
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
        self.state.lat_rad += (v_avg[0] * dt_s) / (r_m + self.state.alt_m);
        let cos_lat = libm::cos(self.state.lat_rad);
        if libm::fabs(cos_lat) > 1e-6 {
            self.state.lon_rad += (v_avg[1] * dt_s) / ((r_n + self.state.alt_m) * cos_lat);
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
