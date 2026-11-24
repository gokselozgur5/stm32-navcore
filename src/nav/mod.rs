//! Inertial and integrated navigation modules.

#[cfg(feature = "f64-geodesy")]
pub mod dead_reckoning;
#[cfg(feature = "f64-geodesy")]
pub use dead_reckoning::{dead_reckoning_step, DeadReckoningState, NavError};

pub mod ekf;
pub use ekf::{AccelBiasEstimator, EkfError, NavEkf, STATE_DIM};

pub mod fdir;
pub use fdir::{DualImuManager, ImuId, ImuReading};

pub mod gnss_dop;
pub use gnss_dop::{
    compute_dop, compute_dop_from_az_el, DopAccumulator, DopError, GnssDop, SatelliteLos,
};

#[cfg(feature = "f64-geodesy")]
pub mod ins_strapdown;
#[cfg(feature = "f64-geodesy")]
pub use ins_strapdown::{
    coriolis_acceleration, earth_rate_ned, meridian_radius, somigliana_gravity, transport_rate_ned,
    transverse_radius, InsStrapdownConfig, InsStrapdownError, InsStrapdownIntegrator,
    StrapdownState,
};

pub mod turn_rate;
pub use turn_rate::{
    accelerated_stall_speed, bank_angle_deg, bank_angle_from_radius, bank_angle_rad,
    centripetal_accel_mps2, compute_coordinated_turn, load_factor, standard_rate_bank_angle_rad,
    time_to_turn_s, turn_radius_from_rate, turn_radius_m, turn_rate_deg_s, turn_rate_rad_s,
    TurnAerodynamics, TurnRateError, STANDARD_GRAVITY_MPS2, STANDARD_RATE_TURN_DEG_S,
    STANDARD_RATE_TURN_RAD_S,
};

pub mod vel_damping;
pub use vel_damping::{
    calculate_innovation, damp_velocity_step, fuse_velocity_step, VelDampingConfig,
    VelDampingError, VelDampingFilter,
};

pub mod zupt;
pub use zupt::{
    DefaultZuptDetector, StancePhase, ZuptConfig, ZuptDetector, ZuptError, ZuptMetrics,
};
pub type StationaryDetector = ZuptDetector<10>;
