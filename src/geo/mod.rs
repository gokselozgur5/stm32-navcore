//! Geodesy module: Earth reference models and coordinate transforms.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeoError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
}

pub mod vincenty;

/// WGS-84 semi-major axis (equatorial radius) in meters.
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 reciprocal flattening (1/f).
pub const WGS84_INV_F: f64 = 298.257_223_563;

/// WGS-84 flattening: f = 1 / 298.257223563.
pub const WGS84_F: f64 = 1.0 / WGS84_INV_F;

/// WGS-84 semi-minor axis (polar radius) in meters: b = a * (1 - f).
pub const WGS84_B: f64 = 6_356_752.314_245_179;

/// WGS-84 first eccentricity squared: e^2 = 2f - f^2.
pub const WGS84_E2: f64 = 0.006_694_379_990_141_316_5;

/// WGS-84 second eccentricity squared: e'^2 = (a^2 - b^2) / b^2.
pub const WGS84_E_PRIME2: f64 = 0.006_739_496_742_276_434;

/// WGS-84 Earth rotation rate in rad/s.
pub const WGS84_OMEGA_EARTH: f64 = 7.292_115e-5;

pub mod ecef;
pub use ecef::{lla_to_ecef, Ecef, Lla};
pub mod bowring;
pub use bowring::ecef_to_lla;
pub mod ned;
pub use ned::{LtpFrame, Ned};
pub mod ecef_ned;
pub use ecef_ned::{
    ecef_to_geodetic, ecef_to_ned, geodetic_to_ecef, geodetic_to_ned, ned_to_ecef, ned_to_geodetic,
    EcefNedError, EcefPos, GeodeticPos, NedFrame, NedPos,
};

pub mod gravity_wgs84;
pub use gravity_wgs84::{
    gravity_ned, gravity_ned_f32, normal_gravity_elevation, normal_gravity_elevation_f32,
    somigliana_surface, somigliana_surface_f32, SphericalHarmonicGravity, Wgs84GravityCalculator,
    Wgs84GravityError,
};

pub mod gravity;
pub use gravity::{normal_gravity, normal_gravity_surface};

pub mod gravity_somigliana;

pub mod mag_declination;
pub use mag_declination::{
    estimate_wmm2020, estimate_wmm2020_rad, magnetic_to_true_heading_rad, MagDeclinationError,
    MagDeclinationLut, MagFieldEstimate, WMM2020_DECLINATION, WMM2020_INCLINATION, WMM2020_LUT,
};

pub mod wmm_declination;
pub use wmm_declination::{
    wmm_declination_deg, wmm_declination_rad, DeclinationError, DeclinationPolynomial,
};

pub mod bearing;
pub use bearing::{initial_bearing_deg, initial_bearing_rad};

pub mod earth_rate;
pub use earth_rate::{earth_rotation_rate, transport_rate};

pub mod haversine;
pub use haversine::{haversine, HaversineResult, EARTH_RADIUS_M};
