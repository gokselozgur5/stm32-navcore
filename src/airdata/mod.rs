//! Air data estimation: pitot-static airspeed, barometric altitude, variometer, and wind triangle.

pub mod airspeed;
pub mod baro_alt;
pub mod hypsometric;
pub mod pitot_tas;
pub mod variometer;
pub mod wind_estimator;

pub use airspeed::{
    air_density, calibrated_airspeed, equivalent_airspeed, impact_pressure_to_mach,
    incompressible_ias, true_airspeed, AirspeedError,
};
pub use baro_alt::{
    altitude_from_pressure_isa, altitude_from_pressure_qnh, calibrate_qnh,
    hypsometric_altitude as baro_hypsometric_altitude, pressure_at_altitude_isa, BaroAltError,
    BaroAltEstimator,
};
pub use hypsometric::{
    hypsometric_altitude, hypsometric_thickness, HypsometricError, HypsometricProfile,
    TempLapseCompensator,
};
pub use pitot_tas::{
    air_density as pitot_air_density, celsius_to_kelvin, density_ratio, diff_pressure_to_ias,
    diff_pressure_to_tas, ias_to_tas, ias_to_tas_with_density, AirspeedReading, PitotError,
    PitotTasEstimator,
};
pub use variometer::{VarioTone, Variometer, VariometerConfig, VariometerError};
pub use wind_estimator::{
    decompose_relative_wind, instantaneous_wind_from_course, instantaneous_wind_vector,
    wind_correction_angle_rad, wind_speed_and_direction, RelativeWind, WindEstimate, WindEstimator,
    WindEstimatorConfig, WindEstimatorError, WindVectorInfo,
};
