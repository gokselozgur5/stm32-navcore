use crate::geo::{Ecef, GeoError, Lla};

pub fn ecef_to_lla(ecef: &Ecef) -> Result<Lla, GeoError> {
    if !ecef.x.is_finite() || !ecef.y.is_finite() || !ecef.z.is_finite() {
        return Err(GeoError::NonFinite);
    }
    Ok(Lla {
        lat_rad: 0.0,
        lon_rad: 0.0,
        alt_m: 0.0,
    })
}
