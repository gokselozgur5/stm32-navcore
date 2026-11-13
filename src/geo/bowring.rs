use crate::geo::{Ecef, GeoError, Lla, WGS84_B};

pub fn ecef_to_lla(ecef: &Ecef) -> Result<Lla, GeoError> {
    if !ecef.x.is_finite() || !ecef.y.is_finite() || !ecef.z.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let p = libm::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    if p < 1e-12 {
        let half_pi = core::f64::consts::FRAC_PI_2;
        let (lat_rad, alt_m) = if ecef.z >= 0.0 {
            (half_pi, ecef.z - WGS84_B)
        } else {
            (-half_pi, -ecef.z - WGS84_B)
        };
        return Ok(Lla {
            lat_rad,
            lon_rad: 0.0,
            alt_m,
        });
    }
    let lon_rad = libm::atan2(ecef.y, ecef.x);
    Ok(Lla {
        lat_rad: 0.0,
        lon_rad,
        alt_m: 0.0,
    })
}
