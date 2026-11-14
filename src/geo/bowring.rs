use crate::geo::{Ecef, GeoError, Lla, WGS84_A, WGS84_B, WGS84_E2, WGS84_E_PRIME2};

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
    let theta = libm::atan2(ecef.z * WGS84_A, p * WGS84_B);
    let sin_th = libm::sin(theta);
    let cos_th = libm::cos(theta);
    let sin3_th = sin_th * sin_th * sin_th;
    let cos3_th = cos_th * cos_th * cos_th;

    let num = ecef.z + WGS84_E_PRIME2 * WGS84_B * sin3_th;
    let den = p - WGS84_E2 * WGS84_A * cos3_th;
    let lat_rad = if num == 0.0 && den <= 0.0 {
        0.0
    } else {
        libm::atan2(num, den)
    };
    let sin_lat = libm::sin(lat_rad);
    let cos_lat = libm::cos(lat_rad);
    let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    let alt_m = if libm::fabs(cos_lat) > 1e-6 {
        p / cos_lat - n
    } else {
        ecef.z / sin_lat - n * (1.0 - WGS84_E2)
    };
    Lla::new(lat_rad, lon_rad, alt_m)
}
