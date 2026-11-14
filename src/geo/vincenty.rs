//! Vincenty inverse geodetic solution on WGS84.

/// WGS84 semi-major axis in metres.
pub const A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const F: f64 = 1.0 / 298.257_223_563;
const B: f64 = A * (1.0 - F);

/// Result of the inverse problem.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inverse {
    /// Ellipsoidal distance in metres.
    pub distance_m: f64,
    /// Forward azimuth at the first point in radians, from north.
    pub azimuth1: f64,
    /// Forward azimuth at the second point in radians, from north.
    pub azimuth2: f64,
}

/// Ellipsoidal arc length from the converged auxiliary-sphere terms.
fn arc_length(cos2a: f64, sigma: f64, sin_s: f64, cos_s: f64, cos_2sm: f64) -> f64 {
    let u2 = cos2a * (A * A - B * B) / (B * B);
    let ka = 1.0 + u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));
    let kb = u2 / 1024.0 * (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)));
    let c2 = cos_2sm * cos_2sm;
    let d = kb
        * sin_s
        * (cos_2sm
            + kb / 4.0
                * (cos_s * (-1.0 + 2.0 * c2)
                    - kb / 6.0 * cos_2sm * (-3.0 + 4.0 * sin_s * sin_s) * (-3.0 + 4.0 * c2)));
    B * ka * (sigma - d)
}
