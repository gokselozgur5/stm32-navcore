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

/// One fixed-point update of the longitude difference on the auxiliary sphere.
fn next_lambda(l: f64, t: &[f64; 6]) -> f64 {
    let [sigma, sin_s, cos_s, sin_a, cos2a, cos_2sm] = *t;
    let c = F / 16.0 * cos2a * (4.0 + F * (4.0 - 3.0 * cos2a));
    let inner = cos_2sm + c * cos_s * (-1.0 + 2.0 * cos_2sm * cos_2sm);
    l + (1.0 - c) * F * sin_a * (sigma + c * sin_s * inner)
}

/// Builds the result from the converged auxiliary-sphere terms.
fn finish(u: [f64; 4], sl: f64, cl: f64, x: f64, t: &[f64; 6]) -> Inverse {
    let [su1, cu1, su2, cu2] = u;
    let [sigma, sin_s, cos_s, _, cos2a, cos_2sm] = *t;
    Inverse {
        distance_m: arc_length(cos2a, sigma, sin_s, cos_s, cos_2sm),
        azimuth1: libm::atan2(cu2 * sl, x),
        azimuth2: libm::atan2(cu1 * sl, -su1 * cu2 + cu1 * su2 * cl),
    }
}

/// Solves the inverse problem; inputs in radians. Returns `None` if the
/// iteration fails to converge (nearly antipodal points) or inputs are not finite.
pub fn inverse(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> Option<Inverse> {
    let l = lon2 - lon1;
    let (su1, cu1) = libm::sincos(libm::atan((1.0 - F) * libm::tan(lat1)));
    let (su2, cu2) = libm::sincos(libm::atan((1.0 - F) * libm::tan(lat2)));
    let mut lam = l;
    for _ in 0..200 {
        let (sl, cl) = libm::sincos(lam);
        let x = cu1 * su2 - su1 * cu2 * cl;
        let sin_s = libm::hypot(cu2 * sl, x);
        if sin_s == 0.0 {
            return Some(Inverse { distance_m: 0.0, azimuth1: 0.0, azimuth2: 0.0 });
        }
        let cos_s = su1 * su2 + cu1 * cu2 * cl;
        let sigma = libm::atan2(sin_s, cos_s);
        let sin_a = cu1 * cu2 * sl / sin_s;
        let cos2a = 1.0 - sin_a * sin_a;
        let cos_2sm = if cos2a == 0.0 { 0.0 } else { cos_s - 2.0 * su1 * su2 / cos2a };
        let t = [sigma, sin_s, cos_s, sin_a, cos2a, cos_2sm];
        let next = next_lambda(l, &t);
        if !next.is_finite() {
            return None;
        }
        if libm::fabs(next - lam) < 1e-12 {
            return Some(finish([su1, cu1, su2, cu2], sl, cl, x, &t));
        }
        lam = next;
    }
    None
}
