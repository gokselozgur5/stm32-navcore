//! Geodetic, ECEF, and local tangent plane NED coordinate transforms.
//!
//! Provides deterministic, zero-allocation transformations using WGS-84.

/// Errors occurring during coordinate conversion or validation.
pub use crate::geo::GeoError as EcefNedError;

/// Geodetic coordinates represented by latitude, longitude, and height.
pub use crate::geo::Lla as GeodeticPos;

/// Earth-Centered, Earth-Fixed Cartesian coordinates in meters.
pub use crate::geo::Ecef as EcefPos;

/// Local Tangent Plane North-East-Down (NED) coordinates in meters.
pub use crate::geo::Ned as NedPos;

pub use crate::geo::{WGS84_A, WGS84_B, WGS84_E2, WGS84_F, WGS84_INV_F};

/// Transform geodetic coordinates to ECEF Cartesian coordinates.
#[inline]
pub fn geodetic_to_ecef(geo: &GeodeticPos) -> EcefPos {
    crate::geo::lla_to_ecef(geo)
}

/// Transform ECEF Cartesian coordinates to geodetic coordinates via Bowring's method.
#[inline]
pub fn ecef_to_geodetic(ecef: &EcefPos) -> Result<GeodeticPos, EcefNedError> {
    crate::geo::ecef_to_lla(ecef)
}

/// Local tangent plane reference frame caching trigonometric components.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NedFrame {
    pub origin_geo: GeodeticPos,
    pub origin_ecef: EcefPos,
    sin_lat: f64,
    cos_lat: f64,
    sin_lon: f64,
    cos_lon: f64,
}

impl NedFrame {
    /// Initialize local tangent plane frame at a given geodetic origin.
    pub fn from_geodetic(origin: &GeodeticPos) -> Self {
        let origin_ecef = geodetic_to_ecef(origin);
        Self {
            origin_geo: *origin,
            origin_ecef,
            sin_lat: libm::sin(origin.lat_rad),
            cos_lat: libm::cos(origin.lat_rad),
            sin_lon: libm::sin(origin.lon_rad),
            cos_lon: libm::cos(origin.lon_rad),
        }
    }
}

impl NedFrame {
    /// Convert an ECEF point into NED coordinates relative to this frame.
    pub fn ecef_to_ned(&self, point: &EcefPos) -> Result<NedPos, EcefNedError> {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(EcefNedError::NonFinite);
        }
        let dx = point.x - self.origin_ecef.x;
        let dy = point.y - self.origin_ecef.y;
        let dz = point.z - self.origin_ecef.z;

        let north = -self.sin_lat * self.cos_lon * dx - self.sin_lat * self.sin_lon * dy
            + self.cos_lat * dz;
        let east = -self.sin_lon * dx + self.cos_lon * dy;
        let down = -self.cos_lat * self.cos_lon * dx
            - self.cos_lat * self.sin_lon * dy
            - self.sin_lat * dz;

        Ok(NedPos::new(north, east, down))
    }
}

impl NedFrame {
    /// Convert NED coordinates back into ECEF coordinates.
    pub fn ned_to_ecef(&self, ned: &NedPos) -> Result<EcefPos, EcefNedError> {
        if !ned.north.is_finite() || !ned.east.is_finite() || !ned.down.is_finite() {
            return Err(EcefNedError::NonFinite);
        }
        let x = self.origin_ecef.x
            - self.sin_lat * self.cos_lon * ned.north
            - self.sin_lon * ned.east
            - self.cos_lat * self.cos_lon * ned.down;
        let y = self.origin_ecef.y - self.sin_lat * self.sin_lon * ned.north
            + self.cos_lon * ned.east
            - self.cos_lat * self.sin_lon * ned.down;
        let z = self.origin_ecef.z + self.cos_lat * ned.north - self.sin_lat * ned.down;

        Ok(EcefPos::new(x, y, z))
    }
}

/// Transform ECEF coordinates to NED coordinates relative to a geodetic reference point.
pub fn ecef_to_ned(point: &EcefPos, ref_geo: &GeodeticPos) -> Result<NedPos, EcefNedError> {
    let frame = NedFrame::from_geodetic(ref_geo);
    frame.ecef_to_ned(point)
}

/// Transform NED coordinates to ECEF coordinates relative to a geodetic reference point.
pub fn ned_to_ecef(ned: &NedPos, ref_geo: &GeodeticPos) -> Result<EcefPos, EcefNedError> {
    let frame = NedFrame::from_geodetic(ref_geo);
    frame.ned_to_ecef(ned)
}

impl NedFrame {
    /// Convert a geodetic point directly into NED coordinates.
    pub fn geodetic_to_ned(&self, point: &GeodeticPos) -> Result<NedPos, EcefNedError> {
        let ecef = geodetic_to_ecef(point);
        self.ecef_to_ned(&ecef)
    }

    /// Convert NED coordinates directly into geodetic coordinates.
    pub fn ned_to_geodetic(&self, ned: &NedPos) -> Result<GeodeticPos, EcefNedError> {
        let ecef = self.ned_to_ecef(ned)?;
        ecef_to_geodetic(&ecef)
    }
}

/// Direct transform from geodetic point to NED given a geodetic reference point.
pub fn geodetic_to_ned(point: &GeodeticPos, ref_geo: &GeodeticPos) -> Result<NedPos, EcefNedError> {
    let frame = NedFrame::from_geodetic(ref_geo);
    frame.geodetic_to_ned(point)
}

/// Direct transform from NED to geodetic given a geodetic reference point.
pub fn ned_to_geodetic(ned: &NedPos, ref_geo: &GeodeticPos) -> Result<GeodeticPos, EcefNedError> {
    let frame = NedFrame::from_geodetic(ref_geo);
    frame.ned_to_geodetic(ned)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_input_validation_and_errors() {
        assert_eq!(
            GeodeticPos::new(f64::NAN, 0.0, 0.0),
            Err(EcefNedError::NonFinite)
        );
        assert_eq!(
            GeodeticPos::new(2.0, 0.0, 0.0),
            Err(EcefNedError::InvalidLatitude)
        );
        assert_eq!(
            GeodeticPos::new(0.0, 4.0, 0.0),
            Err(EcefNedError::InvalidLongitude)
        );
        let bad_ecef = EcefPos::new(f64::NAN, 0.0, 0.0);
        assert_eq!(ecef_to_geodetic(&bad_ecef), Err(EcefNedError::NonFinite));
        let center = EcefPos::new(0.0, 0.0, 0.0);
        let center_geo = ecef_to_geodetic(&center).unwrap();
        assert_eq!(center_geo.lat_deg(), 90.0);
    }

    #[test]
    fn test_known_points_geodetic_to_ecef() {
        let equator = GeodeticPos::from_degrees(0.0, 0.0, 0.0).unwrap();
        let ecef_eq = geodetic_to_ecef(&equator);
        assert!((ecef_eq.x - WGS84_A).abs() < 1e-3);
        assert!(ecef_eq.y.abs() < 1e-3);
        assert!(ecef_eq.z.abs() < 1e-3);

        let pole = GeodeticPos::from_degrees(90.0, 0.0, 0.0).unwrap();
        let ecef_pole = geodetic_to_ecef(&pole);
        assert!(ecef_pole.x.abs() < 1e-3);
        assert!(ecef_pole.y.abs() < 1e-3);
        assert!((ecef_pole.z - WGS84_B).abs() < 1e-3);
    }

    #[test]
    fn test_geodetic_ecef_roundtrip() {
        let original = GeodeticPos::from_degrees(41.0082, 28.9784, 150.0).unwrap();
        let ecef = geodetic_to_ecef(&original);
        let recovered = ecef_to_geodetic(&ecef).unwrap();
        assert!((recovered.lat_deg() - original.lat_deg()).abs() < 1e-7);
        assert!((recovered.lon_deg() - original.lon_deg()).abs() < 1e-7);
        assert!((recovered.alt_m - original.alt_m).abs() < 1e-3);
    }

    #[test]
    fn test_ned_local_tangent_plane_roundtrip() {
        let origin = GeodeticPos::from_degrees(41.0, 29.0, 100.0).unwrap();
        let frame = NedFrame::from_geodetic(&origin);
        let origin_ned = frame.geodetic_to_ned(&origin).unwrap();
        assert!(origin_ned.north.abs() < 1e-4);
        assert!(origin_ned.east.abs() < 1e-4);
        assert!(origin_ned.down.abs() < 1e-4);

        let target_ned = NedPos::new(1000.0, -500.0, -50.0);
        let ecef_pt = frame.ned_to_ecef(&target_ned).unwrap();
        let recovered_ned = frame.ecef_to_ned(&ecef_pt).unwrap();
        assert!((recovered_ned.north - target_ned.north).abs() < 1e-4);
        assert!((recovered_ned.east - target_ned.east).abs() < 1e-4);
        assert!((recovered_ned.down - target_ned.down).abs() < 1e-4);
    }

    #[test]
    fn test_free_functions_ned_conversions() {
        let origin = GeodeticPos::from_degrees(37.7749, -122.4194, 10.0).unwrap();
        let target = GeodeticPos::from_degrees(37.7750, -122.4190, 15.0).unwrap();
        let ned = geodetic_to_ned(&target, &origin).unwrap();
        let recovered = ned_to_geodetic(&ned, &origin).unwrap();
        assert!((recovered.lat_deg() - target.lat_deg()).abs() < 1e-7);
        assert!((recovered.lon_deg() - target.lon_deg()).abs() < 1e-7);
        assert!((recovered.alt_m - target.alt_m).abs() < 1e-3);
    }
}
