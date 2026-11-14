use core::f32;

const PI: f32 = std::f32::consts::PI;
const TWO_PI: f32 = 2. * PI;

pub struct ECEF {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl ECEF {
    pub fn from_wgs84(lat: f32, lon: f32, alt: f32) -> Self {
        let wgs = WGS84::new();
        let (x, y, z) = wgs.lla_to_ecef(lat, lon, alt);

        ECEF { x, y, z }
    }

    pub fn to_lla(&self) -> (f32, f32, f32) {
        let a = 6378137.0;
        let b = 6356752.314245179;
        let e2 = (a.powi(2) - b.powi(2)) / a.powi(2);

