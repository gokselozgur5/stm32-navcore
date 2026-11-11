use core::mem::size_of;
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::Quaternion;
use stm32_navcore::protocol::{CanFrame20B, NavTelemetry};

const TEST_STEPS: usize = 1000;
const DT: f32 = 0.01;
const GOLDEN_FNV1A_DIGEST: u64 = 0xb363_8a42_504f_301a;
