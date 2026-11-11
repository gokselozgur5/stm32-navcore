use core::mem::size_of;
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::Quaternion;
use stm32_navcore::protocol::{CanFrame20B, NavTelemetry};

const TEST_STEPS: usize = 1000;
const DT: f32 = 0.01;
const GOLDEN_FNV1A_DIGEST: u64 = 0xb363_8a42_504f_301a;

#[test]
fn test_memory_budget_and_alignment_invariants() {
    assert!(size_of::<NavEkf>() <= 512, "NavEkf exceeds 512B budget");
    assert!(
        size_of::<CanFrame20B>() <= 16,
        "CanFrame20B exceeds 16B budget"
    );
    assert_eq!(core::mem::align_of::<NavEkf>(), 4);
    assert_eq!(core::mem::align_of::<Quaternion>(), 4);
}
