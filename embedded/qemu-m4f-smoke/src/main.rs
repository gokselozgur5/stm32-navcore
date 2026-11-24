#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_halt as _;
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::filters::MedianFilter;
use stm32_navcore::madgwick::MadgwickAhrs;
use stm32_navcore::protocol::NavTelemetry;

#[entry]
fn main() -> ! {
    let mut ekf = NavEkf::new();
    let imu_gyro = [0.01f32, -0.01, 0.005];
    let imu_accel = [0.0f32, 0.0, -9.80665];
    ekf.predict(imu_gyro, imu_accel, 0.01).unwrap();

    let mut ahrs = MadgwickAhrs::new(0.041).unwrap();
    let quat = ahrs.update_imu(imu_gyro, imu_accel, 0.01).unwrap();
    let _ = quat.w;

    let mut median = MedianFilter::<5>::new();
    let med_val = median.update(42.0);
    assert_eq!(med_val, 42.0);

    let telem = NavTelemetry {
        timestamp_ms: 1000,
        pos_ned_m: [1.0, 2.0, -3.0],
        vel_ned_mps: [0.1, 0.2, -0.3],
        euler_rad: [0.01, 0.02, 0.03],
        status: 1,
    };
    let mut uart_buf = [0u8; NavTelemetry::MAX_UART_FRAME_LEN];
    let packed_len = telem.pack_uart(&mut uart_buf).unwrap();
    let unpacked = NavTelemetry::unpack_uart(&uart_buf[..packed_len]).unwrap();
    assert_eq!(unpacked.timestamp_ms, 1000);

    hprintln!("STM32-NAVCORE CORTEX-M4F QEMU SMOKE TEST PASSED");
    debug::exit(debug::EXIT_SUCCESS);

    loop {}
}
