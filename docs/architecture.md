# Architecture

This document describes how the modules of `stm32-navcore` fit together.
Each claim names the source file or test that implements it.

## Constraints

The crate is `#![no_std]` outside of tests (`src/lib.rs`).
It does not allocate: all buffers are fixed-size arrays or const generics.
The only dependency is `libm`, used for trigonometry and square roots in `src/math.rs` and `src/ekf.rs`.

## Module map

| Module | Source | Role |
| --- | --- | --- |
| `math` | `src/math.rs` | `Mat<R, C>`, `Quaternion`, magnetometer calibration, tilt-compensated yaw |
| `ekf` | `src/ekf.rs` | `NavEkf` error-state filter, scalar Joseph updates, stationary detector |
| `fdir` | `src/fdir.rs` | `DualImuManager` chi-square comparison and primary/secondary switchover |
| `sensors` | `src/sensors/` | Register codecs and scaling for BMI088, ICM-42688-P, DPS310, IST8310 |
| `protocol` | `src/protocol.rs`, `src/protocol/rc_esc.rs` | COBS, CRC-16-CCITT, `NavTelemetry` UART and CAN framing, calibration block, CRSF and DShot codecs |
| `ubx` | `src/ubx.rs` | Streaming UBX parser for NAV-PVT and NAV-RELPOSNED |

## Data flow

```text
IMU samples (sensors::bmi088, sensors::icm42688)
    -> fdir::DualImuManager        selects the healthy IMU
    -> ekf::NavEkf::predict        strapdown propagation and covariance
    -> ekf::NavEkf::update_*       magnetometer yaw, GNSS position and velocity
    -> protocol::NavTelemetry      pack_uart / pack_can
```

The loop in `examples/flight_loop_bench.rs` follows this order.
It calls `NavEkf::predict` for every IMU sample, calls `tilt_compensated_yaw` followed by `NavEkf::update_mag_yaw` for magnetometer samples, and calls `update_gnss_pos` and `update_gnss_vel` for GNSS epochs.
It then packs the telemetry with `pack_uart` and `pack_can` and decodes both back.

`DualImuManager` is a standalone component.
The benchmark does not feed its output into `NavEkf`, so the wiring between FDIR and the filter is left to the integrator.

## Attitude and state estimation

`NavEkf` holds the attitude quaternion, NED position, NED velocity, the gyro Z bias, and the covariance `Mat<8, 8>` (`STATE_DIM` in `src/ekf.rs`).
`NavEkf::predict` rejects a non-positive or non-finite `dt_s` with `EkfError::InvalidInput`.

`Quaternion::integrate_gyro` applies the exact finite-angle rotation and renormalizes the result (`src/math.rs`).
A small-angle branch avoids the division by a near-zero angle.

Each measurement update is a scalar update in Joseph form, `update_joseph_scalar`, and the covariance is symmetrized after each update (`src/ekf.rs`).
A measurement is accepted only when its normalized innovation squared is at or below the gate passed by the caller.
The gate value is an argument to each `update_*` method, so the threshold is chosen by the caller.

`tilt_compensated_yaw` applies the calibration in `calibrate_magnetometer` and then projects the field into the horizontal plane.
It returns `MathError::GimbalLock` when the pitch is too close to +/-90 degrees.

`StationaryDetector` looks at accelerometer variance and gyro magnitude over a window.
When it reports a stationary state, `NavEkf::update_zupt` can apply a zero-velocity update.
