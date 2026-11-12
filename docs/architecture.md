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
