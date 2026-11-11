//! Zero-alloc protocol codec and telemetry framing.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
    BufferTooSmall,
    InvalidCobsEncoding,
    CrcMismatch,
    InvalidCanId,
    InvalidCanDlc,
    NonFiniteFloat,
}

pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

pub fn cobs_encode(src: &[u8], dst: &mut [u8]) -> Result<usize, ProtocolError> {
    if dst.is_empty() {
        return Err(ProtocolError::BufferTooSmall);
    }
    let mut code_idx = 0usize;
    let mut write_idx = 1usize;
    let mut code = 1u8;

    for (i, &byte) in src.iter().enumerate() {
        if byte == 0 {
            dst[code_idx] = code;
            code_idx = write_idx;
            if write_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            write_idx += 1;
            code = 1;
        } else {
            if write_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            dst[write_idx] = byte;
            write_idx += 1;
            code += 1;
            if code == 0xFF {
                dst[code_idx] = code;
                if i + 1 < src.len() {
                    code_idx = write_idx;
                    if write_idx >= dst.len() {
                        return Err(ProtocolError::BufferTooSmall);
                    }
                    write_idx += 1;
                    code = 1;
                } else {
                    code = 0;
                }
            }
        }
    }
    if code != 0 {
        dst[code_idx] = code;
    }
    Ok(write_idx)
}

pub fn cobs_decode(src: &[u8], dst: &mut [u8]) -> Result<usize, ProtocolError> {
    if src.is_empty() {
        return Err(ProtocolError::InvalidCobsEncoding);
    }
    let mut src_idx = 0usize;
    let mut dst_idx = 0usize;

    while src_idx < src.len() {
        let code = src[src_idx];
        if code == 0 {
            return Err(ProtocolError::InvalidCobsEncoding);
        }
        src_idx += 1;
        for _ in 1..code {
            if src_idx >= src.len() {
                return Err(ProtocolError::InvalidCobsEncoding);
            }
            let byte = src[src_idx];
            if byte == 0 {
                return Err(ProtocolError::InvalidCobsEncoding);
            }
            if dst_idx >= dst.len() {
                return Err(ProtocolError::BufferTooSmall);
            }
            dst[dst_idx] = byte;
            dst_idx += 1;
            src_idx += 1;
        }
