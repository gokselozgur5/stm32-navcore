use stm32_navcore::protocol::{cobs_decode, cobs_encode, ProtocolError};

fn enc(src: &[u8]) -> Vec<u8> {
    let mut d = vec![0u8; src.len() + src.len() / 254 + 2];
    let n = cobs_encode(src, &mut d).unwrap();
    d.truncate(n);
    d
}

#[test]
fn small_vectors() {
    assert_eq!(enc(&[]), [1]);
    assert_eq!(enc(&[0]), [1, 1]);
    assert_eq!(enc(&[0, 0]), [1, 1, 1]);
    assert_eq!(enc(&[1, 0, 2]), [2, 1, 2, 2]);
}

#[test]
fn run_of_254_nonzero_bytes() {
    let e = enc(&[7u8; 254]);
    assert_eq!(e.len(), 255);
    assert_eq!(e[0], 0xFF);
    assert_eq!(enc(&[7u8; 253])[0], 0xFE);
}

#[test]
fn run_of_255_nonzero_bytes_splits() {
    let e = enc(&[7u8; 255]);
    assert_eq!(e.len(), 257);
    assert_eq!((e[0], e[255], e[256]), (0xFF, 2, 7));
}

#[test]
fn roundtrip_all_lengths() {
    for len in 0..=300usize {
        for fill in [0u8, 1, 0xAA] {
            let src: Vec<u8> = (0..len).map(|i| if fill == 0xAA && i % 7 == 0 { 0 } else { fill }).collect();
            let e = enc(&src);
            assert!(!e.contains(&0));
            let mut d = vec![0u8; len + 1];
            let n = cobs_decode(&e, &mut d).unwrap();
            assert_eq!(&d[..n], &src[..]);
        }
    }
}
