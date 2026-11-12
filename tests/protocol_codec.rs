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
