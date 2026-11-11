#![cfg_attr(not(test), no_std)]

pub mod math;
pub mod ekf;
pub mod protocol;

#[cfg(test)]
mod tests {
    #[test]
    fn basic() {
        // placeholder test
        assert_eq!(2 + 2, 4);
    }
}
