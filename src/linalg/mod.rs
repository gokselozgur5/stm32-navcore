//! Fixed-size linear algebra on `Mat<R, C>`.

pub mod cholesky;
pub mod spd;
pub mod triangular;

/// Failure modes shared by the factorizations and solvers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinalgError {
    /// A pivot was not strictly positive.
    NotPositiveDefinite,
    /// The input contained NaN or infinity.
    NonFinite,
    /// A triangular system had a zero diagonal entry.
    ZeroPivot,
}
