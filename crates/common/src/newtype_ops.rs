//! Arithmetic for one-field numeric newtypes.

/// Implements the listed operator traits on a one-field newtype, each
/// acting on its field.
///
/// `Add`, `AddAssign`, `Sub`, `SubAssign`, `Neg` and `Sum` work between
/// values of the type; `Add<T>`, `Sub<T>`, `Mul<T>` and `Div<T>` take a
/// bare scalar `T` the field accepts.
///
/// ```
/// #[derive(Clone, Copy, Debug, PartialEq)]
/// struct Steps(i64);
/// pare_common::newtype_ops!(Steps: Add, Sub, Mul<i64>);
///
/// assert_eq!((Steps(3) + Steps(4)) * 2, Steps(14));
/// ```
#[macro_export]
macro_rules! newtype_ops {
    ($name:ident: $($ops:tt)+) => {
        $crate::newtype_ops!(@each $name; $($ops)+);
    };
    (@each $name:ident;) => {};
    (@each $name:ident; , $($rest:tt)*) => {
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Add<$scalar:ty> $($rest:tt)*) => {
        impl ::core::ops::Add<$scalar> for $name {
            type Output = Self;
            fn add(self, rhs: $scalar) -> Self {
                Self(self.0 + rhs)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Sub<$scalar:ty> $($rest:tt)*) => {
        impl ::core::ops::Sub<$scalar> for $name {
            type Output = Self;
            fn sub(self, rhs: $scalar) -> Self {
                Self(self.0 - rhs)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Mul<$scalar:ty> $($rest:tt)*) => {
        impl ::core::ops::Mul<$scalar> for $name {
            type Output = Self;
            fn mul(self, rhs: $scalar) -> Self {
                Self(self.0 * rhs)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Div<$scalar:ty> $($rest:tt)*) => {
        impl ::core::ops::Div<$scalar> for $name {
            type Output = Self;
            fn div(self, rhs: $scalar) -> Self {
                Self(self.0 / rhs)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Add $($rest:tt)*) => {
        impl ::core::ops::Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; AddAssign $($rest:tt)*) => {
        impl ::core::ops::AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Sub $($rest:tt)*) => {
        impl ::core::ops::Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; SubAssign $($rest:tt)*) => {
        impl ::core::ops::SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Neg $($rest:tt)*) => {
        impl ::core::ops::Neg for $name {
            type Output = Self;
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
    (@each $name:ident; Sum $($rest:tt)*) => {
        impl ::core::iter::Sum for $name {
            fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
                Self(iter.map(|v| v.0).sum())
            }
        }
        $crate::newtype_ops!(@each $name; $($rest)*);
    };
}

#[cfg(test)]
mod tests;
