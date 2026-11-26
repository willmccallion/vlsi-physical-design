//! Enums named by keywords, printed and parsed from one table.

/// Defines an enum whose every variant has a keyword, from one table of
/// `Variant = "KEYWORD"`, so printing and parsing cannot drift apart.
///
/// Besides the enum it gives `ALL`, every value in declaration order;
/// `as_str`, a value's keyword; and `from_keyword`, the value whose keyword
/// is exactly a word.
///
/// ```
/// pare_common::keyword_enum! {
///     #[derive(Clone, Copy, Debug, PartialEq, Eq)]
///     pub enum Side {
///         Left = "LEFT",
///         Right = "RIGHT",
///     }
/// }
///
/// assert_eq!(Side::Right.as_str(), "RIGHT");
/// assert_eq!(Side::from_keyword("LEFT"), Some(Side::Left));
/// assert_eq!(Side::ALL, [Side::Left, Side::Right]);
/// ```
#[macro_export]
macro_rules! keyword_enum {
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident {
            $(
                $(#[$variant_meta:meta])*
                $variant:ident = $keyword:literal
            ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        $vis enum $name {
            $(
                $(#[$variant_meta])*
                $variant,
            )+
        }

        impl $name {
            /// Every value, in declaration order.
            $vis const ALL: [Self; [$($keyword),+].len()] = [$(Self::$variant),+];

            /// The keyword of this value.
            #[must_use]
            $vis const fn as_str(self) -> &'static str {
                match self {
                    $(Self::$variant => $keyword,)+
                }
            }

            /// The value whose keyword is exactly `word`.
            #[must_use]
            $vis fn from_keyword(word: &str) -> Option<Self> {
                match word {
                    $($keyword => Some(Self::$variant),)+
                    _ => None,
                }
            }
        }
    };
}

#[cfg(test)]
mod tests;
