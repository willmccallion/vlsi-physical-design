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
mod tests {
    crate::keyword_enum! {
        /// A test enum.
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        enum Mode {
            /// The first.
            #[default]
            Fast = "FAST",
            Slow = "SLOW",
        }
    }

    #[test]
    fn every_value_prints_as_the_keyword_it_parses_from() {
        let round_trip = Mode::ALL.map(|m| Mode::from_keyword(m.as_str()));

        assert_eq!(round_trip, [Some(Mode::Fast), Some(Mode::Slow)]);
    }

    #[test]
    fn a_word_parses_only_as_written() {
        assert_eq!(
            [Mode::from_keyword("fast"), Mode::from_keyword("MEDIUM")],
            [None, None]
        );
    }

    #[test]
    fn variant_attributes_carry_over() {
        assert_eq!(Mode::default(), Mode::Fast);
    }
}
