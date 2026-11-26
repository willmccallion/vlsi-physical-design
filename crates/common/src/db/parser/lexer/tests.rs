use super::*;

fn words(src: &str) -> Vec<(&str, usize)> {
    let mut lexer = Lexer::new(src, Path::new("t"));
    std::iter::from_fn(|| lexer.next_token())
        .map(|t| (t.text, t.line))
        .collect()
}

#[test]
fn comments_run_to_the_end_of_the_line() {
    assert_eq!(
        words("WIDTH 0.17 ; # LI 1\nAREA 1 ;"),
        [
            ("WIDTH", 1),
            ("0.17", 1),
            (";", 1),
            ("AREA", 2),
            ("1", 2),
            (";", 2)
        ]
    );
}

#[test]
fn a_glued_semicolon_becomes_its_own_token() {
    assert_eq!(
        words("SPACING 0.065;"),
        [("SPACING", 1), ("0.065", 1), (";", 1)]
    );
}

#[test]
fn quoted_strings_are_single_tokens() {
    assert_eq!(
        words("PROPERTY LEF58_TYPE \"TYPE NWELL ;\" ;"),
        [
            ("PROPERTY", 1),
            ("LEF58_TYPE", 1),
            ("TYPE NWELL ;", 1),
            (";", 1)
        ]
    );
}

#[test]
fn an_unterminated_quoted_string_is_an_error_at_its_line() {
    let mut lexer = Lexer::new("A\n\"B C\nD ;", Path::new("t"));

    let first = lexer.expect_token("word").map(|t| t.text);
    let second = lexer.expect_token("word").map(|t| t.text);

    assert_eq!(first.ok(), Some("A"));
    match second {
        Err(ParseError::Syntax { line, message, .. }) => {
            assert_eq!((line, message.as_str()), (2, "unterminated string"));
        }
        other => panic!("expected a syntax error, got {other:?}"),
    }
}

#[test]
fn a_hash_inside_a_word_is_not_a_comment() {
    assert_eq!(words("net#1 ;"), [("net#1", 1), (";", 1)]);
}
