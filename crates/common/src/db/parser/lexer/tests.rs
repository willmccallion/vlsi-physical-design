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

#[test]
fn a_comment_may_end_the_source() {
    assert_eq!(words("END ;\n# last"), [("END", 1), (";", 1)]);
}

#[test]
fn lines_count_on_past_a_quoted_string_spanning_lines() {
    assert_eq!(words("\"a\nb\" next"), [("a\nb", 1), ("next", 2)]);
}

#[test]
fn skipping_a_statement_stops_after_its_semicolon() {
    let mut lexer = Lexer::new("A B C ; D", Path::new("t"));
    let _ = lexer.next_token();

    lexer.skip_statement().unwrap();

    assert_eq!(lexer.next_token().map(|t| t.text), Some("D"));
}
