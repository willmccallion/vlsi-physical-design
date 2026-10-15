//! Tokenizer shared by the LEF and DEF parsers.
//!
//! Tokens are whitespace-separated words. A `#` at the start of a word begins
//! a comment that runs to the end of the line, double-quoted strings are one
//! token, and a `;` glued to the end of a word is split off on its own.

use std::path::{Path, PathBuf};
use std::str::FromStr;

/// A failure to read or understand an input file.
#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    /// The file could not be read.
    #[error("cannot read {}: {source}", path.display())]
    Io {
        /// The file.
        path: PathBuf,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
    /// The file's contents are malformed or inconsistent.
    #[error("{}:{line}: {message}", path.display())]
    Syntax {
        /// The file.
        path: PathBuf,
        /// One-based line number.
        line: usize,
        /// What was wrong.
        message: String,
    },
}

/// Reads a whole input file.
pub fn read_file(path: &Path) -> Result<String, ParseError> {
    std::fs::read_to_string(path).map_err(|source| ParseError::Io {
        path: path.to_path_buf(),
        source,
    })
}

/// One word of input and the line it started on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Token<'a> {
    /// The word, without surrounding quotes.
    pub text: &'a str,
    /// One-based line number.
    pub line: usize,
}

/// A stream of tokens over one file.
#[derive(Debug)]
pub struct Lexer<'a> {
    src: &'a str,
    pos: usize,
    line: usize,
    path: &'a Path,
    peeked: Option<Token<'a>>,
    split_semicolon: Option<Token<'a>>,
}

impl<'a> Lexer<'a> {
    /// Creates a lexer over `src`, which was read from `path`.
    pub const fn new(src: &'a str, path: &'a Path) -> Self {
        Self {
            src,
            pos: 0,
            line: 1,
            path,
            peeked: None,
            split_semicolon: None,
        }
    }

    /// Creates a lexer over text that starts at `line` of `path`, such as the
    /// contents of a quoted LEF property value.
    pub const fn starting_at(src: &'a str, path: &'a Path, line: usize) -> Self {
        Self {
            src,
            pos: 0,
            line,
            path,
            peeked: None,
            split_semicolon: None,
        }
    }

    /// The file being read.
    pub const fn path(&self) -> &Path {
        self.path
    }

    /// An error at `line` of this file.
    pub fn error(&self, line: usize, message: impl Into<String>) -> ParseError {
        ParseError::Syntax {
            path: self.path.to_path_buf(),
            line,
            message: message.into(),
        }
    }

    /// The line the next token starts on, or the last line at end of input.
    pub fn line(&mut self) -> usize {
        self.peek().map_or(self.line, |t| t.line)
    }

    /// Returns the next token without consuming it.
    pub fn peek(&mut self) -> Option<Token<'a>> {
        if self.peeked.is_none() {
            self.peeked = self.scan();
        }
        self.peeked
    }

    /// Consumes and returns the next token, or `None` at end of input.
    pub fn next_token(&mut self) -> Option<Token<'a>> {
        self.peeked.take().or_else(|| self.scan())
    }

    /// Consumes the next token, failing at end of input.
    pub fn expect_token(&mut self, what: &str) -> Result<Token<'a>, ParseError> {
        let line = self.line;
        self.next_token()
            .ok_or_else(|| self.error(line, format!("unexpected end of file, expected {what}")))
    }

    /// Consumes the next token and checks that it is `keyword`.
    pub fn expect(&mut self, keyword: &str) -> Result<(), ParseError> {
        let t = self.expect_token(&format!("'{keyword}'"))?;
        if t.text == keyword {
            Ok(())
        } else {
            Err(self.error(t.line, format!("expected '{keyword}', found '{}'", t.text)))
        }
    }

    /// Consumes the next token if it is `keyword`.
    pub fn accept(&mut self, keyword: &str) -> bool {
        if self.peek().is_some_and(|t| t.text == keyword) {
            self.peeked = None;
            true
        } else {
            false
        }
    }

    /// Consumes the next token and parses it as a number.
    pub fn number<T: FromStr>(&mut self, what: &str) -> Result<T, ParseError> {
        let t = self.expect_token(what)?;
        t.text
            .parse()
            .map_err(|_| self.error(t.line, format!("expected {what}, found '{}'", t.text)))
    }

    /// Consumes tokens up to and including the next `;`.
    pub fn skip_statement(&mut self) -> Result<(), ParseError> {
        while self.expect_token("';'")?.text != ";" {}
        Ok(())
    }

    fn scan(&mut self) -> Option<Token<'a>> {
        if let Some(t) = self.split_semicolon.take() {
            return Some(t);
        }
        let bytes = self.src.as_bytes();
        loop {
            while self.pos < bytes.len() && bytes[self.pos].is_ascii_whitespace() {
                if bytes[self.pos] == b'\n' {
                    self.line += 1;
                }
                self.pos += 1;
            }
            if self.pos >= bytes.len() {
                return None;
            }
            if bytes[self.pos] != b'#' {
                break;
            }
            while self.pos < bytes.len() && bytes[self.pos] != b'\n' {
                self.pos += 1;
            }
        }

        let line = self.line;
        if bytes[self.pos] == b'"' {
            let start = self.pos + 1;
            let len = self.src[start..]
                .find('"')
                .unwrap_or(self.src.len() - start);
            self.line += self.src[start..start + len].matches('\n').count();
            self.pos = (start + len + 1).min(self.src.len());
            return Some(Token {
                text: &self.src[start..start + len],
                line,
            });
        }

        let start = self.pos;
        while self.pos < bytes.len() && !bytes[self.pos].is_ascii_whitespace() {
            self.pos += 1;
        }
        let word = &self.src[start..self.pos];
        if word.len() > 1
            && let Some(stripped) = word.strip_suffix(';')
        {
            self.split_semicolon = Some(Token { text: ";", line });
            return Some(Token {
                text: stripped,
                line,
            });
        }
        Some(Token { text: word, line })
    }
}

#[cfg(test)]
mod tests {
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
    fn a_hash_inside_a_word_is_not_a_comment() {
        assert_eq!(words("net#1 ;"), [("net#1", 1), (";", 1)]);
    }
}
