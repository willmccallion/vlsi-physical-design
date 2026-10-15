//! Readers for LEF design rule statements, shared between native statements
//! and the LEF57/LEF58 `PROPERTY` strings that carry the same syntax.
//!
//! Each reader starts after the statement's keyword and consumes its `;`.
//! Options the model does not represent are errors, never skipped.

use crate::db::parser::lexer::{Lexer, ParseError, Token};
use crate::db::rules::{
    CellEdge, CellEdgeSpacing, CellSide, ConvexCornerSpacing, CutApproach, CutSpacing,
    CutSpacingCondition, EdgeType, Enclosure, EndOfLineSpacing, MinEnclosedArea, MinStep,
    MinimumCut, ParallelEdge, ParallelRunLengthTable, SpacingTable, TwoWidthsRow, TwoWidthsTable,
    WidthRangeSpacing,
};
use crate::geom::dbu::{Dbu, DbuArea, DbuPerMicron};

/// A metal layer `SPACING` statement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RoutingSpacing {
    /// Plain minimum spacing.
    Plain(Dbu),
    /// Spacing for a range of shape widths.
    WidthRange(WidthRangeSpacing),
    /// End-of-line spacing.
    EndOfLine(EndOfLineSpacing),
}

/// Reads rule statements from a lexer at a database resolution.
pub(crate) struct RuleReader<'l, 'a> {
    lx: &'l mut Lexer<'a>,
    dbu: DbuPerMicron,
}

impl<'l, 'a> RuleReader<'l, 'a> {
    pub(crate) const fn new(lx: &'l mut Lexer<'a>, dbu: DbuPerMicron) -> Self {
        Self { lx, dbu }
    }

    pub(crate) fn microns(&mut self, what: &str) -> Result<Dbu, ParseError> {
        let line = self.lx.line();
        let value: f64 = self.lx.number(what)?;
        self.dbu
            .to_dbu(value)
            .map_err(|e| self.lx.error(line, e.to_string()))
    }

    pub(crate) fn area(&mut self, what: &str) -> Result<DbuArea, ParseError> {
        let line = self.lx.line();
        let value: f64 = self.lx.number(what)?;
        self.dbu
            .to_dbu_area(value)
            .map_err(|e| self.lx.error(line, e.to_string()))
    }

    /// Consumes the next statement keyword of a property string, which must
    /// be `keyword`, or returns `None` at the end of the string.
    pub(crate) fn next_keyword(&mut self, keyword: &str) -> Result<Option<Token<'a>>, ParseError> {
        let Some(t) = self.lx.next_token() else {
            return Ok(None);
        };
        if t.text == keyword {
            Ok(Some(t))
        } else {
            Err(self
                .lx
                .error(t.line, format!("expected '{keyword}', found '{}'", t.text)))
        }
    }

    fn count(&mut self, what: &str) -> Result<u32, ParseError> {
        self.lx.number(what)
    }

    fn unsupported(&mut self, rule: &str) -> ParseError {
        let line = self.lx.line();
        let found = self.lx.peek().map_or("end of input", |t| t.text);
        self.lx
            .error(line, format!("unsupported {rule} option '{found}'"))
    }

    fn end(&mut self, rule: &str) -> Result<(), ParseError> {
        if self.lx.accept(";") {
            Ok(())
        } else {
            Err(self.unsupported(rule))
        }
    }

    /// `SPACING s [RANGE min max | ENDOFLINE w WITHIN d [PARALLELEDGE s WITHIN
    /// d [TWOEDGES]]] ;`
    pub(crate) fn routing_spacing(&mut self) -> Result<RoutingSpacing, ParseError> {
        let spacing = self.microns("spacing")?;
        if self.lx.accept(";") {
            return Ok(RoutingSpacing::Plain(spacing));
        }
        if self.lx.accept("RANGE") {
            let min_width = self.microns("range minimum width")?;
            let max_width = self.microns("range maximum width")?;
            self.end("SPACING RANGE")?;
            return Ok(RoutingSpacing::WidthRange(WidthRangeSpacing {
                spacing,
                min_width,
                max_width,
            }));
        }
        if !self.lx.accept("ENDOFLINE") {
            return Err(self.unsupported("metal SPACING"));
        }
        let eol_width = self.microns("end-of-line width")?;
        self.lx.expect("WITHIN")?;
        let within = self.microns("end-of-line within")?;
        let parallel_edge = if self.lx.accept("PARALLELEDGE") {
            let spacing = self.microns("parallel edge spacing")?;
            self.lx.expect("WITHIN")?;
            let within = self.microns("parallel edge within")?;
            let two_edges = self.lx.accept("TWOEDGES");
            Some(ParallelEdge {
                spacing,
                within,
                two_edges,
            })
        } else {
            None
        };
        self.end("ENDOFLINE")?;
        Ok(RoutingSpacing::EndOfLine(EndOfLineSpacing {
            spacing,
            eol_width,
            within,
            parallel_edge,
        }))
    }

    /// `SPACING s [CENTERTOCENTER] [SAMENET] [ADJACENTCUTS n WITHIN d |
    /// PARALLELOVERLAP] ;`
    pub(crate) fn cut_spacing(&mut self) -> Result<CutSpacing, ParseError> {
        let spacing = self.microns("cut spacing")?;
        let mut center_to_center = false;
        let mut same_net = false;
        loop {
            if self.lx.accept("CENTERTOCENTER") {
                center_to_center = true;
            } else if self.lx.accept("SAMENET") {
                same_net = true;
            } else {
                break;
            }
        }
        let condition = if self.lx.accept("ADJACENTCUTS") {
            let cuts = self.count("adjacent cut count")?;
            self.lx.expect("WITHIN")?;
            let within = self.microns("adjacent cut distance")?;
            CutSpacingCondition::AdjacentCuts { cuts, within }
        } else if self.lx.accept("PARALLELOVERLAP") {
            CutSpacingCondition::ParallelOverlap
        } else {
            CutSpacingCondition::Always
        };
        self.end("cut SPACING")?;
        Ok(CutSpacing {
            spacing,
            center_to_center,
            same_net,
            condition,
        })
    }

    /// `ENCLOSURE [ABOVE|BELOW] o1 o2 [WIDTH w | LENGTH l] ;`
    pub(crate) fn enclosure(&mut self) -> Result<Enclosure, ParseError> {
        let side = self.approach("ABOVE", "BELOW");
        let overhang_1 = self.microns("overhang")?;
        let overhang_2 = self.microns("overhang")?;
        let mut min_width = None;
        let mut min_length = None;
        if self.lx.accept("WIDTH") {
            min_width = Some(self.microns("enclosure width")?);
        } else if self.lx.accept("LENGTH") {
            min_length = Some(self.microns("enclosure length")?);
        }
        self.end("ENCLOSURE")?;
        Ok(Enclosure {
            side,
            overhang_1,
            overhang_2,
            min_width,
            min_length,
        })
    }

    fn approach(&mut self, above: &str, below: &str) -> Option<CutApproach> {
        if self.lx.accept(above) {
            Some(CutApproach::Above)
        } else if self.lx.accept(below) {
            Some(CutApproach::Below)
        } else {
            None
        }
    }

    /// `MINIMUMCUT n WIDTH w [WITHIN d] [FROMABOVE|FROMBELOW] [LENGTH l WITHIN d] ;`
    pub(crate) fn minimum_cut(&mut self) -> Result<MinimumCut, ParseError> {
        let cuts = self.count("cut count")?;
        self.lx.expect("WIDTH")?;
        let width = self.microns("minimum cut width")?;
        let within = if self.lx.accept("WITHIN") {
            Some(self.microns("minimum cut within")?)
        } else {
            None
        };
        let from = self.approach("FROMABOVE", "FROMBELOW");
        let length = if self.lx.accept("LENGTH") {
            let length = self.microns("minimum cut length")?;
            self.lx.expect("WITHIN")?;
            Some((length, self.microns("minimum cut distance")?))
        } else {
            None
        };
        self.end("MINIMUMCUT")?;
        Ok(MinimumCut {
            cuts,
            width,
            within,
            from,
            length,
        })
    }

    /// `MINSTEP l [MAXEDGES n] ;`
    pub(crate) fn min_step(&mut self) -> Result<MinStep, ParseError> {
        let length = self.microns("minimum step")?;
        let max_edges = if self.lx.accept("MAXEDGES") {
            Some(self.count("maximum edges")?)
        } else {
            None
        };
        self.end("MINSTEP")?;
        Ok(MinStep { length, max_edges })
    }

    /// `MINENCLOSEDAREA a [WIDTH w] ;`
    pub(crate) fn min_enclosed_area(&mut self) -> Result<MinEnclosedArea, ParseError> {
        let area = self.area("minimum enclosed area")?;
        let width = if self.lx.accept("WIDTH") {
            Some(self.microns("enclosed area width")?)
        } else {
            None
        };
        self.end("MINENCLOSEDAREA")?;
        Ok(MinEnclosedArea { area, width })
    }

    /// `CONVEXCORNER [EXCEPTEOL w] WIDTH w SPACING s ... ;`, after `CORNERSPACING`.
    pub(crate) fn corner_spacing(&mut self) -> Result<ConvexCornerSpacing, ParseError> {
        if !self.lx.accept("CONVEXCORNER") {
            return Err(self.unsupported("CORNERSPACING"));
        }
        let except_eol = if self.lx.accept("EXCEPTEOL") {
            Some(self.microns("end-of-line width")?)
        } else {
            None
        };
        let mut spacing_by_width = Vec::new();
        while self.lx.accept("WIDTH") {
            let width = self.microns("corner width")?;
            self.lx.expect("SPACING")?;
            spacing_by_width.push((width, self.microns("corner spacing")?));
        }
        self.end("CORNERSPACING")?;
        Ok(ConvexCornerSpacing {
            except_eol,
            spacing_by_width,
        })
    }

    /// `PARALLELRUNLENGTH ...` or `TWOWIDTHS ...`, after `SPACINGTABLE`.
    pub(crate) fn spacing_table(&mut self) -> Result<SpacingTable, ParseError> {
        if self.lx.accept("PARALLELRUNLENGTH") {
            return self.parallel_run_length_table();
        }
        if self.lx.accept("TWOWIDTHS") {
            return self.two_widths_table();
        }
        Err(self.unsupported("SPACINGTABLE"))
    }

    fn parallel_run_length_table(&mut self) -> Result<SpacingTable, ParseError> {
        let line = self.lx.line();
        let mut run_lengths = Vec::new();
        while !self.lx.accept("WIDTH") {
            run_lengths.push(self.microns("parallel run length")?);
        }
        if run_lengths.is_empty() {
            return Err(self.lx.error(line, "spacing table has no run lengths"));
        }
        let mut rows = Vec::new();
        loop {
            let width = self.microns("width")?;
            let spacings = (0..run_lengths.len())
                .map(|_| self.microns("spacing"))
                .collect::<Result<Vec<Dbu>, ParseError>>()?;
            rows.push((width, spacings));
            if self.lx.accept(";") {
                break;
            }
            self.lx.expect("WIDTH")?;
        }
        Ok(SpacingTable::ParallelRunLength(ParallelRunLengthTable {
            run_lengths,
            rows,
        }))
    }

    fn two_widths_table(&mut self) -> Result<SpacingTable, ParseError> {
        let line = self.lx.line();
        let mut rows = Vec::new();
        while self.lx.accept("WIDTH") {
            let width = self.microns("width")?;
            let run_length = if self.lx.accept("PRL") {
                Some(self.microns("parallel run length")?)
            } else {
                None
            };
            let mut spacings = Vec::new();
            while self
                .lx
                .peek()
                .is_some_and(|t| t.text != "WIDTH" && t.text != ";")
            {
                spacings.push(self.microns("spacing")?);
            }
            rows.push(TwoWidthsRow {
                width,
                run_length,
                spacings,
            });
        }
        self.lx.expect(";")?;
        if rows.is_empty() || rows.iter().any(|r| r.spacings.len() != rows.len()) {
            return Err(self.lx.error(
                line,
                "TWOWIDTHS table needs one spacing per width in every row",
            ));
        }
        Ok(SpacingTable::TwoWidths(TwoWidthsTable { rows }))
    }

    /// `{LEFT|RIGHT|BOTH|TOP|BOTTOM} type ;`, after `EDGETYPE`.
    pub(crate) fn cell_edge(&mut self) -> Result<CellEdge, ParseError> {
        let t = self.lx.expect_token("cell side")?;
        let side = match t.text {
            "LEFT" => CellSide::Left,
            "RIGHT" => CellSide::Right,
            "BOTH" => CellSide::Both,
            "TOP" => CellSide::Top,
            "BOTTOM" => CellSide::Bottom,
            other => {
                return Err(self
                    .lx
                    .error(t.line, format!("unknown cell side '{other}'")));
            }
        };
        let edge_type = EdgeType(self.lx.expect_token("edge type")?.text.to_string());
        self.end("EDGETYPE")?;
        Ok(CellEdge { side, edge_type })
    }

    /// `EDGETYPE a b spacing ... ;`, after `CELLEDGESPACINGTABLE`.
    pub(crate) fn cell_edge_spacing_table(&mut self) -> Result<Vec<CellEdgeSpacing>, ParseError> {
        let mut table = Vec::new();
        while self.lx.accept("EDGETYPE") {
            let first = EdgeType(self.lx.expect_token("edge type")?.text.to_string());
            let second = EdgeType(self.lx.expect_token("edge type")?.text.to_string());
            let spacing = self.microns("cell edge spacing")?;
            table.push(CellEdgeSpacing {
                first,
                second,
                spacing,
            });
        }
        self.end("CELLEDGESPACINGTABLE")?;
        Ok(table)
    }
}
