//! Area recovery: trading buffers back while the slack stays within the relaxation.

use super::{
    AreaRecoveryFailed, Frontier, JunctionSides, Rebuffer, insert_assured, prune_cap_against_area,
    wire, with_arrivals,
};
use crate::bnet::{Bnet, BnetPtr, Fixed, Kind};
use crate::fuzzy;
use crate::resizer::Resizer;
use crate::sta::Sta;
use pare_common::geom::dbu::Dbu;
use pare_common::units::CellArea;
use std::rc::Rc;

impl Rebuffer {
    /// The options worth keeping once every buffer size is offered on top
    /// of `options`, for timing or, given `area`, for area.
    pub(super) fn buffered_frontier(
        &self,
        resizer: &Resizer<'_, Sta>,
        options: &[BnetPtr],
        next_segment: Dbu,
        area: Option<(Fixed, &Bnet)>,
    ) -> Frontier {
        let mut frontier = Frontier::new(area);
        let Some(strong) = self.sizes.last().copied() else {
            return frontier;
        };
        let mut at = 0usize;
        for size in &self.sizes {
            while at < options.len() && options[at].cap <= size.input_cap {
                frontier.offer(resizer, &strong, &options[at], next_segment);
                at += 1;
            }
            let start = if frontier.kept.is_empty() && at == options.len() && at > 0 {
                at - 1
            } else {
                at
            };
            if let Some((option, delay)) = frontier.best_buffered(resizer, size, &options[start..])
            {
                frontier.push(Bnet::buffer(option.at, size, option, delay));
            }
        }
        for option in &options[at..] {
            frontier.offer(resizer, &strong, option, next_segment);
        }
        frontier
    }

    /// Trades buffers for area while every node keeps at least a share
    /// `alpha` of the way from its present slack to the slack target, as
    /// `recoverArea`.
    pub(super) fn recover_area(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        root: &BnetPtr,
        slack_target: Fixed,
        alpha: f64,
    ) -> Result<BnetPtr, AreaRecoveryFailed> {
        let (correction, target) = if root.slack_edges.is_some() {
            (
                Fixed::from_seconds(self.driver_timing(resizer, root).correction),
                slack_target,
            )
        } else {
            (Fixed::ZERO, -Fixed::INF)
        };
        let root = with_arrivals(root, -correction);
        let options = self.area_options(resizer, &root, target, alpha, Dbu::ZERO)?;
        let mut best_slack: Option<(Fixed, BnetPtr)> = None;
        let mut best_area: Option<(CellArea, BnetPtr)> = None;
        for option in options {
            let Some(slack) = self.evaluate_option(resizer, &option) else {
                continue;
            };
            let unconstrained = option.slack_edges.is_none();
            if best_slack.is_none()
                || unconstrained
                || best_slack.as_ref().is_some_and(|(s, _)| slack > *s)
            {
                best_slack = Some((slack, Rc::clone(&option)));
            }
            if (slack >= target || unconstrained)
                && best_area
                    .as_ref()
                    .is_none_or(|(a, _)| fuzzy::less(option.area, *a))
            {
                best_area = Some((option.area, Rc::clone(&option)));
            }
        }
        best_area
            .map(|(_, o)| o)
            .or_else(|| best_slack.map(|(_, o)| o))
            .ok_or(AreaRecoveryFailed)
    }

    fn area_options(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &BnetPtr,
        slack_target: Fixed,
        alpha: f64,
        upstream_wire: Dbu,
    ) -> Result<Vec<BnetPtr>, AreaRecoveryFailed> {
        Ok(match &node.kind {
            Kind::Load { .. } => vec![Rc::clone(node)],
            Kind::Via {
                to,
                layer,
                to_layer,
            } => self
                .area_options(resizer, to, slack_target, alpha, upstream_wire)?
                .into_iter()
                .map(|o| Bnet::via(node.at, *layer, *to_layer, o, &resizer.tree_rc()))
                .collect(),
            Kind::Wire { .. } | Kind::Buffer { .. } => {
                let inner = match &node.kind {
                    Kind::Buffer { to, .. } => Rc::clone(to),
                    _ => Rc::clone(node),
                };
                let mut options = if let Kind::Wire { to, layer } = &inner.kind {
                    let options =
                        self.area_options(resizer, to, slack_target, alpha, inner.length())?;
                    options
                        .into_iter()
                        .map(|o| wire(resizer, inner.at, *layer, o))
                        .collect()
                } else {
                    self.area_options(resizer, &inner, slack_target, alpha, Dbu::ZERO)?
                };
                let threshold = Fixed::lerp(node.slack, slack_target + node.arrival_delay, alpha);
                self.insert_area_options(resizer, &mut options, upstream_wire, (threshold, node))?;
                options
            }
            Kind::Junction { left, right } => {
                let left_options =
                    self.area_options(resizer, left, slack_target, alpha, upstream_wire)?;
                let right_options =
                    self.area_options(resizer, right, slack_target, alpha, upstream_wire)?;
                let threshold = Fixed::lerp(node.slack, slack_target + node.arrival_delay, alpha);
                let sides = JunctionSides {
                    left: &left_options,
                    right: &right_options,
                };
                self.paired_area_options(node, sides, threshold)
            }
        })
    }

    /// Every pairing of the junction `node`'s side options within the
    /// fanout limit, pruned to those whose capacitance beats every
    /// smaller one's, with the first pairing that fits `threshold` put
    /// back where none left does.
    fn paired_area_options(
        &self,
        node: &Bnet,
        sides: JunctionSides<'_>,
        threshold: Fixed,
    ) -> Vec<BnetPtr> {
        let envelope = node.metrics().with_slack(threshold);
        let fanout_limit = self
            .driver
            .as_ref()
            .map_or(f64::INFINITY, |d| d.fanout_limit);
        let mut fallback: Option<BnetPtr> = None;
        let mut options = Vec::with_capacity(sides.left.len() * sides.right.len());
        for l in sides.left {
            for r in sides.right {
                let junction = Bnet::junction(node.at, Rc::clone(l), Rc::clone(r));
                if fallback.is_none() && junction.fits(&envelope) {
                    fallback = Some(Rc::clone(&junction));
                }
                if junction.fanout <= fanout_limit {
                    options.push(junction);
                }
            }
        }
        prune_cap_against_area(&mut options);
        if !options.iter().any(|o| o.fits(&envelope))
            && let Some(fallback) = fallback
        {
            insert_assured(&mut options, fallback);
        }
        options
    }
}
