//! Pattern Routing for Simple Nets on Edge-Based `GCell` Grid.
//!
//! Attempts L-shaped (1-bend) and Z-shaped (2-bend) routes before falling
//! back to full A* maze routing. Uses proper layer assignment: horizontal
//! segments on horizontal-preferred layers, vertical segments on
//! vertical-preferred layers, with vias at bends. Checks edge capacities
//! rather than node occupancy.

use crate::grid::RoutingGrid;
use pare_common::db::core::{LayerDirection, NetlistDB};
use pare_common::geom::coord::GridCoord;

/// Attempts to route a 2-pin connection using pattern routes on the gcell grid.
///
/// Tries L-shaped routes first (1 bend), then Z-shaped routes (2 bends).
/// Returns the path as a vector of `GridCoords` if successful, None if the
/// pattern routes are blocked and A* fallback is needed.
pub fn try_pattern_route<G: RoutingGrid + ?Sized>(
    grid: &G,
    start: GridCoord,
    end: GridCoord,
    db: &NetlistDB,
) -> Option<Vec<GridCoord>> {
    let (h_layer, v_layer) = find_layer_pair(db, start.z, end.z, grid.layers())?;

    // Try L-routes first (1 bend)
    if let Some(path) = try_l_route(grid, start, end, h_layer, v_layer) {
        return Some(path);
    }

    // Try Z-routes (2 bends) - only if meaningful distance
    let dx = (end.x as i32 - start.x as i32).unsigned_abs();
    let dy = (end.y as i32 - start.y as i32).unsigned_abs();
    if (dx >= 2 || dy >= 2)
        && let Some(path) = try_z_route(grid, start, end, h_layer, v_layer)
    {
        return Some(path);
    }

    // Try 3-bend routes (S-shape and U-shape) - only if sufficient distance
    if (dx >= 3 || dy >= 3)
        && let Some(path) = try_three_bend_route(grid, start, end, h_layer, v_layer)
    {
        return Some(path);
    }

    None
}

/// Finds a layer pair (`horizontal_layer`, `vertical_layer`) near the pin layers.
fn find_layer_pair(db: &NetlistDB, z1: u8, z2: u8, max_layers: u8) -> Option<(u8, u8)> {
    let base = z1.min(z2).max(1);
    let top = max_layers.min(base + 3);

    let mut h_layer = None;
    let mut v_layer = None;

    for z in base..top {
        if (z as usize) >= db.layers.len() {
            break;
        }
        match db.layers[z as usize].direction {
            LayerDirection::Horizontal => {
                if h_layer.is_none() {
                    h_layer = Some(z);
                }
            }
            LayerDirection::Vertical => {
                if v_layer.is_none() {
                    v_layer = Some(z);
                }
            }
            LayerDirection::Unknown => {}
        }
        if h_layer.is_some() && v_layer.is_some() {
            break;
        }
    }

    match (h_layer, v_layer) {
        (Some(h), Some(v)) => Some((h, v)),
        _ => None,
    }
}

/// Checks if a horizontal segment from x1 to x2 at (y, layer) is feasible.
/// Checks edge capacities along the path. Fails if any edge is at >=80% capacity.
fn trace_horizontal<G: RoutingGrid + ?Sized>(
    grid: &G,
    path: &mut Vec<GridCoord>,
    x1: u32,
    x2: u32,
    y: u32,
    z: u8,
) -> bool {
    if x1 == x2 {
        let coord = GridCoord::new(x1, y, z);
        if path.last() != Some(&coord) {
            path.push(coord);
        }
        return true;
    }
    let (lo, hi) = if x1 < x2 { (x1, x2) } else { (x2, x1) };

    // Check all horizontal edges - fail if any is congested or near capacity
    for x in lo..hi {
        let cap = grid.h_edge_cap(x, y, z);
        if cap == 0 {
            return false;
        }
        let usage = grid.h_edge_usage(x, y, z);
        // Fail if at or above 80% capacity to leave room for other nets
        if usage >= cap || (cap > 2 && usage as u32 * 5 >= cap as u32 * 4) {
            return false;
        }
    }

    // Path is feasible, add gcell coords
    for x in lo..=hi {
        let coord = GridCoord::new(x, y, z);
        if path.last() != Some(&coord) {
            path.push(coord);
        }
    }
    true
}

/// Checks if a vertical segment from y1 to y2 at (x, layer) is feasible.
fn trace_vertical<G: RoutingGrid + ?Sized>(
    grid: &G,
    path: &mut Vec<GridCoord>,
    x: u32,
    y1: u32,
    y2: u32,
    z: u8,
) -> bool {
    if y1 == y2 {
        let coord = GridCoord::new(x, y1, z);
        if path.last() != Some(&coord) {
            path.push(coord);
        }
        return true;
    }
    let (lo, hi) = if y1 < y2 { (y1, y2) } else { (y2, y1) };

    // Check all vertical edges - fail if any is congested or near capacity
    for y in lo..hi {
        let cap = grid.v_edge_cap(x, y, z);
        if cap == 0 {
            return false;
        }
        let usage = grid.v_edge_usage(x, y, z);
        // Fail if at or above 80% capacity to leave room for other nets
        if usage >= cap || (cap > 2 && usage as u32 * 5 >= cap as u32 * 4) {
            return false;
        }
    }

    for y in lo..=hi {
        let coord = GridCoord::new(x, y, z);
        if path.last() != Some(&coord) {
            path.push(coord);
        }
    }
    true
}

/// Adds via cells connecting two layers at the same (x, y).
fn add_via(path: &mut Vec<GridCoord>, x: u32, y: u32, from_z: u8, to_z: u8) {
    if from_z == to_z {
        return;
    }
    let (lo, hi) = if from_z < to_z {
        (from_z, to_z)
    } else {
        (to_z, from_z)
    };
    for z in lo..=hi {
        let coord = GridCoord::new(x, y, z);
        if path.last() != Some(&coord) {
            path.push(coord);
        }
    }
}

/// Tries L-shaped routes using proper layer assignment.
fn try_l_route<G: RoutingGrid + ?Sized>(
    grid: &G,
    start: GridCoord,
    end: GridCoord,
    h_layer: u8,
    v_layer: u8,
) -> Option<Vec<GridCoord>> {
    // Route A: horizontal on h_layer, then vertical on v_layer
    {
        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, h_layer);
        let h_ok = trace_horizontal(grid, &mut path, start.x, end.x, start.y, h_layer);
        if h_ok {
            add_via(&mut path, end.x, start.y, h_layer, v_layer);
            let v_ok = trace_vertical(grid, &mut path, end.x, start.y, end.y, v_layer);
            if v_ok {
                add_via(&mut path, end.x, end.y, v_layer, end.z);
                if !path.is_empty() {
                    return Some(path);
                }
            }
        }
    }

    // Route B: vertical on v_layer, then horizontal on h_layer
    {
        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, v_layer);
        let v_ok = trace_vertical(grid, &mut path, start.x, start.y, end.y, v_layer);
        if v_ok {
            add_via(&mut path, start.x, end.y, v_layer, h_layer);
            let h_ok = trace_horizontal(grid, &mut path, start.x, end.x, end.y, h_layer);
            if h_ok {
                add_via(&mut path, end.x, end.y, h_layer, end.z);
                if !path.is_empty() {
                    return Some(path);
                }
            }
        }
    }

    None
}

/// Tries Z-shaped routes using proper layer assignment.
fn try_z_route<G: RoutingGrid + ?Sized>(
    grid: &G,
    start: GridCoord,
    end: GridCoord,
    h_layer: u8,
    v_layer: u8,
) -> Option<Vec<GridCoord>> {
    // Z-route A: H-V-H pattern
    {
        let mid_x = u32::midpoint(start.x, end.x);
        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, h_layer);
        let ok = trace_horizontal(grid, &mut path, start.x, mid_x, start.y, h_layer);
        if ok {
            add_via(&mut path, mid_x, start.y, h_layer, v_layer);
            let ok2 = trace_vertical(grid, &mut path, mid_x, start.y, end.y, v_layer);
            if ok2 {
                add_via(&mut path, mid_x, end.y, v_layer, h_layer);
                let ok3 = trace_horizontal(grid, &mut path, mid_x, end.x, end.y, h_layer);
                if ok3 {
                    add_via(&mut path, end.x, end.y, h_layer, end.z);
                    if !path.is_empty() {
                        return Some(path);
                    }
                }
            }
        }
    }

    // Z-route B: V-H-V pattern
    {
        let mid_y = u32::midpoint(start.y, end.y);
        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, v_layer);
        let ok = trace_vertical(grid, &mut path, start.x, start.y, mid_y, v_layer);
        if ok {
            add_via(&mut path, start.x, mid_y, v_layer, h_layer);
            let ok2 = trace_horizontal(grid, &mut path, start.x, end.x, mid_y, h_layer);
            if ok2 {
                add_via(&mut path, end.x, mid_y, h_layer, v_layer);
                let ok3 = trace_vertical(grid, &mut path, end.x, mid_y, end.y, v_layer);
                if ok3 {
                    add_via(&mut path, end.x, end.y, v_layer, end.z);
                    if !path.is_empty() {
                        return Some(path);
                    }
                }
            }
        }
    }

    None
}

/// Tries 3-bend routes: S-shapes (asymmetric splits) and U-shapes (detours).
///
/// S-shapes split the horizontal or vertical span at 1/3 and 2/3 points,
/// providing alternative paths when the midpoint Z-route is blocked.
/// U-shapes detour by overshooting in one direction, useful when the
/// direct corridor between start and end is congested.
fn try_three_bend_route<G: RoutingGrid + ?Sized>(
    grid: &G,
    start: GridCoord,
    end: GridCoord,
    h_layer: u8,
    v_layer: u8,
) -> Option<Vec<GridCoord>> {
    // S-shape: H-V-H with 1/3 and 2/3 split points
    for &frac in &[1, 2] {
        let split_x = start.x as i32 + (end.x as i32 - start.x as i32) * frac / 3;
        let split_x = split_x.max(0) as u32;

        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, h_layer);
        if trace_horizontal(grid, &mut path, start.x, split_x, start.y, h_layer) {
            add_via(&mut path, split_x, start.y, h_layer, v_layer);
            if trace_vertical(grid, &mut path, split_x, start.y, end.y, v_layer) {
                add_via(&mut path, split_x, end.y, v_layer, h_layer);
                if trace_horizontal(grid, &mut path, split_x, end.x, end.y, h_layer) {
                    add_via(&mut path, end.x, end.y, h_layer, end.z);
                    if !path.is_empty() {
                        return Some(path);
                    }
                }
            }
        }
    }

    // S-shape: V-H-V with 1/3 and 2/3 split points
    for &frac in &[1, 2] {
        let split_y = start.y as i32 + (end.y as i32 - start.y as i32) * frac / 3;
        let split_y = split_y.max(0) as u32;

        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, v_layer);
        if trace_vertical(grid, &mut path, start.x, start.y, split_y, v_layer) {
            add_via(&mut path, start.x, split_y, v_layer, h_layer);
            if trace_horizontal(grid, &mut path, start.x, end.x, split_y, h_layer) {
                add_via(&mut path, end.x, split_y, h_layer, v_layer);
                if trace_vertical(grid, &mut path, end.x, split_y, end.y, v_layer) {
                    add_via(&mut path, end.x, end.y, v_layer, end.z);
                    if !path.is_empty() {
                        return Some(path);
                    }
                }
            }
        }
    }

    // U-shape: detour by overshooting in Y direction
    // H to target X, then V overshoot past target Y, then H back, then V to target
    let dy = (end.y as i32 - start.y as i32).unsigned_abs();
    let dx = (end.x as i32 - start.x as i32).unsigned_abs();
    let overshoot = (dy.max(dx) / 3).max(2);

    for &dir in &[-1i32, 1] {
        let detour_y = end.y as i32 + dir * overshoot as i32;
        if detour_y < 0 {
            continue;
        }
        let detour_y = detour_y as u32;

        // U-shape A: V-H-V-H — go vertically to detour_y, horizontally to end.x,
        // vertically to end.y, done
        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, v_layer);
        if trace_vertical(grid, &mut path, start.x, start.y, detour_y, v_layer) {
            add_via(&mut path, start.x, detour_y, v_layer, h_layer);
            if trace_horizontal(grid, &mut path, start.x, end.x, detour_y, h_layer) {
                add_via(&mut path, end.x, detour_y, h_layer, v_layer);
                if trace_vertical(grid, &mut path, end.x, detour_y, end.y, v_layer) {
                    add_via(&mut path, end.x, end.y, v_layer, end.z);
                    if !path.is_empty() {
                        return Some(path);
                    }
                }
            }
        }
    }

    // U-shape: detour by overshooting in X direction
    for &dir in &[-1i32, 1] {
        let detour_x = end.x as i32 + dir * overshoot as i32;
        if detour_x < 0 {
            continue;
        }
        let detour_x = detour_x as u32;

        // U-shape B: H-V-H-V — go horizontally to detour_x, vertically to end.y,
        // horizontally to end.x, done
        let mut path = Vec::new();
        add_via(&mut path, start.x, start.y, start.z, h_layer);
        if trace_horizontal(grid, &mut path, start.x, detour_x, start.y, h_layer) {
            add_via(&mut path, detour_x, start.y, h_layer, v_layer);
            if trace_vertical(grid, &mut path, detour_x, start.y, end.y, v_layer) {
                add_via(&mut path, detour_x, end.y, v_layer, h_layer);
                if trace_horizontal(grid, &mut path, detour_x, end.x, end.y, h_layer) {
                    add_via(&mut path, end.x, end.y, h_layer, end.z);
                    if !path.is_empty() {
                        return Some(path);
                    }
                }
            }
        }
    }

    None
}
