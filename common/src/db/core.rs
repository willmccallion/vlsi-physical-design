use crate::db::indices::*;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq)]
pub enum LayerDirection {
    Vertical,
    Horizontal,
    Unknown,
}

#[derive(Clone, Debug)]
pub struct LayerData {
    pub name: String,
    pub index: u8,
    pub direction: LayerDirection,
    pub pitch: f64,
    pub width: f64,
}

#[derive(Clone, Debug)]
pub struct RouteSegment {
    pub layer: u8,
    pub p1: Point<f64>,
    pub p2: Point<f64>,
}

#[derive(Clone, Debug)]
pub struct CellData {
    pub name: String,
    pub lib_name: String,
    pub width: f64,
    pub height: f64,
    pub is_fixed: bool,
    pub is_macro: bool,
    pub pins: Vec<PinId>,
}

#[derive(Clone, Debug)]
pub struct NetData {
    pub name: String,
    pub weight: f64,
    pub pins: Vec<PinId>,
    pub route_segments: Vec<RouteSegment>,
}

#[derive(Clone, Debug, Default)]
pub struct TrackDef {
    pub layer: String,
    pub direction: String,
    pub start: f64,
    pub num_tracks: u32,
    pub step: f64,
}

pub struct NetlistDB {
    pub layers: Vec<LayerData>,
    pub cells: Vec<CellData>,
    pub nets: Vec<NetData>,
    pub tracks: Vec<TrackDef>,

    pub pin_offsets: Vec<Point<f64>>,
    pub pin_names: Vec<String>,
    pub pin_to_cell: Vec<CellId>,
    pub pin_to_net: Vec<NetId>,

    pub positions: Vec<Point<f64>>,
    pub die_area: Rect,

    pub cell_name_map: HashMap<String, CellId>,
    pub net_name_map: HashMap<String, NetId>,
    pub layer_name_map: HashMap<String, u8>,

    pub macro_pins: HashMap<String, HashMap<String, Point<f64>>>,
    pub macro_sizes: HashMap<String, (f64, f64)>,
}

impl NetlistDB {
    pub fn new() -> Self {
        Self {
            layers: Vec::new(),
            cells: Vec::with_capacity(1000),
            nets: Vec::with_capacity(1000),
            tracks: Vec::new(),
            pin_offsets: Vec::with_capacity(5000),
            pin_names: Vec::with_capacity(5000),
            pin_to_cell: Vec::with_capacity(5000),
            pin_to_net: Vec::with_capacity(5000),
            positions: Vec::with_capacity(1000),
            die_area: Rect::default(),
            cell_name_map: HashMap::new(),
            net_name_map: HashMap::new(),
            layer_name_map: HashMap::new(),
            macro_pins: HashMap::new(),
            macro_sizes: HashMap::new(),
        }
    }

    pub fn num_cells(&self) -> usize {
        self.cells.len()
    }
    pub fn num_nets(&self) -> usize {
        self.nets.len()
    }

    #[inline]
    pub fn get_pin_position(&self, pin: PinId, cell_pos: &Point<f64>) -> Point<f64> {
        let offset = self.pin_offsets[pin.index()];
        *cell_pos + offset
    }

    pub fn add_layer(&mut self, name: String, direction: LayerDirection, pitch: f64, width: f64) {
        let idx = self.layers.len() as u8;
        self.layer_name_map.insert(name.clone(), idx);
        self.layers.push(LayerData {
            name,
            index: idx,
            direction,
            pitch,
            width,
        });
    }

    pub fn add_cell(
        &mut self,
        name: String,
        lib_name: String,
        width: f64,
        height: f64,
        is_fixed: bool,
    ) -> CellId {
        let id = CellId::new(self.cells.len());
        self.cells.push(CellData {
            name: name.clone(),
            lib_name,
            width,
            height,
            is_fixed,
            is_macro: false,
            pins: Vec::new(),
        });
        self.positions.push(Point::new(0.0, 0.0));
        self.cell_name_map.insert(name, id);
        id
    }

    pub fn add_net(&mut self, name: String) -> NetId {
        if let Some(&id) = self.net_name_map.get(&name) {
            return id;
        }
        let id = NetId::new(self.nets.len());
        self.nets.push(NetData {
            name: name.clone(),
            weight: 1.0,
            pins: Vec::new(),
            route_segments: Vec::new(),
        });
        self.net_name_map.insert(name, id);
        id
    }

    pub fn add_pin(&mut self, cell: CellId, net: NetId, offset: Point<f64>, name: String) {
        let pid = PinId::new(self.pin_offsets.len());
        self.pin_offsets.push(offset);
        self.pin_names.push(name);
        self.pin_to_cell.push(cell);
        self.pin_to_net.push(net);

        self.cells[cell.index()].pins.push(pid);
        self.nets[net.index()].pins.push(pid);
    }

    pub fn get_or_create_io_cell(&mut self) -> CellId {
        if let Some(&id) = self.cell_name_map.get("IO_VIRTUAL_CELL") {
            return id;
        }
        let id = self.add_cell(
            "IO_VIRTUAL_CELL".to_string(),
            "IO_LIB".to_string(),
            0.0,
            0.0,
            true,
        );
        self.positions[id.index()] = Point::new(0.0, 0.0);
        id
    }
}
