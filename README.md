# EDA Placer and Router

A modular physical design toolchain for digital integrated circuits, written in Rust. This project implements analytical placement using electrostatic analogies and congestion-aware routing based on the Pathfinder algorithm.

![10k Nets Routing](assets/routed_10k.png)
*Visualization of 10,000 nets routed on a multi-layer metal stack.*

## Project Status

| Benchmark | Size | Status | Notes |
| :--- | :--- | :--- | :--- |
| **Random** | 1k - 5k Nets | Stable | Converges in under 30 seconds. |
| **Random** | 10k Nets | Experimental | Usually converges, but may occasionally stall with 1-2 conflicts. |
| **GCD** | ~500 Nets | Stable | Full flow (Placement + Routing) works. |
| **AES** | ~50k Nets | In Progress | Placement works; Routing optimization is ongoing. |

## Overview

This tool accepts industry-standard LEF (Library Exchange Format) and DEF (Design Exchange Format) files. It transforms a logical netlist into a physical layout through three stages:

1.  **Global Placement:** Solves a non-linear optimization problem to distribute cells while minimizing wirelength.
2.  **Legalization:** Aligns cells to the manufacturing grid and removes overlaps using the Abacus algorithm.
3.  **Detailed Routing:** Connects pins using a 3D maze routing approach with negotiation-based congestion resolution.

## Visualization Gallery

| 5k Nets | GCD (Real Design) |
| :---: | :---: |
| <img src="assets/routed_5k.png" width="400"> | <img src="assets/routed_gcd.png" width="400"> |

## Methodology

### Analytical Global Placement
The placement problem is formulated as a constrained optimization problem. The objective function minimizes total wirelength subject to a density penalty.

#### Wirelength Model
The tool uses the **Weighted Average (WA)** smooth approximation for Half-Perimeter Wirelength (HPWL). This provides a differentiable function that pulls connected components together.

#### Density Model
To prevent cell overlap, the tool models cell density as an electrostatic system. Cells are treated as positively charged particles, and the target density is a uniform negative background charge. The system solves **Poisson's Equation** using the **Fast Fourier Transform (FFT)** to compute the potential field. The gradient of this field yields a repulsive force that spreads cells apart.

#### Optimization
The total gradient (wirelength + density) is minimized using **Nesterov's Accelerated Gradient Descent**.

### Congestion-Aware Routing
The router implements the **Pathfinder algorithm**, utilizing an iterative **Rip-up and Reroute** strategy. It solves the shortest path problem on a 3D grid graph using A* search.

The cost function for a grid node is dynamic:
*   **Base Cost:** Distance and via penalties.
*   **History Cost:** Increases permanently every time a node is congested.
*   **Present Penalty:** Increases geometrically with iteration count.

This formulation forces nets to negotiate for resources. If a node is highly contested, the cost becomes prohibitive, forcing nets to find alternative paths.

## Usage

### Prerequisites
*   Rust (latest stable toolchain)

### Building
Build in release mode for performance:

```bash
cargo build --release
```

### Running the Flow

**Generate Benchmark**
Generate a random netlist with unique pin constraints.

```bash
cargo run --release -p eda-cli -- generate --cells 5000 --nets 5000
```

**Execute Flow**
Run Parsing, Placement, Legalization, and Routing.

```bash
cargo run --release -p eda-cli -- flow
```

Configuration parameters (placement density, routing iterations, penalty factors) can be modified in the `config.toml` file located in the project root.

## Project Structure

*   **`common/`**: Core database structures (`NetlistDB`), geometry primitives, and LEF/DEF parsers.
*   **`placer/`**:
    *   `physics/`: FFT-based electrostatic solver and wirelength gradients.
    *   `solver/`: Nesterov optimizer.
    *   `legalize/`: Abacus legalization logic.
*   **`router/`**:
    *   `grid/`: Dense grid management with history/occupancy tracking.
    *   `algo/`: A* pathfinder logic.
*   **`cli/`**: Command-line driver and benchmark generator.

## Output
*   **`output/placed.def`**: Final component locations and routing.
*   **`output/placed.png`**: Visualization of cell placement.
*   **`output/routed.png`**: Visualization of routed metal layers.
