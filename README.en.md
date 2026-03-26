# IBP-B-Star Pathfinding

Thanks to Twilight-Dream-Of-Magic for the support.

## 1. Overview

This repository is a grid-based pathfinding comparison project. It currently includes:

- `BFS`
- `A*`
- `Dijkstra`
- `B* PaperCrawl`
- `B* GreedyLite`
- `B* Robust`
- `IBP-B*`

The project is not only an algorithm implementation demo. It also includes:

- a unified comparison driver
- curated map files
- JSON output for external tools
- a PySide6 GUI

## 2. B* Variants in This Repository

The three B* variants are intentionally kept separate:

- `B* PaperCrawl`
  - the current default Zhao-style / paper-style variant
  - entry: `RunBranchStar(...)`
- `B* GreedyLite`
  - a lighter classic greedy branch-following variant
  - entry: `RunBranchStarClassic(...)`
- `B* Robust`
  - a heavier fallback-oriented comparison variant
  - entry: `RunBranchStarLegacy(...)`

`IBP-B*` is your optimized bidirectional enhanced algorithm and is wrapped through `RunIbpBStar(...)`.

## 3. Project Layout

- `apps/`
  - executable entry points
- `include/`
  - shared headers
- `src/`
  - main implementations
- `maps/`
  - curated map files
- `scripts/`
  - GUI and reference scripts
- `docs/papers/`
  - related papers

## 4. Build

### 4.1 CMake

```bash
cmake -S . -B build
cmake --build build --config Release
```

### 4.2 Manual Build

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8 -Iinclude apps/pathfinding_compare_main.cpp src/pathfinding_grid.cpp src/pathfinding_algorithms.cpp src/pathfinding_compare.cpp src/ibp_bstar_core.cpp -o pathfinding_compare
```

## 5. Example Usage

### 5.1 Compare the default set

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --no-print --ascii
```

### 5.2 Compare paper B* and IBP-B*

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --algorithms bstar_paper,ibp_bstar --no-print --ascii
```

### 5.3 Compare all B* variants

```bash
./pathfinding_compare --map maps/one_blockers.txt --algorithms bstar_paper,bstar_greedy_lite,bstar_robust,ibp_bstar --no-print --ascii
```

### 5.4 Random map

```bash
./pathfinding_compare --random 64 64 0.25 --seed 12345 --no-print
```

### 5.5 Staggered wall map

```bash
./pathfinding_compare --staggered-walls 120 60 0.35 --seed 11 --branch-no-reverse --no-print
```

### 5.6 Perfect maze

```bash
./pathfinding_compare --maze 31 31 --seed 7 --algorithms bstar_paper,ibp_bstar --no-print
```

### 5.7 Paper-strict IBP-B*

```bash
./pathfinding_compare --map maps/closed_goal.txt --algorithms ibp_bstar --ibp-paper-strict --no-print
```

### 5.8 JSON output

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --algorithms astar,bstar_paper,ibp_bstar --json > result.json
```

## 6. GUI

Install the dependency:

```bash
python -m pip install -r scripts/requirements-gui.txt
```

Run the viewer:

```bash
python scripts/pathfinding_gui.py
```

The GUI:

- calls the compiled `pathfinding_compare` executable through `subprocess`
- requests `--json` output
- renders the map and selected path
- supports path overlay
- supports start/goal override and click-picking

By default, the GUI now looks for `pathfinding_compare.exe` in the current project directory first.

## 7. Maps

All curated map files are now stored directly under `maps/`.

See [maps/README.md](/d:/编程/C&C++/项目/IBP-BSTAR算法/IBP-B-StarPathfinding/maps/README.md) for the current map list and short descriptions.

## 8. Output Fields

The summary table prints:

- `Algorithm`
- `Success`
- `PathLen`
- `Expanded`
- `Time(us)`
- `GapToBFS`

Where:

- `PathLen` is the final path length
- `Expanded` is the number of expanded nodes or states
- `Time(us)` is runtime in microseconds
- `GapToBFS` is the path-length difference relative to BFS

## 9. References

- `docs/papers/An Intelligent Bi-Directional Parallel B-Star Routing Algorithm.pdf`
- `docs/papers/jcc_2020062915444066.pdf`
