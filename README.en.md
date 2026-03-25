# IBP-B-Star Pathfinding

## 1. Overview

This repository is organized as a **multi-algorithm pathfinding comparison project**.

It currently compares:

- `BFS`
- `A*`
- `Dijkstra`
- `Branch Star`
- `IBP-B*`

Two B*-related algorithms are intentionally kept separate:

- `Branch Star`
  - the classic branch-based obstacle-bypassing idea
  - move greedily toward the goal, then split into obstacle-following branches when blocked
- `IBP-B*`
  - your optimized implementation
  - based on the intelligent bi-directional parallel B* paper and its extensions

---

## 2. Project Layout

The project is now organized by responsibility:

- `apps/`
  - executable entry points
  - `pathfinding_compare_main.cpp`
  - `ibp_bstar_zigzag_demo_main.cpp`

- `include/`
  - public headers shared by executables and libraries
  - `pathfinding_types.hpp`
  - `pathfinding_grid.hpp`
  - `pathfinding_algorithms.hpp`
  - `pathfinding_compare.hpp`
  - `ibp_bstar_core.hpp`
  - `ibp_bstar_app.hpp`

- `src/`
  - implementation files
  - `pathfinding_grid.cpp`
  - `pathfinding_algorithms.cpp`
  - `pathfinding_compare.cpp`
  - `ibp_bstar_core.cpp`
  - `ibp_bstar_app.cpp`

- `legacy/`
  - older standalone or compatibility code
  - `IBP-B-Star_Pathfinding_PaperStrict.cpp`
  - `ibp_bstar.hpp`
  - `bfs_pathfinder.cpp`

- `maps/`
  - sample map files
  - `map.txt`

- `scripts/`
  - reference / experiment scripts
  - `AStar-BStar New Implementation.py`

- `docs/papers/`
  - related papers and PDFs

This layout keeps benchmark code, algorithm code, legacy code, scripts, and reference materials separated.

---

## 3. Algorithm Meaning in This Repository

### 3.1 Branch Star

`Branch Star` here means the classic branch-based obstacle bypassing strategy:

1. move greedily toward the goal
2. continue straight if the preferred direction is free
3. when blocked, branch around the obstacle
4. once a branch regains a valid goal-oriented move, switch back to direct motion
5. succeed if one branch reaches the goal

Current unified entry:

- `RunBranchStar(...)`
- implemented in `src/pathfinding_algorithms.cpp`

### 3.2 IBP-B*

`IBP-B*` here is your optimized algorithm with features such as:

- bidirectional expansion
- greedy advancement
- obstacle crawling
- rebirth
- concave pre-exploration
- optional local zigzag expansion
- optional maze rescue

It is exposed through:

- `RunIbpBStar(...)`
- wrapper in `src/pathfinding_algorithms.cpp`

The core implementation remains in:

- `src/ibp_bstar_core.cpp`

---

## 4. Build

### 4.1 CMake

Recommended:

```bash
cmake -S . -B build
cmake --build build --config Release
```

Main targets:

- `pathfinding_compare`
- `ibp_bstar_zigzag_mode`
- `ibp_bstar_paper_strict`

### 4.2 Manual Build of the Comparison Program

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8 -Iinclude apps/pathfinding_compare_main.cpp src/pathfinding_grid.cpp src/pathfinding_algorithms.cpp src/pathfinding_compare.cpp src/ibp_bstar_core.cpp -o pathfinding_compare
```

### 4.3 Manual Build of the Legacy Zigzag Demo

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8 -Iinclude apps/ibp_bstar_zigzag_demo_main.cpp src/ibp_bstar_core.cpp src/ibp_bstar_app.cpp -o ibp_bstar_zigzag_mode
```

---

## 5. Example Usage

### 5.1 Compare the Default Algorithm Set

```bash
./pathfinding_compare --map maps/map.txt --no-print --ascii
```

### 5.2 Compare Only Branch Star and IBP-B*

```bash
./pathfinding_compare --map maps/map.txt --algorithms branchstar,ibp_bstar --no-print --ascii
```

### 5.3 Random Map

```bash
./pathfinding_compare --random 64 64 0.25 --seed 12345 --no-print
```

### 5.4 Perfect Maze

```bash
./pathfinding_compare --maze 31 31 --seed 7 --algorithms branchstar,ibp_bstar --no-print
```

### 5.5 Run IBP-B* in a More Paper-Strict Style

```bash
./pathfinding_compare --map maps/map.txt --algorithms ibp_bstar --ibp-paper-strict --no-print
```

### 5.6 Disable Reverse Crawling in Branch Star

```bash
./pathfinding_compare --map maps/map.txt --algorithms branchstar --branch-no-reverse --no-print
```

### 5.7 Print Rendered Paths

```bash
./pathfinding_compare --map maps/map.txt --algorithms bfs,branchstar,ibp_bstar --print-path --ascii
```

---

## 6. CLI Options

### 6.1 Map Source

- `--map FILE`
- `--random W H P`
- `--maze W H`
- `--seed N`

### 6.2 Algorithm Selection

- `--algorithms bfs,astar,dijkstra,branchstar,ibp_bstar`

Accepted aliases include:

- `branchstar`
- `branch-star`
- `b*`
- `ibp_bstar`
- `ibp-bstar`
- `ibp`

### 6.3 Start / Goal Overrides

- `--sx R --sy C`
- `--ex R --ey C`

### 6.4 IBP-B* Options

- `--ibp-wait N`
- `--ibp-paper-strict`
- `--ibp-zigzag`
- `--ibp-zigzag-threshold N`
- `--ibp-rescue`
- `--no-ibp-rescue`

### 6.5 Branch Star Options

- `--branch-no-reverse`

### 6.6 Output Options

- `--print-path`
- `--no-print`
- `--no-summary`
- `--ascii`
- `--unicode`
- `--arrow`

---

## 7. Output Fields

The summary table currently prints:

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

---

## 8. Map Format

The text map loader supports:

- `+` for walls
- `.` for free cells
- `S` for start
- `E` for goal

Example:

```text
+++++++
S...+..+
.+.+.+.+
.+...+E+
+++++++
```

---

## 9. References

- `docs/papers/An Intelligent Bi-Directional Parallel B-Star Routing Algorithm.pdf`
- `docs/papers/jcc_2020062915444066.pdf`
