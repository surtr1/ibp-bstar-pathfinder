# IBP-B-Star Pathfinding

This repository is organized as a pathfinding comparison project centered on:

- `Branch Star`
- `IBP-B*`
- `BFS`
- `A*`
- `Dijkstra`

Quick links:

- Chinese documentation: `README.zh-CN.md`
- English documentation: `README.en.md`

Current top-level layout:

- `apps/` executable entry points
- `include/` public headers
- `src/` core implementations
- `legacy/` older single-file or compatibility code
- `maps/` sample maps
- `scripts/` reference or experiment scripts
- `docs/papers/` related papers and PDFs

Quick example:

```bash
./pathfinding_compare --map maps/map.txt --algorithms bfs,astar,dijkstra,branchstar,ibp_bstar --no-print --ascii
```
