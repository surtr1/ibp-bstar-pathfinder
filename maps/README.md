# Maps

Current curated map files are stored directly under `maps/`.

## Benchmark-style maps

- `ring_obstacle.txt`
  - ring / enclosure style obstacle map
  - suitable for observing B* boundary-following behavior

- `closed_goal.txt`
  - closed obstacle / unreachable-goal case
  - useful for comparing failure behavior

- `one_blockers.txt`
  - repeated local blockers
  - suitable for comparing local bypass behavior

## Game-style maps

- `moba1.txt`
  - MOBA-style layout variant 1

- `moba2.txt`
  - MOBA-style layout variant 2

- `arcade1.txt`
  - arcade-style layout variant 1

- `arcade2.txt`
  - arcade-style layout variant 2

- `arcade3.txt`
  - arcade-style layout variant 3

- `arcade4.txt`
  - arcade-style layout variant 4

You can run any map with:

```bash
./pathfinding_compare --map maps/<file>.txt --no-print --ascii
```
