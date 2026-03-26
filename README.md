# IBP-B-Star Pathfinding

感谢  Twilight-Dream-Of-Magic 的支持 (github:https://github.com/Twilight-Dream-Of-Magic)

多算法路径规划对比项目，当前包含：

- `BFS`
- `A*`
- `Dijkstra`
- `B* PaperCrawl`
- `B* GreedyLite`
- `B* Robust`
- `IBP-B*`

文档入口：

- 中文说明：[README.zh-CN.md](/d:/编程/C&C++/项目/IBP-BSTAR算法/IBP-B-StarPathfinding/README.zh-CN.md)
- English guide: [README.en.md](/d:/编程/C&C++/项目/IBP-BSTAR算法/IBP-B-StarPathfinding/README.en.md)
- 地图说明：[maps/README.md](/d:/编程/C&C++/项目/IBP-BSTAR算法/IBP-B-StarPathfinding/maps/README.md)

目录结构：

- `apps/` 可执行入口
- `include/` 公共头文件
- `src/` 核心实现
- `maps/` 示例地图
- `scripts/` GUI 与参考脚本
- `docs/papers/` 论文资料

快速示例：

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --algorithms bfs,astar,dijkstra,bstar_paper,ibp_bstar --no-print --ascii
```

JSON 示例：

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --algorithms astar,bstar_paper,ibp_bstar --json > result.json
```
