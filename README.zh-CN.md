# IBP-B-Star Pathfinding 中文文档

感谢  Twilight-Dream-Of-Magic 的支持 (github:https://github.com/Twilight-Dream-Of-Magic)

## 1. 项目简介

这是一个基于二维栅格地图的多算法路径规划对比项目，当前包含：

- `BFS`
- `A*`
- `Dijkstra`
- `B* PaperCrawl`
- `B* GreedyLite`
- `B* Robust`
- `IBP-B*`

这个项目不只是算法实现，还包含：

- 统一的算法对比入口
- 固定测试地图
- 供外部工具使用的 JSON 输出
- 基于 PySide6 的可视化界面

## 2. 本项目中的 B* 版本

项目里把三种 B* 变体明确区分开：

- `B* PaperCrawl`
  - 当前默认的 论文语义版
  - 入口：`RunBranchStar(...)`
- `B* GreedyLite`
  - 更轻量的经典贪心分支绕障版
  - 入口：`RunBranchStarClassic(...)`
- `B* Robust`
  - 更重、更强调兜底能力的对照版
  - 入口：`RunBranchStarLegacy(...)`

`IBP-B*` 是你优化实现的双向增强算法，对比框架中的入口是 `RunIbpBStar(...)`。

## 3. 目录结构

- `apps/`
  - 可执行入口
- `include/`
  - 公共头文件
- `src/`
  - 主要实现
- `maps/`
  - 测试地图
- `scripts/`
  - GUI 和参考脚本
- `docs/papers/`
  - 论文资料

## 4. 构建方式

### 4.1 使用 CMake

```bash
cmake -S . -B build
cmake --build build --config Release
```

### 4.2 手动编译

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8 -Iinclude apps/pathfinding_compare_main.cpp src/pathfinding_grid.cpp src/pathfinding_algorithms.cpp src/pathfinding_compare.cpp src/ibp_bstar_core.cpp -o pathfinding_compare
```

## 5. 运行示例

### 5.1 运行默认算法集合

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --no-print --ascii
```

### 5.2 只对比论文版 B* 和 IBP-B*

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --algorithms bstar_paper,ibp_bstar --no-print --ascii
```

### 5.3 对比三种 B* 版本

```bash
./pathfinding_compare --map maps/one_blockers.txt --algorithms bstar_paper,bstar_greedy_lite,bstar_robust,ibp_bstar --no-print --ascii
```

### 5.4 随机地图

```bash
./pathfinding_compare --random 64 64 0.25 --seed 12345 --no-print
```

### 5.5 交错竖墙地图

```bash
./pathfinding_compare --staggered-walls 120 60 0.35 --seed 11 --branch-no-reverse --no-print
```

### 5.6 完美迷宫

```bash
./pathfinding_compare --maze 31 31 --seed 7 --algorithms bstar_paper,ibp_bstar --no-print
```

### 5.7 更严格的论文风格 IBP-B*

```bash
./pathfinding_compare --map maps/closed_goal.txt --algorithms ibp_bstar --ibp-paper-strict --no-print
```

### 5.8 导出 JSON

```bash
./pathfinding_compare --map maps/ring_obstacle.txt --algorithms astar,bstar_paper,ibp_bstar --json > result.json
```

## 6. GUI

先安装依赖：

```bash
python -m pip install -r scripts/requirements-gui.txt
```

启动界面：

```bash
python scripts/pathfinding_gui.py
```

GUI 会：

- 通过 `subprocess` 调用 `pathfinding_compare`
- 请求 `--json` 输出
- 显示地图和路径
- 支持多算法路径叠加
- 支持起点终点覆盖和点图选点

现在 GUI 会优先查找当前目录下的 `pathfinding_compare.exe`。

## 7. 地图说明

当前正式地图统一放在 `maps/` 目录下。

地图列表和简要说明见：

- [maps/README.md](/d:/编程/C&C++/项目/IBP-BSTAR算法/IBP-B-StarPathfinding/maps/README.md)

## 8. 输出字段说明

摘要表会输出：

- `Algorithm`
- `Success`
- `PathLen`
- `Expanded`
- `Time(us)`
- `GapToBFS`

其中：

- `PathLen` 表示最终路径长度
- `Expanded` 表示扩展节点或状态数
- `Time(us)` 表示微秒级耗时
- `GapToBFS` 表示相对 BFS 的路径长度差值

## 9. 参考资料

- `docs/papers/An Intelligent Bi-Directional Parallel B-Star Routing Algorithm.pdf`
- `docs/papers/jcc_2020062915444066.pdf`
