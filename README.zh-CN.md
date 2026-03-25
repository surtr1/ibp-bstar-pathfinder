# IBP-B-Star Pathfinding 中文文档

## 1. 项目简介

本项目现在整理成了一个**多算法路径规划对比项目**。

当前接入的算法包括：

- `BFS`
- `A*`
- `Dijkstra`
- `Branch Star`
- `IBP-B*`

其中两种和 B* 相关的算法被明确区分开：

- `Branch Star`
  - 经典分支绕障思路
  - 先朝目标贪心前进，遇障后分支绕行
- `IBP-B*`
  - 你优化后的实现
  - 基于智能双向并行 B* 论文，并带有扩展功能

---

## 2. 目录结构

项目现在按职责分层：

- `apps/`
  - 可执行程序入口
  - `pathfinding_compare_main.cpp`
  - `ibp_bstar_zigzag_demo_main.cpp`

- `include/`
  - 公共头文件
  - `pathfinding_types.hpp`
  - `pathfinding_grid.hpp`
  - `pathfinding_algorithms.hpp`
  - `pathfinding_compare.hpp`
  - `ibp_bstar_core.hpp`
  - `ibp_bstar_app.hpp`

- `src/`
  - 具体实现
  - `pathfinding_grid.cpp`
  - `pathfinding_algorithms.cpp`
  - `pathfinding_compare.cpp`
  - `ibp_bstar_core.cpp`
  - `ibp_bstar_app.cpp`

- `legacy/`
  - 较早的单文件实现或兼容代码
  - `IBP-B-Star_Pathfinding_PaperStrict.cpp`
  - `ibp_bstar.hpp`
  - `bfs_pathfinder.cpp`

- `maps/`
  - 示例地图
  - `map.txt`

- `scripts/`
  - 参考脚本或实验脚本
  - `AStar-BStar New Implementation.py`

- `docs/papers/`
  - 论文和参考 PDF

这样整理后，算法代码、对比入口、旧版代码、脚本和资料就不会再混在根目录里。

---

## 3. 本项目中的算法含义

### 3.1 Branch Star

这里的 `Branch Star` 指经典分支绕障算法：

1. 优先朝目标方向前进
2. 如果前方可走，就继续直走
3. 如果被障碍挡住，就沿障碍分支绕行
4. 一旦重新获得朝目标推进的机会，就恢复直推
5. 任意分支到达终点即成功

统一入口在：

- `RunBranchStar(...)`
- 文件：`src/pathfinding_algorithms.cpp`

### 3.2 IBP-B*

这里的 `IBP-B*` 是你当前优化后的算法，包含：

- 双向扩展
- 贪心推进
- 绕障 crawling
- rebirth
- concave 预探索
- 可选局部 zigzag 增强
- 可选迷宫 rescue

统一对比框架里的入口是：

- `RunIbpBStar(...)`
- 文件：`src/pathfinding_algorithms.cpp`

底层核心实现仍然放在：

- `src/ibp_bstar_core.cpp`

---

## 4. 构建方式

### 4.1 使用 CMake

推荐方式：

```bash
cmake -S . -B build
cmake --build build --config Release
```

主要目标包括：

- `pathfinding_compare`
- `ibp_bstar_zigzag_mode`
- `ibp_bstar_paper_strict`

### 4.2 手动编译对比程序

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8 -Iinclude apps/pathfinding_compare_main.cpp src/pathfinding_grid.cpp src/pathfinding_algorithms.cpp src/pathfinding_compare.cpp src/ibp_bstar_core.cpp -o pathfinding_compare
```

### 4.3 手动编译旧的 Zigzag 演示程序

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8 -Iinclude apps/ibp_bstar_zigzag_demo_main.cpp src/ibp_bstar_core.cpp src/ibp_bstar_app.cpp -o ibp_bstar_zigzag_mode
```

---

## 5. 运行示例

### 5.1 使用默认算法集合

```bash
./pathfinding_compare --map maps/map.txt --no-print --ascii
```

### 5.2 只对比 Branch Star 和 IBP-B*

```bash
./pathfinding_compare --map maps/map.txt --algorithms branchstar,ibp_bstar --no-print --ascii
```

### 5.3 随机地图

```bash
./pathfinding_compare --random 64 64 0.25 --seed 12345 --no-print
```

### 5.4 完美迷宫

```bash
./pathfinding_compare --maze 31 31 --seed 7 --algorithms branchstar,ibp_bstar --no-print
```

### 5.5 更严格的论文风格 IBP-B*

```bash
./pathfinding_compare --map maps/map.txt --algorithms ibp_bstar --ibp-paper-strict --no-print
```

### 5.6 关闭 Branch Star 的回退

```bash
./pathfinding_compare --map maps/map.txt --algorithms branchstar --branch-no-reverse --no-print
```

### 5.7 打印路径

```bash
./pathfinding_compare --map maps/map.txt --algorithms bfs,branchstar,ibp_bstar --print-path --ascii
```

---

## 6. 命令行参数

### 6.1 地图来源

- `--map FILE`
- `--random W H P`
- `--maze W H`
- `--seed N`

### 6.2 算法选择

- `--algorithms bfs,astar,dijkstra,branchstar,ibp_bstar`

兼容别名包括：

- `branchstar`
- `branch-star`
- `b*`
- `ibp_bstar`
- `ibp-bstar`
- `ibp`

### 6.3 起终点覆盖

- `--sx R --sy C`
- `--ex R --ey C`

### 6.4 IBP-B* 相关参数

- `--ibp-wait N`
- `--ibp-paper-strict`
- `--ibp-zigzag`
- `--ibp-zigzag-threshold N`
- `--ibp-rescue`
- `--no-ibp-rescue`

### 6.5 Branch Star 相关参数

- `--branch-no-reverse`

### 6.6 输出相关参数

- `--print-path`
- `--no-print`
- `--no-summary`
- `--ascii`
- `--unicode`
- `--arrow`

---

## 7. 输出字段说明

摘要表当前输出：

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
- `GapToBFS` 表示相对 BFS 路径长度的差值

---

## 8. 地图格式

文本地图支持：

- `+` 表示障碍
- `.` 表示空地
- `S` 表示起点
- `E` 表示终点

示例：

```text
+++++++
S...+..+
.+.+.+.+
.+...+E+
+++++++
```

---

## 9. 参考资料

- `docs/papers/An Intelligent Bi-Directional Parallel B-Star Routing Algorithm.pdf`
- `docs/papers/jcc_2020062915444066.pdf`
