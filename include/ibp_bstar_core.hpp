#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace IBP_BStarAlgorithm
{
	// 表示网格中的一个单元坐标，使用 (row, col) 形式。
	// 该结构会在路径重建和队列传递中频繁复制，因此保持轻量。
	struct CellPosition
	{
		int row = -1;
		int col = -1;

		bool operator==( const CellPosition& other ) const
		{
			return row == other.row && col == other.col;
		}
	};

	// 栅格地图：0 表示可通行，1 表示障碍。
	// 这种简单表示法便于后续把同一张地图交给多个算法统一对比。
	using Grid = std::vector<std::vector<int>>;

	// 算法运行后的基础统计信息。
	// 这些字段刻意保持通用，便于和 BFS / A* / Dijkstra 等算法做横向比较。
	struct SearchStatistics
	{
		int expanded_node_count = 0;
		int final_path_length = 0;
	};

	// 算法返回结果的统一封装。
	// meet_position 对双向搜索尤其有意义，也适合在调试和评测报告里输出。
	struct SearchOutcome
	{
		std::vector<CellPosition> final_path;
		CellPosition			  meet_position { -1, -1 };
		SearchStatistics		  statistics;
		bool					  success = false;
	};

	// 只影响算法核心行为的参数集合。
	// 与命令行、输出格式等应用层配置分离，避免核心逻辑依赖外部壳层。
	struct AlgorithmOptions
	{
		bool enable_local_zigzag_expansion = false;
		int	 zigzag_threshold = 8;
	};

	// 这些基础工具函数会被算法核心、渲染层和未来的 benchmark 驱动共同复用。
	int				 ToLinearIndex( int row, int col, int grid_width );
	std::pair<int, int> FromLinearIndex( int linear_index, int grid_width );
	bool			 IsCellPassable( const Grid& grid, int row, int col );
	bool			 IsCellBlocked( const Grid& grid, int row, int col );
	char			 OppositeDirection( char direction_char );
	std::pair<char, char> LeftRightDirections( char direction_char );
	char			 ChooseGreedyDirection( int current_row, int current_col, int goal_row, int goal_col );

	// 校验路径是否合法：
	// 1. 每个点都在地图内
	// 2. 每个点都不是障碍
	// 3. 相邻两个点必须是四连通的一步移动
	bool			 ValidatePathContiguity( const Grid& grid, const std::vector<CellPosition>& path );

	// 论文风格的 IBP-B* 核心实现。
	// 当 options.enable_local_zigzag_expansion 为 true 时，会在局部绕障失败后尝试更激进的侧向扩展。
	SearchOutcome	 RunIbpBStar( const Grid& grid, CellPosition start_pos, CellPosition goal_pos, int wait_layers, const AlgorithmOptions& options = {} );

	// 面向迷宫场景的补救式搜索。
	// 该搜索显式记录“上一步方向 + 转向偏好 + 是否刚转角”等状态，以提高走廊和拐角中的存活率。
	SearchOutcome	 RunZigzagMazeRescue( const Grid& grid, CellPosition start_pos, CellPosition goal_pos );

	// 实用增强入口：
	// 先运行 IBP-B*，若失败或路径无效，再自动回退到 Zigzag 迷宫补救搜索。
	SearchOutcome	 RunIbpBStarZigzagEnhanced( const Grid& grid, CellPosition start_pos, CellPosition goal_pos, int wait_layers, const AlgorithmOptions& options = {} );
}  // namespace IBP_BStarAlgorithm
