#pragma once

#include "pathfinding_algorithms.hpp"
#include "pathfinding_grid.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace pathfinding
{
	// CompareConfig 只描述“如何做对比实验”，而不绑定具体算法实现细节。
	// main 函数可以把它当作总开关，再把工作分派给地图层和算法层。
	struct CompareConfig
	{
		bool					use_random_map = true;
		bool					use_maze_map = false;
		int						random_width = 64;
		int						random_height = 64;
		double					random_wall_probability = 0.25;
		std::uint32_t			seed = 12345u;
		std::string				map_file_path;
		std::optional<Position> cli_start_override;
		std::optional<Position> cli_goal_override;
		bool					print_path = false;
		bool					print_summary = true;
		RenderStyle				render_style;
		std::vector<AlgorithmId> algorithms = GetDefaultAlgorithms();
		AlgorithmOptions		algorithm_options;
	};

	// 解析对比程序的命令行参数。
	// 这里不直接执行算法，只负责把参数转成结构化配置。
	void ParseCompareArgs( int argc, char** argv, CompareConfig& config );

	// 输出紧凑的对比摘要，便于快速观察成功率、路径长度、扩展规模和耗时。
	void PrintCompareSummary( const std::vector<SearchResult>& results );
}  // namespace pathfinding
