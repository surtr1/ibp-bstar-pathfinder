#pragma once

#include "pathfinding_algorithms.hpp"
#include "pathfinding_grid.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
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
		bool					use_staggered_wall_map = false;
		int						random_width = 64;
		int						random_height = 64;
		double					random_wall_probability = 0.25;
		std::uint32_t			seed = 12345u;
		std::string				map_file_path;
		std::optional<Position> cli_start_override;
		std::optional<Position> cli_goal_override;
		bool					print_path = false;
		bool					print_summary = true;
		bool					output_json = false;
		RenderStyle				render_style;
		std::vector<AlgorithmId> algorithms = GetDefaultAlgorithms();
		AlgorithmOptions		algorithm_options;
	};

	// 解析对比程序的命令行参数。
	// 这里不直接执行算法，只负责把参数转成结构化配置。
	void ParseCompareArgs( int argc, char** argv, CompareConfig& config );

	// 生成一段适合人类阅读的地图来源摘要。
	// 控制台模式和 JSON 模式都会复用这段描述。
	std::string DescribeMapSource( const CompareConfig& config );

	// 输出紧凑的对比摘要，便于快速观察成功率、路径长度、扩展规模和耗时。
	void PrintCompareSummary( const std::vector<SearchResult>& results );

	// 把一次完整的对比结果序列化为 JSON，供外部脚本或 GUI 调用。
	void PrintCompareJson( const CompareConfig& config, const MapInstance& map_instance, const std::vector<SearchResult>& results, std::ostream& output_stream );

	// 当 JSON 模式下发生错误时，统一输出结构化错误对象。
	void PrintCompareJsonError( const std::string& error_message, std::ostream& output_stream );
}  // namespace pathfinding
