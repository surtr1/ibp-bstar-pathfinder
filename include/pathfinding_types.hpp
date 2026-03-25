#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pathfinding
{
	// 统一的网格坐标类型，所有算法和地图模块都使用 (row, col) 表示位置。
	// 这样后续做算法对比时，不需要在不同模块之间来回做坐标类型转换。
	struct Position
	{
		int row = -1;
		int col = -1;

		// 判断两个坐标是否表示同一个网格位置。
		bool operator==( const Position& other ) const
		{
			return row == other.row && col == other.col;
		}
	};

	// 栅格地图约定：0 表示可通行，1 表示障碍。
	using Grid = std::vector<std::vector<int>>;

	// 算法对比时最常用的基础统计字段。
	// elapsed_microseconds 方便直接输出耗时对比表。
	struct SearchStatistics
	{
		int		  expanded_node_count = 0;
		int		  final_path_length = 0;
		long long elapsed_microseconds = 0;
	};

	// 统一的算法运行结果。
	// 对单向算法而言 meet_position 通常直接等于终点；
	// 对双向 / 特殊算法而言，可用于记录更有意义的会合点或关键节点。
	struct SearchResult
	{
		std::string			  algorithm_name;
		std::vector<Position> path;
		Position			  meet_position { -1, -1 };
		SearchStatistics	  statistics;
		bool				  success = false;
	};
}  // namespace pathfinding
