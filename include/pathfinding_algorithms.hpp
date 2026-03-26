#pragma once

#include "pathfinding_types.hpp"

#include <string>
#include <vector>

namespace pathfinding
{
	// 统一的算法编号，便于命令行解析和对比驱动分发。
	enum class AlgorithmId
	{
		Bfs,
		AStar,
		Dijkstra,
		BranchStar,
		BranchStarClassic,
		BranchStarLegacy,
		IbpBStar
	};

	// 经典 Branch Star 的参数。
	// 这里保持轻量，先把最常用的“是否允许在绕障时回退”暴露出来。
	struct BranchStarOptions
	{
		bool allow_reverse_when_crawling = true;
	};

	// IBP-B* 的参数独立出来，避免和经典 Branch Star 混在一起。
	struct IbpBStarOptions
	{
		int	 wait_layers = 2;
		bool enable_local_zigzag_expansion = true;
		bool enable_maze_rescue = true;
		int	 zigzag_threshold = 2;
	};

	// 预留统一的算法参数入口，方便以后继续往里加更多算法的专属选项。
	struct AlgorithmOptions
	{
		BranchStarOptions branch_star;
		IbpBStarOptions	 ibp_bstar;
	};

	std::string			   GetAlgorithmName( AlgorithmId algorithm_id );
	std::vector<AlgorithmId> GetDefaultAlgorithms();
	std::vector<AlgorithmId> ParseAlgorithmList( const std::string& csv_list );

	SearchResult RunBfs( const Grid& grid, Position start, Position goal );
	SearchResult RunAStar( const Grid& grid, Position start, Position goal );
	SearchResult RunDijkstra( const Grid& grid, Position start, Position goal );
	SearchResult RunBranchStar( const Grid& grid, Position start, Position goal, const BranchStarOptions& options );
	SearchResult RunBranchStarClassic( const Grid& grid, Position start, Position goal, const BranchStarOptions& options );
	SearchResult RunBranchStarLegacy( const Grid& grid, Position start, Position goal, const BranchStarOptions& options );
	SearchResult RunIbpBStar( const Grid& grid, Position start, Position goal, const IbpBStarOptions& options );
	SearchResult RunAlgorithm( AlgorithmId algorithm_id, const Grid& grid, Position start, Position goal, const AlgorithmOptions& options );
}  // namespace pathfinding
