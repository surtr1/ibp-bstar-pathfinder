#include "ibp_bstar_app.hpp"
#include "ibp_bstar_core.hpp"

#include <exception>
#include <iostream>
#include <tuple>

namespace IBP_BStarAlgorithm
{
	namespace
	{
		void RunMazeDemoZigzagEnhanced( const RunConfig& config )
		{
			// 迷宫 demo 的目标不是替代正式 benchmark，
			// 而是快速观察增强版在 maze 场景中的行为，并与 BFS 最短路长度做一个直观对照。
			CellPosition maze_start;
			CellPosition maze_goal;
			Grid		 maze_grid = GeneratePerfectMazeGrid( config.random_map_width, config.random_map_height, config.random_seed ^ 0xA53E1234u, maze_start, maze_goal );

			const int bfs_len = ShortestPathLength_BFS( maze_grid, maze_start, maze_goal );
			if ( bfs_len == -1 )
			{
				std::cout << "[maze] UNSOLVABLE MAZE (BFS could not reach E)\n";
				RenderGridWithPath( maze_grid, {}, maze_start, maze_goal, config.render_style );
				return;
			}

			SearchOutcome maze_out = RunIbpBStarZigzagEnhanced( maze_grid, maze_start, maze_goal, config.flush_wait_layers, config.algorithm_options );
			if ( maze_out.final_path.empty() )
			{
				std::cout << "[maze] NO PATH FOUND by IBP-B* zigzag mode\n";
			}
			else if ( !ValidatePathContiguity( maze_grid, maze_out.final_path ) )
			{
				std::cout << "[maze] INVALID PATH (broken chain)\n";
			}

			if ( config.print_stats )
			{
				std::cout << "\n[maze] path_length=" << maze_out.statistics.final_path_length << " meet=(" << maze_out.meet_position.row << ", " << maze_out.meet_position.col << ")"
						  << " expanded=" << maze_out.statistics.expanded_node_count << "\n";
				std::cout << "[maze] bfs_search_length=" << bfs_len << "\n";
			}
			if ( config.print_path )
			{
				std::cout << "\n[maze] =====================\n";
				RenderGridWithPath( maze_grid, maze_out.final_path, maze_start, maze_goal, config.render_style );
			}
		}
	}  // namespace
}  // namespace IBP_BStarAlgorithm

int main( int argc, char** argv )
{
	// main 函数现在只保留“应用壳层”职责：
	// 解析参数、准备地图、调用算法、输出结果。
	// 真正的算法逻辑已经移到 ibp_bstar_core.* 中，方便后续与其它算法做统一对比。
	std::ios::sync_with_stdio( false );
	std::cin.tie( nullptr );

	using namespace IBP_BStarAlgorithm;

	RunConfig config;
	if ( argc > 1 )
	{
		try
		{
			ParseArgs( argc, argv, config );
		}
		catch ( const std::exception& e )
		{
			std::cerr << "Error parsing args: " << e.what() << "\n";
			return 1;
		}
	}

	int attempt_counter = 0;
	while ( true )
	{
		// 若启用了随机地图重抽，则在失败时继续循环，直到找到可解样例或达到上限。
		++attempt_counter;
		Grid		 grid;
		CellPosition start_pos;
		CellPosition goal_pos;
		try
		{
			if ( config.use_random_map && config.map_file_path.empty() )
			{
				// 未指定地图文件时，使用随机图模式。
				grid = GenerateRandomGrid( config.random_map_width, config.random_map_height, config.random_wall_probability, config.random_seed + attempt_counter - 1, start_pos, goal_pos );
			}
			else
			{
				// 若指定了 map 文件，则优先从文件读取。
				// 之后再用命令行坐标覆盖文件中的 S / E（如果用户显式提供了覆盖参数）。
				std::tie( grid, start_pos, goal_pos ) = LoadGridFromFile( config.map_file_path, config.render_style );
				if ( config.cli_start_override )
					start_pos = *config.cli_start_override;
				if ( config.cli_goal_override )
					goal_pos = *config.cli_goal_override;
			}
		}
		catch ( const std::exception& e )
		{
			std::cerr << "Map load error: " << e.what() << "\n";
			return 1;
		}

		SearchOutcome outcome = RunIbpBStarZigzagEnhanced( grid, start_pos, goal_pos, config.flush_wait_layers, config.algorithm_options );
		if ( !outcome.success || !ValidatePathContiguity( grid, outcome.final_path ) )
		{
			if ( config.reroll_until_solvable && config.use_random_map && attempt_counter < config.reroll_max_attempts )
			{
				// 对随机地图而言，失败不一定说明算法有问题，可能只是当前图无解。
				// 因此允许按配置继续抽下一张图。
				continue;
			}

			if ( !outcome.success )
			{
				std::cout << "No path is reachable for the current obstacle!\n";
			}
			else
			{
				std::cout << "Invalid path reconstructed (broken chain).\n";
				if ( config.print_invalid_path && config.print_path )
				{
					RenderGridWithPath( grid, outcome.final_path, start_pos, goal_pos, config.render_style );
				}
			}
		}

		if ( config.print_stats )
		{
			std::cout << "path_length=" << outcome.statistics.final_path_length << " meet=(" << outcome.meet_position.row << ", " << outcome.meet_position.col << ")"
					  << " expanded=" << outcome.statistics.expanded_node_count << " tries=" << attempt_counter << "\n";
		}
		if ( config.print_path )
		{
			RenderGridWithPath( grid, outcome.final_path, start_pos, goal_pos, config.render_style );
		}
		if ( config.run_maze_demo )
		{
			RunMazeDemoZigzagEnhanced( config );
		}
		return 0;
	}
}
