#include "pathfinding_algorithms.hpp"
#include "pathfinding_compare.hpp"
#include "pathfinding_grid.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
	// 根据 CompareConfig 构造本轮实验要用的地图实例。
	// 这里统一处理三种来源：文件地图、随机障碍图、完美迷宫。
	pathfinding::MapInstance BuildMapInstance( const pathfinding::CompareConfig& config )
	{
		using namespace pathfinding;

		MapInstance map_instance;
		if ( config.use_maze_map )
		{
			map_instance = GeneratePerfectMaze( config.random_width, config.random_height, config.seed );
		}
		else if ( config.use_random_map && config.map_file_path.empty() )
		{
			map_instance = GenerateRandomMap( config.random_width, config.random_height, config.random_wall_probability, config.seed );
		}
		else
		{
			map_instance = LoadMapFromFile( config.map_file_path, config.render_style );
		}

		if ( config.cli_start_override )
		{
			map_instance.start = *config.cli_start_override;
		}
		if ( config.cli_goal_override )
		{
			map_instance.goal = *config.cli_goal_override;
		}

		if ( !IsInsideGrid( map_instance.grid, map_instance.start ) || !IsInsideGrid( map_instance.grid, map_instance.goal ) )
		{
			throw std::runtime_error( "Start or goal is outside the grid" );
		}
		if ( !IsPassable( map_instance.grid, map_instance.start.row, map_instance.start.col ) || !IsPassable( map_instance.grid, map_instance.goal.row, map_instance.goal.col ) )
		{
			throw std::runtime_error( "Start or goal is blocked by an obstacle" );
		}
		return map_instance;
	}

	// 打印本轮实验所使用的地图来源摘要。
	// 这样在保存控制台输出时，结果本身就带有最关键的实验上下文。
	void PrintMapSourceSummary( const pathfinding::CompareConfig& config )
	{
		using namespace pathfinding;

		if ( config.use_maze_map )
		{
			std::cout << "Map source: perfect maze " << config.random_width << "x" << config.random_height << " seed=" << config.seed << "\n";
		}
		else if ( config.use_random_map && config.map_file_path.empty() )
		{
			std::cout << "Map source: random grid " << config.random_width << "x" << config.random_height << " p=" << config.random_wall_probability << " seed=" << config.seed << "\n";
		}
		else
		{
			std::cout << "Map source: file " << config.map_file_path << "\n";
		}
	}
}  // namespace

// 程序总入口：
// 1. 解析参数
// 2. 构造地图
// 3. 依次运行选中的算法
// 4. 输出摘要表和可选路径
int main( int argc, char** argv )
{
	std::ios::sync_with_stdio( false );
	std::cin.tie( nullptr );

	using namespace pathfinding;

	CompareConfig config;
	try
	{
		ParseCompareArgs( argc, argv, config );
		const MapInstance map_instance = BuildMapInstance( config );

		PrintMapSourceSummary( config );
		std::cout << "Start=(" << map_instance.start.row << ", " << map_instance.start.col << ") "
				  << "Goal=(" << map_instance.goal.row << ", " << map_instance.goal.col << ")\n";

		std::vector<SearchResult> results;
		results.reserve( config.algorithms.size() );
		for ( AlgorithmId algorithm_id : config.algorithms )
		{
			results.push_back( RunAlgorithm( algorithm_id, map_instance.grid, map_instance.start, map_instance.goal, config.algorithm_options ) );
		}

		if ( config.print_summary )
		{
			PrintCompareSummary( results );
		}

		if ( config.print_path )
		{
			for ( const SearchResult& result : results )
			{
				std::cout << "\n=== " << result.algorithm_name << " ===\n";
				std::cout << "success=" << ( result.success ? "yes" : "no" ) << " path_length=" << result.statistics.final_path_length
						  << " expanded=" << result.statistics.expanded_node_count << " time_us=" << result.statistics.elapsed_microseconds << "\n";
				if ( result.success )
				{
					RenderGridWithPath( map_instance.grid, result.path, map_instance.start, map_instance.goal, config.render_style );
				}
				else
				{
					std::cout << "No valid path to render.\n";
				}
			}
		}
	}
	catch ( const std::exception& error )
	{
		std::cerr << "Error: " << error.what() << "\n";
		return 1;
	}

	return 0;
}
