#include "pathfinding_compare.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace pathfinding
{
	namespace
	{
		struct ArgStream
		{
			std::vector<std::string> tokens;
			std::size_t				 cursor = 0;

			// 判断参数流里是否还有未消费的 token。
			bool HasNext() const
			{
				return cursor < tokens.size();
			}

			// 取出当前 token，并把游标移动到下一个位置。
			std::string Get()
			{
				return cursor < tokens.size() ? tokens[ cursor++ ] : "";
			}
		};

		// 检查用户是否把起点或终点覆盖参数写完整。
		// 例如只写 --sx 不写 --sy 时，这里会提前报错。
		void ValidateOverridePosition( const std::optional<Position>& position, const std::string& name )
		{
			if ( !position )
			{
				return;
			}
			if ( position->row < 0 || position->col < 0 )
			{
				throw std::runtime_error( name + " override must provide both row and col" );
			}
		}

		// 把字符串转换成小写副本，方便做大小写不敏感比较。
		std::string ToLowerCopy( std::string value )
		{
			std::transform( value.begin(), value.end(), value.begin(), []( unsigned char ch ) { return static_cast<char>( std::tolower( ch ) ); } );
			return value;
		}

		// 把路径长度格式化成表格里的文本。
		// 对失败算法返回 "-"，便于对比摘要阅读。
		std::string FormatPathLength( bool success, long long value )
		{
			if ( !success )
			{
				return "-";
			}
			return std::to_string( value );
		}
	}  // namespace

	// 解析对比程序的命令行参数，并把结果写入 CompareConfig。
	// 这个函数只负责“解释参数”，不直接负责建图和跑算法。
	void ParseCompareArgs( int argc, char** argv, CompareConfig& config )
	{
		ArgStream arg_stream;
		arg_stream.tokens.assign( argv + 1, argv + argc );

		// 读取某个选项后面的值，并在缺失时立即报错。
		auto ExpectValue = [ & ]( const std::string& flag ) {
			if ( !arg_stream.HasNext() )
			{
				throw std::runtime_error( "Missing value after " + flag );
			}
			return arg_stream.Get();
		};

		while ( arg_stream.HasNext() )
		{
			const std::string token = arg_stream.Get();
			if ( token == "--map" )
			{
				config.map_file_path = ExpectValue( token );
				config.use_random_map = false;
				config.use_maze_map = false;
			}
			else if ( token == "--random" )
			{
				if ( arg_stream.cursor + 2 >= arg_stream.tokens.size() )
				{
					throw std::runtime_error( "--random W H P requires 3 values" );
				}
				config.use_random_map = true;
				config.use_maze_map = false;
				config.map_file_path.clear();
				config.random_width = std::stoi( arg_stream.Get() );
				config.random_height = std::stoi( arg_stream.Get() );
				config.random_wall_probability = std::stod( arg_stream.Get() );
			}
			else if ( token == "--maze" )
			{
				if ( arg_stream.cursor + 1 >= arg_stream.tokens.size() )
				{
					throw std::runtime_error( "--maze W H requires 2 values" );
				}
				config.use_random_map = false;
				config.use_maze_map = true;
				config.map_file_path.clear();
				config.random_width = std::stoi( arg_stream.Get() );
				config.random_height = std::stoi( arg_stream.Get() );
			}
			else if ( token == "--seed" )
			{
				config.seed = static_cast<std::uint32_t>( std::stoul( ExpectValue( token ) ) );
			}
			else if ( token == "--sx" )
			{
				if ( !config.cli_start_override )
				{
					config.cli_start_override = Position {};
				}
				config.cli_start_override->row = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--sy" )
			{
				if ( !config.cli_start_override )
				{
					config.cli_start_override = Position {};
				}
				config.cli_start_override->col = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--ex" )
			{
				if ( !config.cli_goal_override )
				{
					config.cli_goal_override = Position {};
				}
				config.cli_goal_override->row = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--ey" )
			{
				if ( !config.cli_goal_override )
				{
					config.cli_goal_override = Position {};
				}
				config.cli_goal_override->col = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--algorithms" )
			{
				config.algorithms = ParseAlgorithmList( ExpectValue( token ) );
			}
			else if ( token == "--ibp-wait" || token == "--wait" )
			{
				config.algorithm_options.ibp_bstar.wait_layers = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--ibp-zigzag-threshold" || token == "--zigzag-threshold" )
			{
				config.algorithm_options.ibp_bstar.zigzag_threshold = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--ibp-paper-strict" || token == "--paper-strict" )
			{
				config.algorithm_options.ibp_bstar.enable_local_zigzag_expansion = false;
				config.algorithm_options.ibp_bstar.enable_maze_rescue = false;
			}
			else if ( token == "--ibp-zigzag" || token == "--zigzag" )
			{
				config.algorithm_options.ibp_bstar.enable_local_zigzag_expansion = true;
			}
			else if ( token == "--ibp-rescue" )
			{
				config.algorithm_options.ibp_bstar.enable_maze_rescue = true;
			}
			else if ( token == "--no-ibp-rescue" )
			{
				config.algorithm_options.ibp_bstar.enable_maze_rescue = false;
			}
			else if ( token == "--branch-no-reverse" )
			{
				config.algorithm_options.branch_star.allow_reverse_when_crawling = false;
			}
			else if ( token == "--print-path" )
			{
				config.print_path = true;
			}
			else if ( token == "--no-print" )
			{
				config.print_path = false;
			}
			else if ( token == "--no-summary" )
			{
				config.print_summary = false;
			}
			else if ( token == "--ascii" )
			{
				config.render_style.use_ascii_glyphs = true;
			}
			else if ( token == "--unicode" )
			{
				config.render_style.use_ascii_glyphs = false;
			}
			else if ( token == "--arrow" )
			{
				config.render_style.print_arrows = true;
			}
			else if ( token == "--help" || token == "-h" )
			{
				throw std::runtime_error(
					"Usage: pathfinding_compare [--map FILE | --random W H P | --maze W H] [--seed N] "
					"[--algorithms bfs,astar,dijkstra,branchstar,ibp_bstar] [--sx R --sy C --ex R --ey C] "
					"[--ibp-wait N] [--ibp-paper-strict] [--ibp-zigzag-threshold N] [--ibp-rescue] [--no-ibp-rescue] "
					"[--branch-no-reverse] [--print-path] [--no-summary] [--ascii|--unicode] [--arrow]" );
			}
			else
			{
				throw std::runtime_error( "Unknown option: " + token );
			}
		}

		if ( config.use_maze_map )
		{
			if ( config.random_width < 3 || config.random_height < 3 )
			{
				throw std::runtime_error( "Maze width and height must be at least 3" );
			}
		}
		if ( config.use_random_map )
		{
			if ( config.random_width <= 0 || config.random_height <= 0 )
			{
				throw std::runtime_error( "Random width and height must be positive" );
			}
			if ( config.random_wall_probability < 0.0 || config.random_wall_probability > 1.0 )
			{
				throw std::runtime_error( "Random wall probability must be in [0, 1]" );
			}
		}
		ValidateOverridePosition( config.cli_start_override, "Start" );
		ValidateOverridePosition( config.cli_goal_override, "Goal" );
	}

	// 打印一张紧凑的对比摘要表。
	// 如果结果里包含 BFS，就顺带给出其它算法相对 BFS 的路径长度差值。
	void PrintCompareSummary( const std::vector<SearchResult>& results )
	{
		int optimal_path_length = -1;
		for ( const SearchResult& result : results )
		{
			if ( ToLowerCopy( result.algorithm_name ) == "bfs" && result.success )
			{
				optimal_path_length = result.statistics.final_path_length;
				break;
			}
		}

		std::cout << "\n=== Compare Summary ===\n";
		std::cout << std::left << std::setw( 14 ) << "Algorithm" << std::setw( 10 ) << "Success" << std::setw( 10 ) << "PathLen" << std::setw( 12 ) << "Expanded"
				  << std::setw( 12 ) << "Time(us)";
		if ( optimal_path_length != -1 )
		{
			std::cout << std::setw( 10 ) << "GapToBFS";
		}
		std::cout << "\n";

		for ( const SearchResult& result : results )
		{
			std::cout << std::left << std::setw( 14 ) << result.algorithm_name << std::setw( 10 ) << ( result.success ? "yes" : "no" )
					  << std::setw( 10 ) << FormatPathLength( result.success, result.statistics.final_path_length ) << std::setw( 12 ) << result.statistics.expanded_node_count
					  << std::setw( 12 ) << result.statistics.elapsed_microseconds;
			if ( optimal_path_length != -1 )
			{
				if ( result.success )
				{
					std::cout << std::setw( 10 ) << ( result.statistics.final_path_length - optimal_path_length );
				}
				else
				{
					std::cout << std::setw( 10 ) << "-";
				}
			}
			std::cout << "\n";
		}
	}
}  // namespace pathfinding
