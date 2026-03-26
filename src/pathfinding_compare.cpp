#include "pathfinding_compare.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <ostream>
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

		// 对 JSON 字符串做最基本的转义，避免路径和错误消息破坏输出格式。
		std::string EscapeJsonString( const std::string& value )
		{
			std::string escaped;
			escaped.reserve( value.size() + 8 );
			for ( unsigned char ch : value )
			{
				switch ( ch )
				{
				case '\"': escaped += "\\\""; break;
				case '\\': escaped += "\\\\"; break;
				case '\b': escaped += "\\b"; break;
				case '\f': escaped += "\\f"; break;
				case '\n': escaped += "\\n"; break;
				case '\r': escaped += "\\r"; break;
				case '\t': escaped += "\\t"; break;
				default:
					if ( ch < 0x20 )
					{
						std::ostringstream hex_stream;
						hex_stream << "\\u" << std::hex << std::setw( 4 ) << std::setfill( '0' ) << static_cast<int>( ch );
						escaped += hex_stream.str();
					}
					else
					{
						escaped.push_back( static_cast<char>( ch ) );
					}
					break;
				}
			}
			return escaped;
		}

		// 计算 BFS 基线的最短路径长度，供摘要表和 JSON 一起计算 gap。
		int FindOptimalBfsPathLength( const std::vector<SearchResult>& results )
		{
			for ( const SearchResult& result : results )
			{
				if ( ToLowerCopy( result.algorithm_name ) == "bfs" && result.success )
				{
					return result.statistics.final_path_length;
				}
			}
			return -1;
		}

		// 把 Position 写成 JSON 对象，避免多个序列化位置点的地方重复拼接。
		void WriteJsonPosition( std::ostream& output_stream, const Position& position )
		{
			output_stream << "{\"row\":" << position.row << ",\"col\":" << position.col << "}";
		}

		// 把路径序列化为 JSON 数组。
		void WriteJsonPath( std::ostream& output_stream, const std::vector<Position>& path )
		{
			output_stream << "[";
			for ( std::size_t index = 0; index < path.size(); ++index )
			{
				if ( index != 0 )
				{
					output_stream << ",";
				}
				WriteJsonPosition( output_stream, path[ index ] );
			}
			output_stream << "]";
		}

		// 把地图网格按二维整数数组输出到 JSON。
		void WriteJsonGrid( std::ostream& output_stream, const Grid& grid )
		{
			output_stream << "[";
			for ( std::size_t row = 0; row < grid.size(); ++row )
			{
				if ( row != 0 )
				{
					output_stream << ",";
				}
				output_stream << "[";
				for ( std::size_t col = 0; col < grid[ row ].size(); ++col )
				{
					if ( col != 0 )
					{
						output_stream << ",";
					}
					output_stream << grid[ row ][ col ];
				}
				output_stream << "]";
			}
			output_stream << "]";
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
				config.use_staggered_wall_map = false;
			}
			else if ( token == "--random" )
			{
				if ( arg_stream.cursor + 2 >= arg_stream.tokens.size() )
				{
					throw std::runtime_error( "--random W H P requires 3 values" );
				}
				config.use_random_map = true;
				config.use_maze_map = false;
				config.use_staggered_wall_map = false;
				config.map_file_path.clear();
				config.random_width = std::stoi( arg_stream.Get() );
				config.random_height = std::stoi( arg_stream.Get() );
				config.random_wall_probability = std::stod( arg_stream.Get() );
			}
			else if ( token == "--staggered-walls" )
			{
				if ( arg_stream.cursor + 2 >= arg_stream.tokens.size() )
				{
					throw std::runtime_error( "--staggered-walls W H P requires 3 values" );
				}
				config.use_random_map = false;
				config.use_maze_map = false;
				config.use_staggered_wall_map = true;
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
				config.use_staggered_wall_map = false;
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
			else if ( token == "--json" )
			{
				config.output_json = true;
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
					"Usage: pathfinding_compare [--map FILE | --random W H P | --staggered-walls W H P | --maze W H] [--seed N] "
					"[--algorithms bfs,astar,dijkstra,bstar_paper,bstar_greedy_lite,bstar_robust,ibp_bstar] [--sx R --sy C --ex R --ey C] "
					"[--ibp-wait N] [--ibp-paper-strict] [--ibp-zigzag-threshold N] [--ibp-rescue] [--no-ibp-rescue] "
					"[--branch-no-reverse] [--print-path] [--no-summary] [--ascii|--unicode] [--arrow] [--json]" );
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
		if ( config.use_staggered_wall_map )
		{
			if ( config.random_width < 9 || config.random_height < 7 )
			{
				throw std::runtime_error( "Staggered wall map requires width >= 9 and height >= 7" );
			}
			if ( config.random_wall_probability < 0.0 || config.random_wall_probability > 1.0 )
			{
				throw std::runtime_error( "Staggered wall density must be in [0, 1]" );
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

	// 生成一段人类可读的地图来源描述。
	std::string DescribeMapSource( const CompareConfig& config )
	{
		std::ostringstream description_stream;
		if ( config.use_maze_map )
		{
			description_stream << "perfect maze " << config.random_width << "x" << config.random_height << " seed=" << config.seed;
		}
		else if ( config.use_staggered_wall_map )
		{
			description_stream << "staggered wall field " << config.random_width << "x" << config.random_height << " density=" << config.random_wall_probability << " seed=" << config.seed;
		}
		else if ( config.use_random_map && config.map_file_path.empty() )
		{
			description_stream << "random grid " << config.random_width << "x" << config.random_height << " p=" << config.random_wall_probability << " seed=" << config.seed;
		}
		else
		{
			description_stream << "file " << config.map_file_path;
		}
		return description_stream.str();
	}

	// 打印一张紧凑的对比摘要表。
	// 如果结果里包含 BFS，就顺带给出其它算法相对 BFS 的路径长度差值。
	void PrintCompareSummary( const std::vector<SearchResult>& results )
	{
		const int optimal_path_length = FindOptimalBfsPathLength( results );

		std::cout << "\n=== Compare Summary ===\n";
		std::cout << std::left << std::setw( 18 ) << "Algorithm" << std::setw( 10 ) << "Success" << std::setw( 10 ) << "PathLen" << std::setw( 12 ) << "Expanded"
				  << std::setw( 12 ) << "Time(us)";
		if ( optimal_path_length != -1 )
		{
			std::cout << std::setw( 10 ) << "GapToBFS";
		}
		std::cout << "\n";

		for ( const SearchResult& result : results )
		{
			std::cout << std::left << std::setw( 18 ) << result.algorithm_name << std::setw( 10 ) << ( result.success ? "yes" : "no" )
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

	// 把完整结果写成 JSON，方便 Python GUI 和其它脚本直接消费。
	void PrintCompareJson( const CompareConfig& config, const MapInstance& map_instance, const std::vector<SearchResult>& results, std::ostream& output_stream )
	{
		const int optimal_path_length = FindOptimalBfsPathLength( results );
		const int grid_height = static_cast<int>( map_instance.grid.size() );
		const int grid_width = grid_height > 0 ? static_cast<int>( map_instance.grid.front().size() ) : 0;

		output_stream << "{";
		output_stream << "\"ok\":true,";
		output_stream << "\"map_source\":{";
		if ( config.use_maze_map )
		{
			output_stream << "\"kind\":\"maze\",";
		}
		else if ( config.use_staggered_wall_map )
		{
			output_stream << "\"kind\":\"staggered_walls\",";
		}
		else if ( config.use_random_map && config.map_file_path.empty() )
		{
			output_stream << "\"kind\":\"random\",";
		}
		else
		{
			output_stream << "\"kind\":\"file\",";
		}
		output_stream << "\"description\":\"" << EscapeJsonString( DescribeMapSource( config ) ) << "\",";
		output_stream << "\"seed\":" << config.seed << ",";
		output_stream << "\"width\":" << grid_width << ",";
		output_stream << "\"height\":" << grid_height;
		if ( config.use_staggered_wall_map )
		{
			output_stream << ",\"barrier_density\":" << config.random_wall_probability;
		}
		if ( config.use_random_map && config.map_file_path.empty() )
		{
			output_stream << ",\"wall_probability\":" << config.random_wall_probability;
		}
		if ( !config.map_file_path.empty() )
		{
			output_stream << ",\"file_path\":\"" << EscapeJsonString( config.map_file_path ) << "\"";
		}
		output_stream << "},";

		output_stream << "\"map\":{";
		output_stream << "\"width\":" << grid_width << ",";
		output_stream << "\"height\":" << grid_height << ",";
		output_stream << "\"start\":";
		WriteJsonPosition( output_stream, map_instance.start );
		output_stream << ",\"goal\":";
		WriteJsonPosition( output_stream, map_instance.goal );
		output_stream << ",\"grid\":";
		WriteJsonGrid( output_stream, map_instance.grid );
		output_stream << "},";

		output_stream << "\"algorithms\":[";
		for ( std::size_t index = 0; index < results.size(); ++index )
		{
			const SearchResult& result = results[ index ];
			if ( index != 0 )
			{
				output_stream << ",";
			}

			output_stream << "{";
			output_stream << "\"name\":\"" << EscapeJsonString( result.algorithm_name ) << "\",";
			output_stream << "\"success\":" << ( result.success ? "true" : "false" ) << ",";
			output_stream << "\"path_length\":" << result.statistics.final_path_length << ",";
			output_stream << "\"expanded\":" << result.statistics.expanded_node_count << ",";
			output_stream << "\"time_us\":" << result.statistics.elapsed_microseconds << ",";
			if ( optimal_path_length != -1 && result.success )
			{
				output_stream << "\"gap_to_bfs\":" << ( result.statistics.final_path_length - optimal_path_length ) << ",";
			}
			else
			{
				output_stream << "\"gap_to_bfs\":null,";
			}
			output_stream << "\"meet_position\":";
			WriteJsonPosition( output_stream, result.meet_position );
			output_stream << ",\"path\":";
			WriteJsonPath( output_stream, result.path );
			output_stream << "}";
		}
		output_stream << "]";
		output_stream << "}";
	}

	// JSON 模式下的错误输出，供外部界面直接读取。
	void PrintCompareJsonError( const std::string& error_message, std::ostream& output_stream )
	{
		output_stream << "{\"ok\":false,\"error\":\"" << EscapeJsonString( error_message ) << "\"}";
	}
}  // namespace pathfinding
