#include "ibp_bstar_app.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace IBP_BStarAlgorithm
{
	// ============================================================================
	// 应用层实现说明
	//
	// 本文件只处理“算法之外”的程序逻辑：
	// - 地图生成 / 读取
	// - 迷宫构造
	// - 控制台渲染
	// - CLI 参数解析
	// - BFS 参考实现
	//
	// 这些内容被放到 app 层，是为了让算法核心可以被单独复用。
	// ============================================================================
	namespace
	{
		// 简单线性同余随机数，用于随机障碍图生成。
		// 这里保留与旧版本相似的轻量行为，方便复现实验。
		struct LinearCongruentialRng32
		{
			std::uint32_t state;

			explicit LinearCongruentialRng32( std::uint32_t seed ) : state( seed ) {}

			std::uint32_t NextUint32()
			{
				state = ( 1664525u * state + 1013904223u );
				return state;
			}

			double NextFloatZeroToOne()
			{
				return static_cast<double>( NextUint32() ) / 4294967296.0;
			}
		};

		class UnionFind
		{
		public:
			// 迷宫生成使用并查集来做 Kruskal 式连通维护。
			explicit UnionFind( int n ) : parent_( n ), rank_( n, 0 )
			{
				for ( int idx = 0; idx < n; ++idx )
				{
					parent_[ idx ] = idx;
				}
			}

			int Find( int x )
			{
				if ( parent_[ x ] != x )
				{
					parent_[ x ] = Find( parent_[ x ] );
				}
				return parent_[ x ];
			}

			bool Connected( int x, int y )
			{
				return Find( x ) == Find( y );
			}

			void Unite( int x, int y )
			{
				int root_x = Find( x );
				int root_y = Find( y );
				if ( root_x == root_y )
					return;
				if ( rank_[ root_x ] < rank_[ root_y ] )
				{
					parent_[ root_x ] = root_y;
				}
				else if ( rank_[ root_x ] > rank_[ root_y ] )
				{
					parent_[ root_y ] = root_x;
				}
				else
				{
					parent_[ root_y ] = root_x;
					rank_[ root_x ]++;
				}
			}

		private:
			std::vector<int> parent_;
			std::vector<int> rank_;
		};

		struct ArgStream
		{
			// 极简参数流包装，避免在 ParseArgs 中手写太多游标细节。
			std::vector<std::string> tokens;
			std::size_t				 cursor = 0;

			bool HasNext() const
			{
				return cursor < tokens.size();
			}

			std::string Get()
			{
				return cursor < tokens.size() ? tokens[ cursor++ ] : "";
			}
		};

		bool IsGridShapeValid( const Grid& grid )
		{
			if ( grid.empty() || grid.front().empty() )
				return false;

			const std::size_t expected_width = grid.front().size();
			for ( const auto& row : grid )
			{
				if ( row.size() != expected_width )
					return false;
			}
			return true;
		}

		void ValidateRandomGridArguments( int width, int height, double wall_probability )
		{
			if ( width <= 0 || height <= 0 )
			{
				throw std::runtime_error( "Random grid width and height must be positive" );
			}
			if ( wall_probability < 0.0 || wall_probability > 1.0 )
			{
				throw std::runtime_error( "Random wall probability must be in [0, 1]" );
			}
		}

		void ValidateOverridePosition( const std::optional<CellPosition>& position, const std::string& name )
		{
			if ( !position )
				return;
			if ( position->row < 0 || position->col < 0 )
			{
				throw std::runtime_error( name + " override must provide both row and col" );
			}
		}
	}  // namespace

	Grid GenerateRandomGrid( int width, int height, double wall_probability, std::uint32_t seed, CellPosition& out_start, CellPosition& out_goal )
	{
		// 生成一张带随机障碍的网格，并强制保证左上角起点、右下角终点可通行。
		// 这样即使障碍概率很高，也不会出现起终点本身就是墙的无意义样例。
		ValidateRandomGridArguments( width, height, wall_probability );
		LinearCongruentialRng32 rng( seed );
		Grid					grid( height, std::vector<int>( width, 0 ) );
		for ( int row = 0; row < height; ++row )
		{
			for ( int col = 0; col < width; ++col )
			{
				if ( rng.NextFloatZeroToOne() < wall_probability )
				{
					grid[ row ][ col ] = 1;
				}
			}
		}
		out_start = { 0, 0 };
		out_goal = { height - 1, width - 1 };
		grid[ out_start.row ][ out_start.col ] = 0;
		grid[ out_goal.row ][ out_goal.col ] = 0;
		return grid;
	}

	std::tuple<Grid, CellPosition, CellPosition> LoadGridFromFile( const std::string& file_path, const RenderStyle& render_style )
	{
		// 读取轻量文本地图。
		// 解析时支持 ASCII 与 Unicode 两套渲染符号，并自动识别 S / E。
		std::ifstream file_stream( file_path );
		if ( !file_stream )
		{
			throw std::runtime_error( "Failed to open map file: " + file_path );
		}

		const std::vector<std::string> wall_tokens = { render_style.wall_unicode, render_style.wall_ascii };
		const std::vector<std::string> empty_tokens = { render_style.empty_unicode, render_style.empty_ascii };
		const std::vector<std::string> start_tokens = { render_style.start_unicode, render_style.start_ascii, "S", "s" };
		const std::vector<std::string> goal_tokens = { render_style.goal_unicode, render_style.goal_ascii, "E", "e" };

		auto IsOneOf = []( const std::string& seg, const std::vector<std::string>& pool ) {
			for ( const auto& token : pool )
			{
				if ( seg == token )
					return true;
			}
			return false;
		};

		std::size_t max_token_length = 1;
		auto UpdateMax = [ & ]( const std::vector<std::string>& tokens ) {
			for ( const auto& token : tokens )
			{
				max_token_length = std::max( max_token_length, token.size() );
			}
		};
		UpdateMax( wall_tokens );
		UpdateMax( empty_tokens );
		UpdateMax( start_tokens );
		UpdateMax( goal_tokens );

		Grid		 grid;
		CellPosition start_pos { -1, -1 };
		CellPosition goal_pos { -1, -1 };
		bool		 start_seen = false;
		bool		 goal_seen = false;
		std::string	 raw_line;
		int			 current_row_index = 0;

		while ( std::getline( file_stream, raw_line ) )
		{
			// 兼容 UTF-8 BOM。
			// 很多 Windows 编辑器保存文本文件时会在首行前写入 BOM，不去掉的话会被地图解析器当成非法字符。
			if ( current_row_index == 0 && raw_line.size() >= 3
				 && static_cast<unsigned char>( raw_line[ 0 ] ) == 0xEF
				 && static_cast<unsigned char>( raw_line[ 1 ] ) == 0xBB
				 && static_cast<unsigned char>( raw_line[ 2 ] ) == 0xBF )
			{
				raw_line.erase( 0, 3 );
			}

			// 忽略空白字符，这样地图文件在排版上更宽松。
			raw_line.erase( std::remove_if( raw_line.begin(), raw_line.end(), []( unsigned char ch ) { return std::isspace( ch ); } ), raw_line.end() );
			if ( raw_line.empty() )
				continue;

			std::vector<int> row_vector;
			for ( std::size_t char_pos = 0; char_pos < raw_line.size(); )
			{
				bool		token_matched = false;
				std::size_t taken_chars = 0;
				std::string token_candidate;
				const std::size_t try_limit = std::min<std::size_t>( max_token_length, raw_line.size() - char_pos );

				for ( std::size_t token_len = try_limit; token_len >= 1; --token_len )
				{
					// 这里从长 token 往短 token 试，兼容多字节渲染字符。
					token_candidate = raw_line.substr( char_pos, token_len );
					if ( IsOneOf( token_candidate, wall_tokens ) || IsOneOf( token_candidate, empty_tokens ) || IsOneOf( token_candidate, start_tokens )
						 || IsOneOf( token_candidate, goal_tokens ) )
					{
						taken_chars = token_len;
						token_matched = true;
						break;
					}
					if ( token_len == 1 )
						break;
				}

				if ( !token_matched )
				{
					// 一旦出现无法识别的字符，就直接报错。
					// 这里保留行号和位置，方便快速定位地图文件中的坏字符。
					throw std::runtime_error( "Bad glyph at row " + std::to_string( current_row_index ) + ", pos " + std::to_string( char_pos ) );
				}

				if ( IsOneOf( token_candidate, wall_tokens ) )
				{
					row_vector.push_back( 1 );
				}
				else if ( IsOneOf( token_candidate, empty_tokens ) )
				{
					row_vector.push_back( 0 );
				}
				else if ( IsOneOf( token_candidate, start_tokens ) )
				{
					if ( start_seen )
					{
						throw std::runtime_error( "Map contains multiple start markers" );
					}
					start_pos = { current_row_index, static_cast<int>( row_vector.size() ) };
					start_seen = true;
					row_vector.push_back( 0 );
				}
				else
				{
					if ( goal_seen )
					{
						throw std::runtime_error( "Map contains multiple goal markers" );
					}
					goal_pos = { current_row_index, static_cast<int>( row_vector.size() ) };
					goal_seen = true;
					row_vector.push_back( 0 );
				}

				char_pos += taken_chars;
			}

			grid.push_back( std::move( row_vector ) );
			++current_row_index;
		}

		if ( !IsGridShapeValid( grid ) )
		{
			throw std::runtime_error( "Map rows must have the same width and the map must not be empty" );
		}
		if ( start_pos.row == -1 || goal_pos.row == -1 )
		{
			throw std::runtime_error( "Map needs S/E" );
		}
		return { grid, start_pos, goal_pos };
	}

	Grid GeneratePerfectMazeGrid( int width, int height, std::uint32_t seed, CellPosition& out_start, CellPosition& out_goal )
	{
		// 生成完美迷宫：
		// 1. 先构造全墙网格
		// 2. 用 Kruskal 打通墙体
		// 3. 再用两次 BFS 近似找一对距离较远的起终点
		std::mt19937 prng( seed );
		if ( width < 3 || height < 3 )
		{
			throw std::runtime_error( "Maze size too small" );
		}

		const int cell_width = ( width - 1 ) / 2;
		const int cell_height = ( height - 1 ) / 2;
		const int total_cells = cell_width * cell_height;
		if ( total_cells == 1 )
		{
			Grid grid( height, std::vector<int>( width, 1 ) );
			grid[ 1 ][ 1 ] = 0;
			out_start = { 1, 1 };
			out_goal = { 1, 1 };
			return grid;
		}

		Grid	  grid( height, std::vector<int>( width, 1 ) );
		UnionFind union_find( total_cells );

		struct Wall
		{
			CellPosition wall_pos;
			int			 cell1_idx = -1;
			int			 cell2_idx = -1;
		};

		std::vector<Wall> walls;
		auto cellToIndex = [ cell_width ]( int cell_row, int cell_col ) {
			return cell_row * cell_width + cell_col;
		};
		auto gridToCell = []( int grid_r, int grid_c ) -> std::pair<int, int> {
			return { ( grid_r - 1 ) / 2, ( grid_c - 1 ) / 2 };
		};

		for ( int cell_r = 0; cell_r < cell_height - 1; ++cell_r )
		{
			for ( int cell_c = 0; cell_c < cell_width; ++cell_c )
			{
				walls.push_back( { { cell_r * 2 + 2, cell_c * 2 + 1 }, cellToIndex( cell_r, cell_c ), cellToIndex( cell_r + 1, cell_c ) } );
			}
		}
		for ( int cell_r = 0; cell_r < cell_height; ++cell_r )
		{
			for ( int cell_c = 0; cell_c < cell_width - 1; ++cell_c )
			{
				walls.push_back( { { cell_r * 2 + 1, cell_c * 2 + 2 }, cellToIndex( cell_r, cell_c ), cellToIndex( cell_r, cell_c + 1 ) } );
			}
		}

		std::shuffle( walls.begin(), walls.end(), prng );
		for ( const Wall& wall : walls )
		{
			// 若两侧单元尚未连通，则打通这堵墙。
			// 这样可以保证最终是一个连通且无环的“完美迷宫”骨架。
			if ( union_find.Connected( wall.cell1_idx, wall.cell2_idx ) )
				continue;
			union_find.Unite( wall.cell1_idx, wall.cell2_idx );
			grid[ wall.wall_pos.row ][ wall.wall_pos.col ] = 0;

			auto [ cell1_r, cell1_c ] = gridToCell( wall.wall_pos.row, wall.wall_pos.col );
			grid[ cell1_r * 2 + 1 ][ cell1_c * 2 + 1 ] = 0;

			auto [ cell2_r, cell2_c ] = gridToCell( wall.wall_pos.row + ( wall.wall_pos.row % 2 == 0 ? 1 : 0 ), wall.wall_pos.col + ( wall.wall_pos.col % 2 == 0 ? 1 : 0 ) );
			grid[ cell2_r * 2 + 1 ][ cell2_c * 2 + 1 ] = 0;
		}

		for ( int cell_r = 0; cell_r < cell_height; ++cell_r )
		{
			for ( int cell_c = 0; cell_c < cell_width; ++cell_c )
			{
				grid[ cell_r * 2 + 1 ][ cell_c * 2 + 1 ] = 0;
			}
		}

		constexpr int dx[ 4 ] = { 0, 1, 0, -1 };
		constexpr int dy[ 4 ] = { -1, 0, 1, 0 };
		auto inBounds = [ & ]( int row, int col ) {
			return row >= 0 && row < height && col >= 0 && col < width;
		};
		auto bfs = [ & ]( CellPosition start, CellPosition& farthest ) {
			// 用 BFS 找离某个起点最远的开放点，常用于近似迷宫直径的两个端点。
			std::vector<int>		 distances( height * width, -1 );
			std::queue<CellPosition> queue;
			queue.push( start );
			distances[ start.row * width + start.col ] = 0;
			farthest = start;
			int max_dist = 0;

			while ( !queue.empty() )
			{
				const CellPosition current = queue.front();
				queue.pop();
				const int current_index = current.row * width + current.col;
				if ( distances[ current_index ] > max_dist )
				{
					max_dist = distances[ current_index ];
					farthest = current;
				}

				for ( int dir = 0; dir < 4; ++dir )
				{
					const int next_row = current.row + dy[ dir ];
					const int next_col = current.col + dx[ dir ];
					if ( !inBounds( next_row, next_col ) )
						continue;
					const int next_index = next_row * width + next_col;
					if ( grid[ next_row ][ next_col ] != 0 || distances[ next_index ] != -1 )
						continue;
					distances[ next_index ] = distances[ current_index ] + 1;
					queue.push( { next_row, next_col } );
				}
			}

			return max_dist;
		};

		std::vector<CellPosition> open_cells;
		for ( int row = 1; row < height; row += 2 )
		{
			for ( int col = 1; col < width; col += 2 )
			{
				if ( grid[ row ][ col ] == 0 )
				{
					open_cells.push_back( { row, col } );
				}
			}
		}
		if ( open_cells.empty() )
		{
			throw std::runtime_error( "No open cell found" );
		}

		std::uniform_int_distribution<int> dist( 0, static_cast<int>( open_cells.size() ) - 1 );
		const CellPosition start_bfs = open_cells[ dist( prng ) ];
		CellPosition	   point_a;
		CellPosition	   point_b;
		bfs( start_bfs, point_a );
		bfs( point_a, point_b );

		out_start = point_a;
		out_goal = point_b;
		grid[ out_start.row ][ out_start.col ] = 0;
		grid[ out_goal.row ][ out_goal.col ] = 0;
		return grid;
	}

	void RenderGridWithPath( const Grid& grid, const std::vector<CellPosition>& path, CellPosition start_pos, CellPosition goal_pos, const RenderStyle& render_style )
	{
		// 纯应用层功能：把最终路径渲染到控制台。
		// 算法核心不依赖这里，因此后续可以轻松替换为 GUI、CSV 或 benchmark 输出。
		if ( !IsGridShapeValid( grid ) )
		{
			return;
		}
		const std::string& glyph_wall = render_style.use_ascii_glyphs ? render_style.wall_ascii : render_style.wall_unicode;
		const std::string& glyph_empty = render_style.use_ascii_glyphs ? render_style.empty_ascii : render_style.empty_unicode;
		const std::string& glyph_path = render_style.use_ascii_glyphs ? render_style.path_ascii : render_style.path_unicode;
		const std::string& glyph_start = render_style.use_ascii_glyphs ? render_style.start_ascii : render_style.start_unicode;
		const std::string& glyph_goal = render_style.use_ascii_glyphs ? render_style.goal_ascii : render_style.goal_unicode;
		const auto&		   glyph_arrows = render_style.use_ascii_glyphs ? render_style.arrows_ascii : render_style.arrows_unicode;

		std::unordered_set<long long> path_cell_keys;
		path_cell_keys.reserve( path.size() * 2 );
		auto MakeKey = []( int row, int col ) {
			return ( static_cast<long long>( row ) << 32 ) | static_cast<unsigned int>( col );
		};
		for ( const auto& pos : path )
		{
			path_cell_keys.insert( MakeKey( pos.row, pos.col ) );
		}

		std::unordered_map<long long, std::string> arrow_glyph_map;
		if ( render_style.print_arrows && path.size() > 1 )
		{
			// 如果开启箭头模式，则为路径中的每个后继点记录一个方向符号。
			for ( std::size_t idx = 1; idx < path.size(); ++idx )
			{
				const CellPosition& previous = path[ idx - 1 ];
				const CellPosition& current = path[ idx ];
				if ( current.row - previous.row == -1 )
					arrow_glyph_map[ MakeKey( current.row, current.col ) ] = glyph_arrows.at( 'U' );
				else if ( current.row - previous.row == 1 )
					arrow_glyph_map[ MakeKey( current.row, current.col ) ] = glyph_arrows.at( 'D' );
				else if ( current.col - previous.col == -1 )
					arrow_glyph_map[ MakeKey( current.row, current.col ) ] = glyph_arrows.at( 'L' );
				else if ( current.col - previous.col == 1 )
					arrow_glyph_map[ MakeKey( current.row, current.col ) ] = glyph_arrows.at( 'R' );
			}
		}

		for ( int row = 0; row < static_cast<int>( grid.size() ); ++row )
		{
			// 逐行构造输出，避免频繁小块写 stdout。
			std::ostringstream line_buffer;
			for ( int col = 0; col < static_cast<int>( grid[ 0 ].size() ); ++col )
			{
				if ( row == start_pos.row && col == start_pos.col )
					line_buffer << glyph_start;
				else if ( row == goal_pos.row && col == goal_pos.col )
					line_buffer << glyph_goal;
				else if ( grid[ row ][ col ] == 1 )
					line_buffer << glyph_wall;
				else if ( path_cell_keys.count( MakeKey( row, col ) ) )
					line_buffer << ( render_style.print_arrows && arrow_glyph_map.count( MakeKey( row, col ) ) ? arrow_glyph_map[ MakeKey( row, col ) ] : glyph_path );
				else
					line_buffer << glyph_empty;
			}
			std::cout << line_buffer.str() << "\n";
		}
	}

	void ParseArgs( int argc, char** argv, RunConfig& config )
	{
		// 命令行只负责填充 RunConfig，不直接驱动算法执行。
		// 这样 benchmark 程序也可以绕过 CLI，直接在代码里构造配置。
		ArgStream arg_stream;
		arg_stream.tokens.assign( argv + 1, argv + argc );

		auto ExpectValue = [ & ]( const std::string& flag ) -> std::string {
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
			}
			else if ( token == "--sx" )
			{
				if ( !config.cli_start_override )
					config.cli_start_override = CellPosition {};
				config.cli_start_override->row = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--sy" )
			{
				if ( !config.cli_start_override )
					config.cli_start_override = CellPosition {};
				config.cli_start_override->col = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--ex" )
			{
				if ( !config.cli_goal_override )
					config.cli_goal_override = CellPosition {};
				config.cli_goal_override->row = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--ey" )
			{
				if ( !config.cli_goal_override )
					config.cli_goal_override = CellPosition {};
				config.cli_goal_override->col = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--random" )
			{
				if ( arg_stream.cursor + 2 >= arg_stream.tokens.size() )
				{
					throw std::runtime_error( "--random W H P requires 3 values" );
				}
				config.use_random_map = true;
				config.random_map_width = std::stoi( arg_stream.Get() );
				config.random_map_height = std::stoi( arg_stream.Get() );
				config.random_wall_probability = std::stod( arg_stream.Get() );
			}
			else if ( token == "--seed" )
			{
				config.random_seed = static_cast<std::uint32_t>( std::stoul( ExpectValue( token ) ) );
			}
			else if ( token == "--wait" )
			{
				config.flush_wait_layers = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--no-print" )
			{
				config.print_path = false;
			}
			else if ( token == "--arrow" )
			{
				config.render_style.print_arrows = true;
			}
			else if ( token == "--ascii" )
			{
				config.render_style.use_ascii_glyphs = true;
			}
			else if ( token == "--no-ensure" )
			{
				config.reroll_until_solvable = false;
			}
			else if ( token == "--max-try" )
			{
				config.reroll_max_attempts = std::stoi( ExpectValue( token ) );
			}
			else if ( token == "--maze-demo" )
			{
				config.run_maze_demo = true;
			}
			else if ( token == "--zigzag" )
			{
				config.algorithm_options.enable_local_zigzag_expansion = true;
			}
			else if ( token == "--paper-strict" )
			{
				config.algorithm_options.enable_local_zigzag_expansion = false;
			}
			else if ( token == "--zigzag-threshold" )
			{
				// 该阈值只影响 IBP-B* 主算法中的“局部 zigzag 扩展”触发时机，
				// 不影响独立的 Zigzag maze rescue 状态搜索模型。
				config.algorithm_options.zigzag_threshold = std::stoi( ExpectValue( token ) );
			}
			else
			{
				throw std::runtime_error( "Unknown option: " + token );
			}
		}

		if ( config.use_random_map && config.map_file_path.empty() )
		{
			ValidateRandomGridArguments( config.random_map_width, config.random_map_height, config.random_wall_probability );
		}
		ValidateOverridePosition( config.cli_start_override, "Start" );
		ValidateOverridePosition( config.cli_goal_override, "Goal" );
	}

	int ShortestPathLength_BFS( const Grid& grid, CellPosition start, CellPosition goal )
	{
		// 标准四连通 BFS 最短路，仅用于参考对照，不参与 IBP-B* 核心逻辑。
		if ( !IsGridShapeValid( grid ) )
			return -1;
		if ( !IsCellPassable( grid, start.row, start.col ) || !IsCellPassable( grid, goal.row, goal.col ) )
			return -1;

		const int height = static_cast<int>( grid.size() );
		const int width = static_cast<int>( grid[ 0 ].size() );
		auto ToIndex = [ & ]( int row, int col ) {
			return row * width + col;
		};

		std::vector<int>		 distance( width * height, -1 );
		std::queue<CellPosition> queue;
		distance[ ToIndex( start.row, start.col ) ] = 0;
		queue.push( start );

		const int drow[ 4 ] = { -1, 1, 0, 0 };
		const int dcol[ 4 ] = { 0, 0, -1, 1 };
		while ( !queue.empty() )
		{
			const CellPosition current = queue.front();
			queue.pop();
			if ( current == goal )
			{
				return distance[ ToIndex( current.row, current.col ) ];
			}
			for ( int dir = 0; dir < 4; ++dir )
			{
				const int next_row = current.row + drow[ dir ];
				const int next_col = current.col + dcol[ dir ];
				if ( !IsCellPassable( grid, next_row, next_col ) )
					continue;
				const int next_index = ToIndex( next_row, next_col );
				if ( distance[ next_index ] != -1 )
					continue;
				distance[ next_index ] = distance[ ToIndex( current.row, current.col ) ] + 1;
				queue.push( { next_row, next_col } );
			}
		}
		return -1;
	}
}  // namespace IBP_BStarAlgorithm
