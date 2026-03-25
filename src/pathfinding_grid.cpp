#include "pathfinding_grid.hpp"

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

namespace pathfinding
{
	namespace
	{
		// 轻量随机数生成器，用于快速复现实验中的随机障碍图。
		struct LinearCongruentialRng32
		{
			std::uint32_t state = 0;

			// 用给定种子初始化轻量随机数生成器。
			explicit LinearCongruentialRng32( std::uint32_t seed ) : state( seed ) {}

			// 生成下一个 32 位无符号随机数。
			std::uint32_t NextUint32()
			{
				state = ( 1664525u * state + 1013904223u );
				return state;
			}

			// 生成 [0, 1) 范围内的随机浮点数。
			double NextFloatZeroToOne()
			{
				return static_cast<double>( NextUint32() ) / 4294967296.0;
			}
		};

		// 完美迷宫生成里的并查集实现，用于维护单元连通性。
		class UnionFind
		{
		public:
			// 构造并查集，并让每个元素初始时自成一个集合。
			explicit UnionFind( int count ) : parent_( count ), rank_( count, 0 )
			{
				for ( int index = 0; index < count; ++index )
				{
					parent_[ index ] = index;
				}
			}

			// 查找某个元素所在集合的代表元，并执行路径压缩。
			int Find( int value )
			{
				if ( parent_[ value ] != value )
				{
					parent_[ value ] = Find( parent_[ value ] );
				}
				return parent_[ value ];
			}

			// 判断两个元素当前是否属于同一个集合。
			bool Connected( int lhs, int rhs )
			{
				return Find( lhs ) == Find( rhs );
			}

			// 合并两个集合，采用按秩合并减少树高度。
			void Unite( int lhs, int rhs )
			{
				int lhs_root = Find( lhs );
				int rhs_root = Find( rhs );
				if ( lhs_root == rhs_root )
				{
					return;
				}
				if ( rank_[ lhs_root ] < rank_[ rhs_root ] )
				{
					parent_[ lhs_root ] = rhs_root;
				}
				else if ( rank_[ lhs_root ] > rank_[ rhs_root ] )
				{
					parent_[ rhs_root ] = lhs_root;
				}
				else
				{
					parent_[ rhs_root ] = lhs_root;
					rank_[ lhs_root ]++;
				}
			}

		private:
			std::vector<int> parent_;
			std::vector<int> rank_;
		};

		struct MazeWall
		{
			Position wall_position;
			int		 first_cell_index = -1;
			int		 second_cell_index = -1;
		};

		// 统一校验随机地图参数，避免尺寸或障碍率非法时把错误拖到生成过程里。
		void ValidateRandomMapArguments( int width, int height, double wall_probability )
		{
			if ( width <= 0 || height <= 0 )
			{
				throw std::runtime_error( "Random map width and height must be positive" );
			}
			if ( wall_probability < 0.0 || wall_probability > 1.0 )
			{
				throw std::runtime_error( "Random wall probability must be in [0, 1]" );
			}
		}

		// 把二维坐标打包成一个 64 位键，便于放进哈希容器里。
		long long MakeCellKey( int row, int col )
		{
			return ( static_cast<long long>( row ) << 32 ) | static_cast<unsigned int>( col );
		}
	}  // namespace

	// 判断网格是否是合法矩形。
	// 这是后续读图、寻路和渲染的基础前置条件。
	bool IsGridShapeValid( const Grid& grid )
	{
		if ( grid.empty() || grid.front().empty() )
		{
			return false;
		}

		const std::size_t expected_width = grid.front().size();
		for ( const auto& row : grid )
		{
			if ( row.size() != expected_width )
			{
				return false;
			}
		}
		return true;
	}

	// 判断一个坐标是否落在网格范围内。
	// 这里顺带要求输入网格本身必须先是合法矩形。
	bool IsInsideGrid( const Grid& grid, const Position& pos )
	{
		return IsGridShapeValid( grid ) && pos.row >= 0 && pos.row < static_cast<int>( grid.size() ) && pos.col >= 0 && pos.col < static_cast<int>( grid.front().size() );
	}

	// 判断指定格子是否可通行。
	// 本项目约定 0 为可通行，1 为障碍。
	bool IsPassable( const Grid& grid, int row, int col )
	{
		return row >= 0 && row < static_cast<int>( grid.size() ) && col >= 0 && col < static_cast<int>( grid.front().size() ) && grid[ row ][ col ] == 0;
	}

	// 校验一条路径是否真的是四连通合法路径。
	// 对比实验里，这个函数很重要，因为它能过滤掉“算法声称成功但路径断裂”的情况。
	bool ValidatePathContiguity( const Grid& grid, const std::vector<Position>& path )
	{
		if ( !IsGridShapeValid( grid ) || path.empty() )
		{
			return false;
		}

		for ( std::size_t index = 0; index < path.size(); ++index )
		{
			const Position& current = path[ index ];
			if ( !IsPassable( grid, current.row, current.col ) )
			{
				return false;
			}
			if ( index == 0 )
			{
				continue;
			}

			const Position& previous = path[ index - 1 ];
			const int manhattan_distance = std::abs( current.row - previous.row ) + std::abs( current.col - previous.col );
			if ( manhattan_distance != 1 )
			{
				return false;
			}
		}
		return true;
	}

	// 生成一张随机障碍图，并固定左上角为起点、右下角为终点。
	// 这里不保证地图一定可解，是否接受无解样本由上层实验驱动决定。
	MapInstance GenerateRandomMap( int width, int height, double wall_probability, std::uint32_t seed )
	{
		// 随机图只负责生成同一张测试地图，不负责“保证可解”。
		// 是否挑选可解样例，留给更上层的比较驱动去决定。
		ValidateRandomMapArguments( width, height, wall_probability );

		MapInstance map_instance;
		map_instance.grid.assign( height, std::vector<int>( width, 0 ) );
		map_instance.start = { 0, 0 };
		map_instance.goal = { height - 1, width - 1 };

		LinearCongruentialRng32 rng( seed );
		for ( int row = 0; row < height; ++row )
		{
			for ( int col = 0; col < width; ++col )
			{
				if ( rng.NextFloatZeroToOne() < wall_probability )
				{
					map_instance.grid[ row ][ col ] = 1;
				}
			}
		}

		map_instance.grid[ map_instance.start.row ][ map_instance.start.col ] = 0;
		map_instance.grid[ map_instance.goal.row ][ map_instance.goal.col ] = 0;
		return map_instance;
	}

	// 从文本文件读取地图。
	// 该函数负责识别 S/E、兼容 ASCII 与 Unicode 图元，并处理常见的 UTF-8 BOM 问题。
	MapInstance LoadMapFromFile( const std::string& file_path, const RenderStyle& render_style )
	{
		std::ifstream input_stream( file_path );
		if ( !input_stream )
		{
			throw std::runtime_error( "Failed to open map file: " + file_path );
		}

		const std::vector<std::string> wall_tokens = { render_style.wall_unicode, render_style.wall_ascii };
		const std::vector<std::string> empty_tokens = { render_style.empty_unicode, render_style.empty_ascii };
		const std::vector<std::string> start_tokens = { render_style.start_unicode, render_style.start_ascii, "S", "s" };
		const std::vector<std::string> goal_tokens = { render_style.goal_unicode, render_style.goal_ascii, "E", "e" };

		// 判断某个文本片段是否属于给定图元集合。
		auto IsOneOf = []( const std::string& token_candidate, const std::vector<std::string>& token_pool ) {
			for ( const auto& token : token_pool )
			{
				if ( token_candidate == token )
				{
					return true;
				}
			}
			return false;
		};

		std::size_t max_token_length = 1;
		// 更新当前支持图元中的最大字符长度，便于后续做最长匹配。
		auto UpdateMaxTokenLength = [ & ]( const std::vector<std::string>& token_pool ) {
			for ( const auto& token : token_pool )
			{
				max_token_length = std::max( max_token_length, token.size() );
			}
		};
		UpdateMaxTokenLength( wall_tokens );
		UpdateMaxTokenLength( empty_tokens );
		UpdateMaxTokenLength( start_tokens );
		UpdateMaxTokenLength( goal_tokens );

		MapInstance	 map_instance;
		bool		 start_seen = false;
		bool		 goal_seen = false;
		std::string	 raw_line;
		int			 current_row_index = 0;

		while ( std::getline( input_stream, raw_line ) )
		{
			// 兼容 UTF-8 BOM，避免首行第一个字符被误判成非法图元。
			if ( current_row_index == 0 && raw_line.size() >= 3
				 && static_cast<unsigned char>( raw_line[ 0 ] ) == 0xEF
				 && static_cast<unsigned char>( raw_line[ 1 ] ) == 0xBB
				 && static_cast<unsigned char>( raw_line[ 2 ] ) == 0xBF )
			{
				raw_line.erase( 0, 3 );
			}

			// 忽略空白字符，方便手工编排地图文件。
			raw_line.erase( std::remove_if( raw_line.begin(), raw_line.end(), []( unsigned char ch ) { return std::isspace( ch ); } ), raw_line.end() );
			if ( raw_line.empty() )
			{
				continue;
			}

			std::vector<int> row_values;
			for ( std::size_t offset = 0; offset < raw_line.size(); )
			{
				bool token_matched = false;
				std::size_t taken_length = 0;
				std::string token_candidate;
				const std::size_t try_limit = std::min<std::size_t>( max_token_length, raw_line.size() - offset );

				for ( std::size_t token_length = try_limit; token_length >= 1; --token_length )
				{
					token_candidate = raw_line.substr( offset, token_length );
					if ( IsOneOf( token_candidate, wall_tokens ) || IsOneOf( token_candidate, empty_tokens ) || IsOneOf( token_candidate, start_tokens )
						 || IsOneOf( token_candidate, goal_tokens ) )
					{
						taken_length = token_length;
						token_matched = true;
						break;
					}
					if ( token_length == 1 )
					{
						break;
					}
				}

				if ( !token_matched )
				{
					throw std::runtime_error( "Bad glyph at row " + std::to_string( current_row_index ) + ", pos " + std::to_string( offset ) );
				}

				if ( IsOneOf( token_candidate, wall_tokens ) )
				{
					row_values.push_back( 1 );
				}
				else if ( IsOneOf( token_candidate, empty_tokens ) )
				{
					row_values.push_back( 0 );
				}
				else if ( IsOneOf( token_candidate, start_tokens ) )
				{
					if ( start_seen )
					{
						throw std::runtime_error( "Map contains multiple start markers" );
					}
					map_instance.start = { current_row_index, static_cast<int>( row_values.size() ) };
					start_seen = true;
					row_values.push_back( 0 );
				}
				else
				{
					if ( goal_seen )
					{
						throw std::runtime_error( "Map contains multiple goal markers" );
					}
					map_instance.goal = { current_row_index, static_cast<int>( row_values.size() ) };
					goal_seen = true;
					row_values.push_back( 0 );
				}

				offset += taken_length;
			}

			map_instance.grid.push_back( std::move( row_values ) );
			++current_row_index;
		}

		if ( !IsGridShapeValid( map_instance.grid ) )
		{
			throw std::runtime_error( "Map rows must have the same width and the map must not be empty" );
		}
		if ( !start_seen || !goal_seen )
		{
			throw std::runtime_error( "Map needs S/E" );
		}
		return map_instance;
	}

	// 生成一张完美迷宫。
	// 先用 Kruskal 风格随机打墙保证“全连通且无环”，再用两次 BFS 近似找出一对距离较远的端点作为起终点。
	MapInstance GeneratePerfectMaze( int width, int height, std::uint32_t seed )
	{
		// 完美迷宫要求每个开放单元最终都连通且无环。
		// 这里使用 Kruskal 风格打墙，再用两次 BFS 近似选出一对相距较远的端点。
		if ( width < 3 || height < 3 )
		{
			throw std::runtime_error( "Maze size too small" );
		}

		std::mt19937 prng( seed );
		const int cell_width = ( width - 1 ) / 2;
		const int cell_height = ( height - 1 ) / 2;
		const int total_cells = cell_width * cell_height;
		if ( total_cells <= 0 )
		{
			throw std::runtime_error( "Maze size too small" );
		}

		MapInstance map_instance;
		map_instance.grid.assign( height, std::vector<int>( width, 1 ) );
		if ( total_cells == 1 )
		{
			map_instance.grid[ 1 ][ 1 ] = 0;
			map_instance.start = { 1, 1 };
			map_instance.goal = { 1, 1 };
			return map_instance;
		}

		UnionFind union_find( total_cells );
		std::vector<MazeWall> walls;

		// 把迷宫单元坐标映射到并查集索引。
		auto CellToIndex = [ cell_width ]( int cell_row, int cell_col ) {
			return cell_row * cell_width + cell_col;
		};
		// 把网格上的“单元中心坐标”映射回迷宫单元坐标。
		auto GridToCell = []( int grid_row, int grid_col ) -> std::pair<int, int> {
			return { ( grid_row - 1 ) / 2, ( grid_col - 1 ) / 2 };
		};

		for ( int cell_row = 0; cell_row < cell_height - 1; ++cell_row )
		{
			for ( int cell_col = 0; cell_col < cell_width; ++cell_col )
			{
				walls.push_back( { { cell_row * 2 + 2, cell_col * 2 + 1 }, CellToIndex( cell_row, cell_col ), CellToIndex( cell_row + 1, cell_col ) } );
			}
		}
		for ( int cell_row = 0; cell_row < cell_height; ++cell_row )
		{
			for ( int cell_col = 0; cell_col < cell_width - 1; ++cell_col )
			{
				walls.push_back( { { cell_row * 2 + 1, cell_col * 2 + 2 }, CellToIndex( cell_row, cell_col ), CellToIndex( cell_row, cell_col + 1 ) } );
			}
		}

		std::shuffle( walls.begin(), walls.end(), prng );
		for ( const MazeWall& wall : walls )
		{
			if ( union_find.Connected( wall.first_cell_index, wall.second_cell_index ) )
			{
				continue;
			}

			union_find.Unite( wall.first_cell_index, wall.second_cell_index );
			map_instance.grid[ wall.wall_position.row ][ wall.wall_position.col ] = 0;

			auto [ first_cell_row, first_cell_col ] = GridToCell( wall.wall_position.row, wall.wall_position.col );
			map_instance.grid[ first_cell_row * 2 + 1 ][ first_cell_col * 2 + 1 ] = 0;

			auto [ second_cell_row, second_cell_col ] =
				GridToCell( wall.wall_position.row + ( wall.wall_position.row % 2 == 0 ? 1 : 0 ), wall.wall_position.col + ( wall.wall_position.col % 2 == 0 ? 1 : 0 ) );
			map_instance.grid[ second_cell_row * 2 + 1 ][ second_cell_col * 2 + 1 ] = 0;
		}

		for ( int cell_row = 0; cell_row < cell_height; ++cell_row )
		{
			for ( int cell_col = 0; cell_col < cell_width; ++cell_col )
			{
				map_instance.grid[ cell_row * 2 + 1 ][ cell_col * 2 + 1 ] = 0;
			}
		}

		constexpr int row_offsets[ 4 ] = { -1, 1, 0, 0 };
		constexpr int col_offsets[ 4 ] = { 0, 0, -1, 1 };
		// 判断某个坐标是否在迷宫网格范围内。
		auto InBounds = [ & ]( int row, int col ) {
			return row >= 0 && row < height && col >= 0 && col < width;
		};
		// 从某个开放格子出发，找距离它最远的另一个开放格子。
		// 这用于近似迷宫直径，从而挑出更有代表性的起终点。
		auto FindFarthestOpenCell = [ & ]( Position start, Position& farthest ) {
			std::vector<int> distance( height * width, -1 );
			std::queue<Position> queue;
			queue.push( start );
			distance[ start.row * width + start.col ] = 0;
			farthest = start;
			int max_distance = 0;

			while ( !queue.empty() )
			{
				const Position current = queue.front();
				queue.pop();
				const int current_index = current.row * width + current.col;
				if ( distance[ current_index ] > max_distance )
				{
					max_distance = distance[ current_index ];
					farthest = current;
				}

				for ( int direction_index = 0; direction_index < 4; ++direction_index )
				{
					const int next_row = current.row + row_offsets[ direction_index ];
					const int next_col = current.col + col_offsets[ direction_index ];
					if ( !InBounds( next_row, next_col ) )
					{
						continue;
					}
					const int next_index = next_row * width + next_col;
					if ( map_instance.grid[ next_row ][ next_col ] != 0 || distance[ next_index ] != -1 )
					{
						continue;
					}
					distance[ next_index ] = distance[ current_index ] + 1;
					queue.push( { next_row, next_col } );
				}
			}
		};

		std::vector<Position> open_cells;
		for ( int row = 1; row < height; row += 2 )
		{
			for ( int col = 1; col < width; col += 2 )
			{
				if ( map_instance.grid[ row ][ col ] == 0 )
				{
					open_cells.push_back( { row, col } );
				}
			}
		}
		if ( open_cells.empty() )
		{
			throw std::runtime_error( "No open cell found in maze" );
		}

		std::uniform_int_distribution<int> distribution( 0, static_cast<int>( open_cells.size() ) - 1 );
		const Position random_open_cell = open_cells[ distribution( prng ) ];
		Position first_endpoint;
		Position second_endpoint;
		FindFarthestOpenCell( random_open_cell, first_endpoint );
		FindFarthestOpenCell( first_endpoint, second_endpoint );

		map_instance.start = first_endpoint;
		map_instance.goal = second_endpoint;
		map_instance.grid[ map_instance.start.row ][ map_instance.start.col ] = 0;
		map_instance.grid[ map_instance.goal.row ][ map_instance.goal.col ] = 0;
		return map_instance;
	}

	// 把地图和路径渲染到终端。
	// 这个函数完全属于展示层，算法核心不依赖它，因此后续很容易换成文件输出或图形界面。
	void RenderGridWithPath( const Grid& grid, const std::vector<Position>& path, Position start, Position goal, const RenderStyle& render_style )
	{
		if ( !IsGridShapeValid( grid ) )
		{
			return;
		}

		const std::string& wall_glyph = render_style.use_ascii_glyphs ? render_style.wall_ascii : render_style.wall_unicode;
		const std::string& empty_glyph = render_style.use_ascii_glyphs ? render_style.empty_ascii : render_style.empty_unicode;
		const std::string& path_glyph = render_style.use_ascii_glyphs ? render_style.path_ascii : render_style.path_unicode;
		const std::string& start_glyph = render_style.use_ascii_glyphs ? render_style.start_ascii : render_style.start_unicode;
		const std::string& goal_glyph = render_style.use_ascii_glyphs ? render_style.goal_ascii : render_style.goal_unicode;
		const auto& arrow_glyphs = render_style.use_ascii_glyphs ? render_style.arrows_ascii : render_style.arrows_unicode;

		std::unordered_set<long long> path_cells;
		path_cells.reserve( path.size() * 2 );
		for ( const Position& position : path )
		{
			path_cells.insert( MakeCellKey( position.row, position.col ) );
		}

		std::unordered_map<long long, std::string> arrows_by_cell;
		if ( render_style.print_arrows && path.size() > 1 )
		{
			// 箭头模式下，用后继点记录移动方向，便于观察路径走势。
			for ( std::size_t index = 1; index < path.size(); ++index )
			{
				const Position& previous = path[ index - 1 ];
				const Position& current = path[ index ];
				if ( current.row - previous.row == -1 )
				{
					arrows_by_cell[ MakeCellKey( current.row, current.col ) ] = arrow_glyphs.at( 'U' );
				}
				else if ( current.row - previous.row == 1 )
				{
					arrows_by_cell[ MakeCellKey( current.row, current.col ) ] = arrow_glyphs.at( 'D' );
				}
				else if ( current.col - previous.col == -1 )
				{
					arrows_by_cell[ MakeCellKey( current.row, current.col ) ] = arrow_glyphs.at( 'L' );
				}
				else if ( current.col - previous.col == 1 )
				{
					arrows_by_cell[ MakeCellKey( current.row, current.col ) ] = arrow_glyphs.at( 'R' );
				}
			}
		}

		for ( int row = 0; row < static_cast<int>( grid.size() ); ++row )
		{
			std::ostringstream line_buffer;
			for ( int col = 0; col < static_cast<int>( grid.front().size() ); ++col )
			{
				const long long cell_key = MakeCellKey( row, col );
				if ( row == start.row && col == start.col )
				{
					line_buffer << start_glyph;
				}
				else if ( row == goal.row && col == goal.col )
				{
					line_buffer << goal_glyph;
				}
				else if ( grid[ row ][ col ] == 1 )
				{
					line_buffer << wall_glyph;
				}
				else if ( path_cells.count( cell_key ) != 0 )
				{
					line_buffer << ( render_style.print_arrows && arrows_by_cell.count( cell_key ) != 0 ? arrows_by_cell[ cell_key ] : path_glyph );
				}
				else
				{
					line_buffer << empty_glyph;
				}
			}
			std::cout << line_buffer.str() << "\n";
		}
	}
}  // namespace pathfinding
