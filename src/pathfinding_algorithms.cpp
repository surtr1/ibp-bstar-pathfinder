#include "pathfinding_algorithms.hpp"

#include "ibp_bstar_core.hpp"
#include "pathfinding_grid.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

namespace pathfinding
{
	namespace
	{
		// 统一使用高精度时钟进行简单 benchmark 计时。
		using TimerClock = std::chrono::high_resolution_clock;

		enum class BranchMode : std::uint8_t
		{
			Direct = 0,
			FollowObstacle = 1
		};

		struct BranchStateNode
		{
			int			 cell_linear_index = -1;
			std::uint8_t mode_index = 0;
			std::uint8_t direction_index = 4;
		};

		struct ClassicBranchNode
		{
			int		 cell_linear_index = -1;
			int		 parent_node_index = -1;
			BranchMode mode = BranchMode::Direct;
			char	 direction = 0;
			int		 collision_count = 0;
		};

		struct PaperBranchNode
		{
			int		 cell_linear_index = -1;
			int		 parent_node_index = -1;
			BranchMode mode = BranchMode::Direct;
			char	 travel_direction = 0;
			char	 original_collision_direction = 0;
			int		 collision_count = 0;
		};

		constexpr std::array<Position, 4> kFourNeighborOffsets = { Position { -1, 0 }, Position { 1, 0 }, Position { 0, -1 }, Position { 0, 1 } };

		// 把二维坐标映射到一维数组下标，方便连续存储访问状态。
		int ToLinearIndex( int row, int col, int grid_width )
		{
			return row * grid_width + col;
		}

		// 把一维数组下标恢复成网格坐标。
		Position FromLinearIndex( int linear_index, int grid_width )
		{
			return { linear_index / grid_width, linear_index % grid_width };
		}

		// 计算两个格子的 Manhattan 距离。
		// 在四连通栅格上，这是 A* 常用的启发式。
		int ManhattanDistance( Position lhs, Position rhs )
		{
			return std::abs( lhs.row - rhs.row ) + std::abs( lhs.col - rhs.col );
		}

		// 统一检查一次搜索请求是否合法。
		// 这样各个算法入口都能共享同一套输入校验逻辑。
		bool IsSearchRequestValid( const Grid& grid, Position start, Position goal )
		{
			return IsGridShapeValid( grid ) && IsInsideGrid( grid, start ) && IsInsideGrid( grid, goal ) && IsPassable( grid, start.row, start.col ) && IsPassable( grid, goal.row, goal.col );
		}

		// 根据 parent 数组从终点反向重建格子路径。
		// BFS、A*、Dijkstra 这类基于 parent 链的算法都会复用它。
		std::vector<Position> ReconstructCellPath( int goal_linear_index, const std::vector<int>& parent_by_index, int grid_width )
		{
			std::vector<Position> path;
			for ( int trace_index = goal_linear_index; trace_index != -1; trace_index = parent_by_index[ trace_index ] )
			{
				path.push_back( FromLinearIndex( trace_index, grid_width ) );
			}
			std::reverse( path.begin(), path.end() );
			return path;
		}

		// 把“重建路径 + 写回结果统计 + 做路径合法性校验”这几个收尾动作集中到一起。
		// 这样可以避免不同算法各自重复一遍同样的收尾代码。
		void FinalizeSuccessfulCellResult( SearchResult& result, const Grid& grid, const std::vector<int>& parent_by_index, int goal_linear_index, int grid_width )
		{
			result.path = ReconstructCellPath( goal_linear_index, parent_by_index, grid_width );
			result.meet_position = result.path.empty() ? Position { -1, -1 } : result.path.back();
			result.statistics.final_path_length = static_cast<int>( result.path.size() );
			result.success = ValidatePathContiguity( grid, result.path );
		}

		// 把命令行里的算法名 token 归一化，便于兼容大小写和空白差异。
		std::string NormalizeToken( std::string token )
		{
			token.erase( std::remove_if( token.begin(), token.end(), []( unsigned char ch ) { return std::isspace( ch ); } ), token.end() );
			std::transform( token.begin(), token.end(), token.begin(), []( unsigned char ch ) { return static_cast<char>( std::tolower( ch ) ); } );
			return token;
		}

		// 统一测量算法耗时，并保证返回值至少为 1 微秒。
		// 这样在 Windows 上也不会因为计时粒度问题把很快的算法显示成 0。
		long long MeasureElapsedMicroseconds( TimerClock::time_point started_at )
		{
			const long long elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>( TimerClock::now() - started_at ).count();
			if ( elapsed_nanoseconds <= 0 )
			{
				return 1;
			}
			return std::max<long long>( 1, ( elapsed_nanoseconds + 999 ) / 1000 );
		}

		// 把方向字符压成小整数，便于状态编码。
		int DirectionToIndex( char direction_char )
		{
			switch ( direction_char )
			{
			case 'U': return 0;
			case 'D': return 1;
			case 'L': return 2;
			case 'R': return 3;
			default: return 4;
			}
		}

		// 把方向下标恢复成方向字符。
		char IndexToDirection( int direction_index )
		{
			switch ( direction_index )
			{
			case 0: return 'U';
			case 1: return 'D';
			case 2: return 'L';
			case 3: return 'R';
			default: return 0;
			}
		}

		// 根据方向字符返回对应的行列位移。
		std::pair<int, int> DirectionDelta( char direction_char )
		{
			switch ( direction_char )
			{
			case 'U': return { -1, 0 };
			case 'D': return { 1, 0 };
			case 'L': return { 0, -1 };
			case 'R': return { 0, 1 };
			default: return { 0, 0 };
			}
		}

		// 返回某个方向的反方向。
		char OppositeDirection( char direction_char )
		{
			switch ( direction_char )
			{
			case 'U': return 'D';
			case 'D': return 'U';
			case 'L': return 'R';
			case 'R': return 'L';
			default: return 0;
			}
		}

		// 返回某个方向对应的“左转 / 右转”方向。
		std::pair<char, char> LeftRightDirections( char direction_char )
		{
			switch ( direction_char )
			{
			case 'U': return { 'L', 'R' };
			case 'D': return { 'R', 'L' };
			case 'L': return { 'D', 'U' };
			case 'R': return { 'U', 'D' };
			default: return { 0, 0 };
			}
		}

		// 根据当前位置和目标位置给出一个“更偏向目标”的贪心方向。
		// Branch Star 直行阶段会依赖这个方向选择。
		char ChooseGreedyDirection( int current_row, int current_col, int goal_row, int goal_col )
		{
			const int delta_row = goal_row - current_row;
			const int delta_col = goal_col - current_col;
			if ( delta_row == 0 && delta_col == 0 )
			{
				return 0;
			}
			if ( std::abs( delta_row ) >= std::abs( delta_col ) )
			{
				return delta_row > 0 ? 'D' : 'U';
			}
			return delta_col > 0 ? 'R' : 'L';
		}

		// 把 Branch Star 的状态编码成一维整数。
		// 这里状态只保留：位置、模式、当前绕障方向，尽量贴近 Python 版的轻量实现。
		int EncodeBranchStateId( int cell_linear_index, std::uint8_t mode_index, std::uint8_t direction_index )
		{
			constexpr int kModeCount = 2;
			constexpr int kDirectionCount = 5;
			return ( cell_linear_index * kModeCount + static_cast<int>( mode_index ) ) * kDirectionCount + static_cast<int>( direction_index );
		}

		// 与 EncodeBranchStateId 配套，用于把压缩状态恢复成可读结构。
		BranchStateNode DecodeBranchStateId( int state_id )
		{
			constexpr int kDirectionCount = 5;
			constexpr int kModeCount = 2;

			BranchStateNode node;
			node.direction_index = static_cast<std::uint8_t>( state_id % kDirectionCount );
			state_id /= kDirectionCount;
			node.mode_index = static_cast<std::uint8_t>( state_id % kModeCount );
			state_id /= kModeCount;
			node.cell_linear_index = state_id;
			return node;
		}

		// 当直行被障碍挡住时，生成左右两个绕障分支。
		// 如果允许回退，则把反方向作为一个更弱的候选。
		std::array<char, 3> BuildInitialBranchDirections( char blocked_direction, bool allow_reverse_when_crawling )
		{
			std::array<char, 3> ordered_directions { 0, 0, 0 };
			const auto [ left_direction, right_direction ] = LeftRightDirections( blocked_direction );
			ordered_directions[ 0 ] = left_direction;
			ordered_directions[ 1 ] = right_direction;
			ordered_directions[ 2 ] = allow_reverse_when_crawling ? OppositeDirection( blocked_direction ) : 0;
			return ordered_directions;
		}

		// 在绕障过程中，如果当前绕障方向走不通，则尝试左右转向。
		// 这个规则比此前的高维状态模型简单很多，更接近 Python 默认版 B* 的“轻量分支绕障”。
		std::array<char, 3> BuildFollowDirections( char current_direction, bool allow_reverse_when_crawling )
		{
			std::array<char, 3> ordered_directions { 0, 0, 0 };
			const auto [ left_direction, right_direction ] = LeftRightDirections( current_direction );
			ordered_directions[ 0 ] = left_direction;
			ordered_directions[ 1 ] = right_direction;
			ordered_directions[ 2 ] = allow_reverse_when_crawling ? OppositeDirection( current_direction ) : 0;
			return ordered_directions;
		}

		// 返回四个正交方向。
		// 当局部分支完全走死时，可以把它们作为兜底扩展方向。
		std::array<char, 4> BuildAllDirections()
		{
			return { 'U', 'D', 'L', 'R' };
		}

		// 生成一个仅包含算法名的空结果对象。
		// 当输入非法或算法提前失败时，可以直接返回这个基础结果。
		SearchResult MakeEmptyResult( AlgorithmId algorithm_id )
		{
			SearchResult result;
			result.algorithm_name = GetAlgorithmName( algorithm_id );
			return result;
		}

		template <typename HeuristicFn>
		SearchResult RunBestFirstSearch( AlgorithmId algorithm_id, const Grid& grid, Position start, Position goal, HeuristicFn heuristic )
		{
			// A* 和 Dijkstra 都可以抽象成“基于优先队列的最佳优先图搜索”，
			// 区别只在于启发式项是否为 0。
			SearchResult result = MakeEmptyResult( algorithm_id );
			const auto	started_at = TimerClock::now();
			if ( !IsSearchRequestValid( grid, start, goal ) )
			{
				result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
				return result;
			}

			const int grid_height = static_cast<int>( grid.size() );
			const int grid_width = static_cast<int>( grid.front().size() );
			const int total_cells = grid_height * grid_width;
			const int start_linear_index = ToLinearIndex( start.row, start.col, grid_width );
			const int goal_linear_index = ToLinearIndex( goal.row, goal.col, grid_width );

			struct OpenNode
			{
				int linear_index = -1;
				int g_cost = 0;
				int f_cost = 0;
				int h_cost = 0;
			};

			struct OpenNodeGreater
			{
				// 定义优先队列的小根堆顺序：
				// f 值更小的状态优先，其次再比较 h 和 g。
				bool operator()( const OpenNode& lhs, const OpenNode& rhs ) const
				{
					if ( lhs.f_cost != rhs.f_cost )
					{
						return lhs.f_cost > rhs.f_cost;
					}
					if ( lhs.h_cost != rhs.h_cost )
					{
						return lhs.h_cost > rhs.h_cost;
					}
					return lhs.g_cost > rhs.g_cost;
				}
			};

			constexpr int kInfinityDistance = std::numeric_limits<int>::max();
			std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeGreater> open_queue;
			std::vector<int> best_distance( total_cells, kInfinityDistance );
			std::vector<int> parent_by_index( total_cells, -1 );
			std::vector<bool> closed( total_cells, false );

			const int start_heuristic = heuristic( start );
			best_distance[ start_linear_index ] = 0;
			// 起点的 g=0，f 仅由启发式决定。
			// A* 会把 h(start) 放进 f，Dijkstra 则因为 h 恒为 0 而退化成纯 g 排序。
			open_queue.push( { start_linear_index, 0, start_heuristic, start_heuristic } );

			bool found_goal = false;
			while ( !open_queue.empty() )
			{
				const OpenNode current = open_queue.top();
				open_queue.pop();
				// 优先队列里可能存在陈旧条目：
				// 某个格子先前以较差代价入队，后来又以更优代价重新入队。
				// 这里一旦发现它已经 closed，就直接跳过。
				if ( closed[ current.linear_index ] )
				{
					continue;
				}

				closed[ current.linear_index ] = true;
				result.statistics.expanded_node_count++;
				if ( current.linear_index == goal_linear_index )
				{
					found_goal = true;
					break;
				}

				const Position current_position = FromLinearIndex( current.linear_index, grid_width );
				for ( const Position& offset : kFourNeighborOffsets )
				{
					const int next_row = current_position.row + offset.row;
					const int next_col = current_position.col + offset.col;
					if ( !IsPassable( grid, next_row, next_col ) )
					{
						continue;
					}

					const int next_linear_index = ToLinearIndex( next_row, next_col, grid_width );
					if ( closed[ next_linear_index ] )
					{
						continue;
					}

					const int candidate_distance = best_distance[ current.linear_index ] + 1;
					if ( candidate_distance >= best_distance[ next_linear_index ] )
					{
						// 只有在找到更短的 g 值时，才更新 parent 并重新入队。
						continue;
					}

					parent_by_index[ next_linear_index ] = current.linear_index;
					best_distance[ next_linear_index ] = candidate_distance;
					const Position next_position { next_row, next_col };
					const int heuristic_value = heuristic( next_position );
					open_queue.push( { next_linear_index, candidate_distance, candidate_distance + heuristic_value, heuristic_value } );
				}
			}

			if ( found_goal )
			{
				FinalizeSuccessfulCellResult( result, grid, parent_by_index, goal_linear_index, grid_width );
			}
			result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
			return result;
		}
	}  // namespace

	// 把算法枚举转成用户可读的名字。
	// 对比摘要表和路径打印都会使用这里的结果。
	std::string GetAlgorithmName( AlgorithmId algorithm_id )
	{
		switch ( algorithm_id )
		{
		case AlgorithmId::Bfs: return "BFS";
		case AlgorithmId::AStar: return "A*";
		case AlgorithmId::Dijkstra: return "Dijkstra";
		case AlgorithmId::BranchStar: return "B* PaperCrawl";
		case AlgorithmId::BranchStarClassic: return "B* GreedyLite";
		case AlgorithmId::BranchStarLegacy: return "B* Robust";
		case AlgorithmId::IbpBStar: return "IBP-B*";
		default: return "Unknown";
		}
	}

	// 返回默认参与对比的一组算法。
	// 当前包含最短路基线、启发式基线、经典 Branch Star 和你的 IBP-B*。
	std::vector<AlgorithmId> GetDefaultAlgorithms()
	{
		return { AlgorithmId::Bfs, AlgorithmId::AStar, AlgorithmId::Dijkstra, AlgorithmId::BranchStar, AlgorithmId::IbpBStar };
	}

	// 解析命令行中的算法列表。
	// 这里会兼容一些常见别名，比如 branch-star、b*、ibp_bstar 等。
	std::vector<AlgorithmId> ParseAlgorithmList( const std::string& csv_list )
	{
		std::vector<AlgorithmId> parsed_algorithms;
		std::size_t begin_index = 0;
		while ( begin_index <= csv_list.size() )
		{
			const std::size_t comma_index = csv_list.find( ',', begin_index );
			const std::string raw_token = csv_list.substr( begin_index, comma_index == std::string::npos ? std::string::npos : comma_index - begin_index );
			const std::string token = NormalizeToken( raw_token );

			if ( !token.empty() )
			{
				if ( token == "bfs" )
				{
					parsed_algorithms.push_back( AlgorithmId::Bfs );
				}
				else if ( token == "astar" || token == "a*" )
				{
					parsed_algorithms.push_back( AlgorithmId::AStar );
				}
				else if ( token == "dijkstra" )
				{
					parsed_algorithms.push_back( AlgorithmId::Dijkstra );
				}
				else if ( token == "branchstar" || token == "branch-star" || token == "bstar" || token == "b*" || token == "bstarpaper"
						  || token == "bstar_paper" || token == "bstarpapercrawl" || token == "bstar_papercrawl" || token == "bstar-paper"
						  || token == "bstar-paper-crawl" || token == "bstar_paper_crawl" )
				{
					parsed_algorithms.push_back( AlgorithmId::BranchStar );
				}
				else if ( token == "branchstarclassic" || token == "branch-star-classic" || token == "branchstar_classic" || token == "branchstareasy"
						  || token == "branch-star-easy" || token == "branchstar_easy" || token == "bstarlite" || token == "bstar_lite"
						  || token == "bstargreedylite" || token == "bstar_greedylite" || token == "bstar-greedy-lite" || token == "bstar_greedy_lite" )
				{
					parsed_algorithms.push_back( AlgorithmId::BranchStarClassic );
				}
				else if ( token == "branchstarlegacy" || token == "branch-star-legacy" || token == "branchstar_legacy" || token == "branchstarfull"
						  || token == "branch-star-full" || token == "branchstar_full" || token == "bstarrobust" || token == "bstar_robust"
						  || token == "bstarfallback" || token == "bstar_fallback" || token == "bstar-robust" || token == "bstar-robust-fallback"
						  || token == "bstar_robust_fallback" )
				{
					parsed_algorithms.push_back( AlgorithmId::BranchStarLegacy );
				}
				else if ( token == "ibpbstar" || token == "ibp-bstar" || token == "ibp_bstar" || token == "ibp-b*" || token == "ibp" )
				{
					parsed_algorithms.push_back( AlgorithmId::IbpBStar );
				}
				else
				{
					throw std::runtime_error( "Unknown algorithm name: " + raw_token );
				}
			}

			if ( comma_index == std::string::npos )
			{
				break;
			}
			begin_index = comma_index + 1;
		}

		if ( parsed_algorithms.empty() )
		{
			throw std::runtime_error( "Algorithm list must not be empty" );
		}
		return parsed_algorithms;
	}

	// 运行标准四连通 BFS。
	// 它既是一个独立的比较算法，也是其它启发式算法的最短路参考基线。
		SearchResult RunBfs( const Grid& grid, Position start, Position goal )
		{
			SearchResult result = MakeEmptyResult( AlgorithmId::Bfs );
			const auto	started_at = TimerClock::now();
		if ( !IsSearchRequestValid( grid, start, goal ) )
		{
			result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
			return result;
		}

		const int grid_height = static_cast<int>( grid.size() );
		const int grid_width = static_cast<int>( grid.front().size() );
		const int total_cells = grid_height * grid_width;
		const int start_linear_index = ToLinearIndex( start.row, start.col, grid_width );
		const int goal_linear_index = ToLinearIndex( goal.row, goal.col, grid_width );

		std::queue<int> open_queue;
		std::vector<int> distance_by_index( total_cells, -1 );
		std::vector<int> parent_by_index( total_cells, -1 );

			distance_by_index[ start_linear_index ] = 0;
			open_queue.push( start_linear_index );

			bool found_goal = false;
			while ( !open_queue.empty() )
			{
				// BFS 始终按“先进先出”处理，天然保证第一次到达某点就是最短层数。
				const int current_linear_index = open_queue.front();
				open_queue.pop();
				result.statistics.expanded_node_count++;
				if ( current_linear_index == goal_linear_index )
				{
				found_goal = true;
				break;
			}

				const Position current_position = FromLinearIndex( current_linear_index, grid_width );
				for ( const Position& offset : kFourNeighborOffsets )
				{
					const int next_row = current_position.row + offset.row;
					const int next_col = current_position.col + offset.col;
				if ( !IsPassable( grid, next_row, next_col ) )
				{
					continue;
				}

					const int next_linear_index = ToLinearIndex( next_row, next_col, grid_width );
					if ( distance_by_index[ next_linear_index ] != -1 )
					{
						// distance != -1 说明这个格子已经在更早或同层被访问过，不再重复入队。
						continue;
					}

					distance_by_index[ next_linear_index ] = distance_by_index[ current_linear_index ] + 1;
					parent_by_index[ next_linear_index ] = current_linear_index;
				open_queue.push( next_linear_index );
			}
		}

		if ( found_goal )
		{
			FinalizeSuccessfulCellResult( result, grid, parent_by_index, goal_linear_index, grid_width );
		}
		result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
		return result;
	}

	// 运行 A* 搜索。
	// 当前启发式采用 Manhattan 距离，适用于四连通栅格。
	SearchResult RunAStar( const Grid& grid, Position start, Position goal )
	{
		return RunBestFirstSearch( AlgorithmId::AStar, grid, start, goal, [ goal ]( Position current ) {
			return ManhattanDistance( current, goal );
		} );
	}

	// 运行 Dijkstra 搜索。
	// 这里本质上是把 A* 的启发式项固定成 0。
	SearchResult RunDijkstra( const Grid& grid, Position start, Position goal )
	{
		return RunBestFirstSearch( AlgorithmId::Dijkstra, grid, start, goal, []( Position ) {
			return 0;
		} );
	}

		SearchResult RunBranchStar( const Grid& grid, Position start, Position goal, const BranchStarOptions& options )
		{
		// 论文语义版 Branch Star：
		// 1. 自由探索点优先沿主方向向目标推进
		// 2. 首次撞障后，按“除当前前进方向外的其它方向”生成绕障分支
		// 3. 绕障分支优先尝试回到最初被挡住的方向
		// 4. 若回归失败，则继续沿当前绕障方向推进
		// 5. 若当前绕障方向也失败，则尝试原始碰撞方向的反方向
		//
		// 这里不是直接照搬旧的轻量 FIFO 版本，而是引入“碰撞次数优先”的 0-1 BFS 风格前沿：
		// - 自由推进 / 继续绕障视为 0 代价
		// - 首次因障碍切换到绕障视为 1 代价
		//
		// 这样既保留了 Zhao Qingsong(2010) 被后续 IBP-B* 论文引用时描述的自由探索 / crawling 语义，
		// 也比单纯 FIFO 更稳，避免轻量版在随机图上过早把可行分支剪死。
		SearchResult result = MakeEmptyResult( AlgorithmId::BranchStar );
		const auto	started_at = TimerClock::now();
		if ( !IsSearchRequestValid( grid, start, goal ) )
		{
			result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
			return result;
		}

		const int grid_height = static_cast<int>( grid.size() );
		const int grid_width = static_cast<int>( grid.front().size() );
		const int total_cells = grid_height * grid_width;
		constexpr int kInfinityCollision = std::numeric_limits<int>::max();
		const int start_linear_index = ToLinearIndex( start.row, start.col, grid_width );
		const int goal_linear_index = ToLinearIndex( goal.row, goal.col, grid_width );

		std::deque<int> frontier;
		std::vector<PaperBranchNode> nodes;
		nodes.reserve( total_cells * 6 );
		std::vector<int> best_direct_collision( total_cells, kInfinityCollision );
		std::vector<int> best_crawling_collision( total_cells * 16, kInfinityCollision );
		std::vector<int> discovered_node_indices;
		discovered_node_indices.reserve( total_cells * 2 );

		auto EncodeCrawlingStateIndex = [ & ]( int cell_linear_index, char travel_direction, char original_collision_direction ) {
			// crawling 状态不能只按“所在格子”去重：
			// 同一个格子上，若当前贴边方向不同，或最初被挡住的方向不同，
			// 后续能否成功回归目标方向的行为也不同，因此要拆成更细的状态。
			const int travel_direction_index = DirectionToIndex( travel_direction );
			const int original_direction_index = DirectionToIndex( original_collision_direction );
			if ( travel_direction_index < 0 || travel_direction_index >= 4 || original_direction_index < 0 || original_direction_index >= 4 )
			{
				return -1;
			}
			return cell_linear_index * 16 + travel_direction_index * 4 + original_direction_index;
		};

		auto PushNode = [ & ]( int next_row,
							   int next_col,
							   BranchMode next_mode,
							   char next_travel_direction,
							   char original_collision_direction,
							   int parent_node_index,
							   int collision_count,
							   bool high_priority ) {
			// PushNode 是论文语义版 B* 的统一入队入口：
			// 1. 检查目标格是否可走
			// 2. 检查这个状态是否比历史记录更优
			// 3. 写入 nodes / best_collision
			// 4. 按优先级放到 deque 头或尾
			if ( !IsPassable( grid, next_row, next_col ) )
			{
				return false;
			}

			const int next_linear_index = ToLinearIndex( next_row, next_col, grid_width );
			if ( next_mode == BranchMode::Direct )
			{
				if ( collision_count >= best_direct_collision[ next_linear_index ] )
				{
					// 对自由推进状态，只按“到这个格子的最小撞障代价”去重。
					return false;
				}
			}
			else
			{
				const int crawling_state_index = EncodeCrawlingStateIndex( next_linear_index, next_travel_direction, original_collision_direction );
				if ( crawling_state_index == -1 || collision_count >= best_crawling_collision[ crawling_state_index ] )
				{
					// 绕障状态要按“格子 + 当前绕障方向 + 原始被挡方向”联合去重。
					return false;
				}
			}

			nodes.push_back( { next_linear_index, parent_node_index, next_mode, next_travel_direction, original_collision_direction, collision_count } );
			const int node_index = static_cast<int>( nodes.size() ) - 1;
			if ( next_mode == BranchMode::Direct )
			{
				best_direct_collision[ next_linear_index ] = collision_count;
			}
			else
			{
				best_crawling_collision[ EncodeCrawlingStateIndex( next_linear_index, next_travel_direction, original_collision_direction ) ] = collision_count;
			}

			if ( high_priority )
			{
				// push_front 对应“0 代价或更希望优先处理”的推进。
				frontier.push_front( node_index );
			}
			else
			{
				// push_back 对应“新增了一次撞障/转向代价”的候选。
				frontier.push_back( node_index );
			}
			discovered_node_indices.push_back( node_index );
			return true;
		};

		best_direct_collision[ start_linear_index ] = 0;
		nodes.push_back( { start_linear_index, -1, BranchMode::Direct, 0, 0, 0 } );
		frontier.push_front( 0 );
		discovered_node_indices.push_back( 0 );

		int goal_node_index = -1;
		std::size_t emergency_cursor = 0;
		while ( true )
		{
			while ( !frontier.empty() )
			{
				const int current_node_index = frontier.front();
				frontier.pop_front();
				const PaperBranchNode current_node = nodes[ current_node_index ];
				// frontier 里允许存在陈旧状态：
				// 如果当前节点携带的 collision_count 已经不是这类状态的最优值，就直接跳过。
				if ( current_node.mode == BranchMode::Direct )
				{
					if ( current_node.collision_count != best_direct_collision[ current_node.cell_linear_index ] )
					{
						continue;
					}
				}
				else
				{
					const int crawling_state_index = EncodeCrawlingStateIndex(
						current_node.cell_linear_index,
						current_node.travel_direction,
						current_node.original_collision_direction );
					if ( crawling_state_index == -1 || current_node.collision_count != best_crawling_collision[ crawling_state_index ] )
					{
						continue;
					}
				}

				result.statistics.expanded_node_count++;
				if ( current_node.cell_linear_index == goal_linear_index )
				{
					goal_node_index = current_node_index;
					break;
				}

				const Position current_position = FromLinearIndex( current_node.cell_linear_index, grid_width );
				const char greedy_direction = ChooseGreedyDirection( current_position.row, current_position.col, goal.row, goal.col );
				const bool is_crawling = current_node.mode == BranchMode::FollowObstacle && current_node.travel_direction != 0;
				const char movement_direction = is_crawling ? current_node.travel_direction : greedy_direction;
				if ( movement_direction == 0 )
				{
					continue;
				}

				const auto movement_offset = DirectionDelta( movement_direction );
				const int next_row = current_position.row + movement_offset.first;
				const int next_col = current_position.col + movement_offset.second;

				if ( current_node.mode == BranchMode::Direct )
				{
					if ( PushNode( next_row, next_col, BranchMode::Direct, 0, 0, current_node_index, current_node.collision_count, true ) )
					{
						// 直行成功并且新状态被接受，就不再额外生成绕障分支。
						continue;
					}

					const auto branch_directions = BuildInitialBranchDirections( movement_direction, options.allow_reverse_when_crawling );
					for ( char direction_char : branch_directions )
					{
						if ( direction_char == 0 )
						{
							continue;
						}
						const auto branch_offset = DirectionDelta( direction_char );
						PushNode(
							current_position.row + branch_offset.first,
							current_position.col + branch_offset.second,
							BranchMode::FollowObstacle,
							direction_char,
							movement_direction,
							current_node_index,
							current_node.collision_count + 1,
							false );
					}
					// 只要直行未能产出有效前沿，就把撞障后的左右绕障候选全部交给队列。
					continue;
				}

				const char original_collision_direction =
					current_node.original_collision_direction == 0 ? movement_direction : current_node.original_collision_direction;
				const auto original_offset = DirectionDelta( original_collision_direction );
				const int original_row = current_position.row + original_offset.first;
				const int original_col = current_position.col + original_offset.second;

				if ( PushNode( original_row, original_col, BranchMode::Direct, 0, 0, current_node_index, current_node.collision_count, true ) )
				{
					// 一旦原先被挡住的方向恢复可走，就立刻退出 crawling，重新回到直行模式。
					continue;
				}

				if ( PushNode(
						 next_row,
						 next_col,
						 BranchMode::FollowObstacle,
						 movement_direction,
						 original_collision_direction,
						 current_node_index,
						 current_node.collision_count,
						 true ) )
				{
					// 回不去原方向时，优先继续沿当前贴边方向推进。
					continue;
				}

				const char reverse_direction = OppositeDirection( original_collision_direction );
				if ( options.allow_reverse_when_crawling && reverse_direction != 0 )
				{
					const auto reverse_offset = DirectionDelta( reverse_direction );
					if ( PushNode(
						current_position.row + reverse_offset.first,
						current_position.col + reverse_offset.second,
						BranchMode::FollowObstacle,
						reverse_direction,
						original_collision_direction,
						current_node_index,
						current_node.collision_count,
						false ) )
					{
						// 当前贴边方向也走不通时，再退一步考虑朝原方向反向绕行。
						continue;
					}
				}

				// 当严格的“回归原方向 / 继续当前方向 / 走原方向反向”都失败时，
				// 增加一次局部转角扩展。
				// 这一步相当于把 Python 参考实现里的 cornering 思想压缩成一个轻量 fallback，
				// 让论文语义版 B* 在四连通随机图上不至于过早陷入局部死角。
				const auto corner_directions = BuildFollowDirections( movement_direction, options.allow_reverse_when_crawling );
				for ( char direction_char : corner_directions )
				{
					if ( direction_char == 0 || direction_char == movement_direction )
					{
						continue;
					}
					const auto corner_offset = DirectionDelta( direction_char );
					PushNode(
						current_position.row + corner_offset.first,
						current_position.col + corner_offset.second,
						BranchMode::FollowObstacle,
						direction_char,
						original_collision_direction,
						current_node_index,
						current_node.collision_count + 1,
						false );
				}
			}

			if ( goal_node_index != -1 )
			{
				break;
			}

			bool injected_new_frontier = false;
			const std::size_t emergency_end = discovered_node_indices.size();
			for ( ; emergency_cursor < emergency_end; ++emergency_cursor )
			{
				const int seed_node_index = discovered_node_indices[ emergency_cursor ];
				const PaperBranchNode& seed_node = nodes[ seed_node_index ];
				const Position seed_position = FromLinearIndex( seed_node.cell_linear_index, grid_width );
				// emergency reinjection：
				// 当前沿耗尽但又还没到终点时，从已发现节点重新播一批 Direct 状态。
				// 这是一种工程化兜底，目的不是严格复现经典 B*，而是避免过早宣告失败。
				for ( char direction_char : BuildAllDirections() )
				{
					const auto offset = DirectionDelta( direction_char );
					injected_new_frontier = PushNode(
												seed_position.row + offset.first,
												seed_position.col + offset.second,
												BranchMode::Direct,
												0,
												0,
												seed_node_index,
												seed_node.collision_count + 1,
												false )
						|| injected_new_frontier;
				}
			}
			if ( !injected_new_frontier )
			{
				break;
			}
		}

		if ( goal_node_index != -1 )
		{
			std::vector<Position> reconstructed_path;
			for ( int trace_node_index = goal_node_index; trace_node_index != -1; trace_node_index = nodes[ trace_node_index ].parent_node_index )
			{
				reconstructed_path.push_back( FromLinearIndex( nodes[ trace_node_index ].cell_linear_index, grid_width ) );
			}
			std::reverse( reconstructed_path.begin(), reconstructed_path.end() );
			result.path = std::move( reconstructed_path );
			result.meet_position = goal;
			result.statistics.final_path_length = static_cast<int>( result.path.size() );
			result.success = ValidatePathContiguity( grid, result.path );
		}

		result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
		return result;
	}

		SearchResult RunBranchStarClassic( const Grid& grid, Position start, Position goal, const BranchStarOptions& options )
		{
		// 经典轻量版 Branch Star：
		// 1. 直行阶段只尝试朝目标方向推进
		// 2. 直行受阻时，生成左右两个贴边分支
		// 3. 贴边阶段优先尝试重新回归目标方向，否则继续沿当前绕障方向前进
		// 4. 当贴边方向也受阻时，只尝试局部转向，不再做全局 reinjection
		SearchResult result = MakeEmptyResult( AlgorithmId::BranchStarClassic );
		const auto	started_at = TimerClock::now();
		if ( !IsSearchRequestValid( grid, start, goal ) )
		{
			result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
			return result;
		}

		const int grid_height = static_cast<int>( grid.size() );
		const int grid_width = static_cast<int>( grid.front().size() );
		const int total_cells = grid_height * grid_width;
		constexpr int kInfinityCollision = std::numeric_limits<int>::max();
		const int start_linear_index = ToLinearIndex( start.row, start.col, grid_width );
		const int goal_linear_index = ToLinearIndex( goal.row, goal.col, grid_width );

		std::deque<int> frontier;
		std::vector<ClassicBranchNode> nodes;
		nodes.reserve( total_cells * 5 );
		std::vector<int> best_direct_collision( total_cells, kInfinityCollision );
		std::vector<int> best_climb_collision( total_cells * 4, kInfinityCollision );

		auto PushDirectNode = [ & ]( int next_row, int next_col, int parent_node_index, int collision_count, bool high_priority ) {
			// 轻量版把 Direct 状态和绕障状态拆成两个更简单的入队函数，
			// 这样读起来更接近“经典 B* 的直行 / 贴边”两种角色。
			if ( !IsPassable( grid, next_row, next_col ) )
			{
				return false;
			}

			const int next_linear_index = ToLinearIndex( next_row, next_col, grid_width );
			if ( collision_count >= best_direct_collision[ next_linear_index ] )
			{
				return false;
			}

			best_direct_collision[ next_linear_index ] = collision_count;
			nodes.push_back( { next_linear_index, parent_node_index, BranchMode::Direct, 0, collision_count } );
			const int node_index = static_cast<int>( nodes.size() ) - 1;
			if ( high_priority )
			{
				frontier.push_front( node_index );
			}
			else
			{
				frontier.push_back( node_index );
			}
			return true;
		};

		auto PushClimbNode = [ & ]( int next_row, int next_col, char direction_char, int parent_node_index, int collision_count, bool high_priority ) {
			if ( !IsPassable( grid, next_row, next_col ) )
			{
				return false;
			}

			const int direction_index = DirectionToIndex( direction_char );
			if ( direction_index < 0 || direction_index >= 4 )
			{
				return false;
			}

			const int next_linear_index = ToLinearIndex( next_row, next_col, grid_width );
			const int best_index = next_linear_index * 4 + direction_index;
			if ( collision_count >= best_climb_collision[ best_index ] )
			{
				return false;
			}

			best_climb_collision[ best_index ] = collision_count;
			nodes.push_back( { next_linear_index, parent_node_index, BranchMode::FollowObstacle, direction_char, collision_count } );
			const int node_index = static_cast<int>( nodes.size() ) - 1;
			if ( high_priority )
			{
				frontier.push_front( node_index );
			}
			else
			{
				frontier.push_back( node_index );
			}
			return true;
		};

		best_direct_collision[ start_linear_index ] = 0;
		nodes.push_back( { start_linear_index, -1, BranchMode::Direct, 0, 0 } );
		frontier.push_front( 0 );

		int goal_node_index = -1;
		while ( !frontier.empty() )
		{
			const int current_node_index = frontier.front();
			frontier.pop_front();

			const ClassicBranchNode current_node = nodes[ current_node_index ];
			// 轻量版同样允许旧状态残留在 frontier 中；
			// 这里通过 best_direct_collision / best_climb_collision 过滤掉陈旧条目。
			if ( current_node.mode == BranchMode::Direct )
			{
				if ( current_node.collision_count != best_direct_collision[ current_node.cell_linear_index ] )
				{
					continue;
				}
			}
			else
			{
				const int direction_index = DirectionToIndex( current_node.direction );
				if ( direction_index < 0 || direction_index >= 4 )
				{
					continue;
				}
				if ( current_node.collision_count != best_climb_collision[ current_node.cell_linear_index * 4 + direction_index ] )
				{
					continue;
				}
			}

			result.statistics.expanded_node_count++;
			if ( current_node.cell_linear_index == goal_linear_index )
			{
				goal_node_index = current_node_index;
				break;
			}

			const Position current_position = FromLinearIndex( current_node.cell_linear_index, grid_width );
			const char greedy_direction = ChooseGreedyDirection( current_position.row, current_position.col, goal.row, goal.col );

			if ( current_node.mode == BranchMode::Direct )
			{
				const auto [ row_delta, col_delta ] = DirectionDelta( greedy_direction );
				const int next_row = current_position.row + row_delta;
				const int next_col = current_position.col + col_delta;

				if ( greedy_direction != 0 && PushDirectNode( next_row, next_col, current_node_index, current_node.collision_count, true ) )
				{
					// 轻量经典版遵循最朴素的规则：直行能走，就绝不额外开分支。
					continue;
				}

				const auto branch_directions = BuildInitialBranchDirections( greedy_direction, options.allow_reverse_when_crawling );
				for ( char direction_char : branch_directions )
				{
					if ( direction_char == 0 )
					{
						continue;
					}
					const auto [ branch_row_delta, branch_col_delta ] = DirectionDelta( direction_char );
					PushClimbNode(
						current_position.row + branch_row_delta,
						current_position.col + branch_col_delta,
						direction_char,
						current_node_index,
						current_node.collision_count + 1,
						false );
				}
				continue;
			}

			if ( greedy_direction != 0 )
			{
				const auto [ row_delta, col_delta ] = DirectionDelta( greedy_direction );
				if ( PushDirectNode( current_position.row + row_delta, current_position.col + col_delta, current_node_index, current_node.collision_count, true ) )
				{
					// 绕障时一旦发现目标方向恢复可走，也立刻回归 Direct。
					continue;
				}
			}

			if ( current_node.direction != 0 )
			{
				const auto [ row_delta, col_delta ] = DirectionDelta( current_node.direction );
				if ( PushClimbNode( current_position.row + row_delta, current_position.col + col_delta, current_node.direction, current_node_index, current_node.collision_count, true ) )
				{
					continue;
				}
			}

			const auto follow_directions = BuildFollowDirections( current_node.direction == 0 ? greedy_direction : current_node.direction, options.allow_reverse_when_crawling );
			for ( char direction_char : follow_directions )
			{
				if ( direction_char == 0 || direction_char == current_node.direction )
				{
					continue;
				}
				const auto [ row_delta, col_delta ] = DirectionDelta( direction_char );
				PushClimbNode(
					current_position.row + row_delta,
					current_position.col + col_delta,
					direction_char,
					current_node_index,
					current_node.collision_count + 1,
					false );
			}
		}

		if ( goal_node_index != -1 )
		{
			std::vector<Position> reconstructed_path;
			for ( int trace_node_index = goal_node_index; trace_node_index != -1; trace_node_index = nodes[ trace_node_index ].parent_node_index )
			{
				reconstructed_path.push_back( FromLinearIndex( nodes[ trace_node_index ].cell_linear_index, grid_width ) );
			}
			std::reverse( reconstructed_path.begin(), reconstructed_path.end() );
			result.path = std::move( reconstructed_path );
			result.meet_position = goal;
			result.statistics.final_path_length = static_cast<int>( result.path.size() );
			result.success = ValidatePathContiguity( grid, result.path );
		}

		result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
		return result;
	}

		SearchResult RunBranchStarLegacy( const Grid& grid, Position start, Position goal, const BranchStarOptions& options )
		{
		// 这是保留下来的旧版 / 增强版 Branch Star。
		// 它带有更重的状态建模、全局 reinjection 和更强的兜底行为，
		// 便于和新的经典轻量版做对比。
		SearchResult result = MakeEmptyResult( AlgorithmId::BranchStarLegacy );
		const auto	started_at = TimerClock::now();
		if ( !IsSearchRequestValid( grid, start, goal ) )
		{
			result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
			return result;
		}

		const int grid_height = static_cast<int>( grid.size() );
		const int grid_width = static_cast<int>( grid.front().size() );
		const int total_cells = grid_height * grid_width;
		constexpr int kModeCount = 2;
		constexpr int kDirectionCount = 5;
		constexpr int kInfinityCollision = std::numeric_limits<int>::max();
		const int total_state_count = total_cells * kModeCount * kDirectionCount;
		const int start_linear_index = ToLinearIndex( start.row, start.col, grid_width );
		const int goal_linear_index = ToLinearIndex( goal.row, goal.col, grid_width );

		std::deque<std::pair<int, int>> frontier;
		std::vector<int> best_collision( total_state_count, kInfinityCollision );
		std::vector<int> parent_state( total_state_count, -1 );
		std::vector<int> best_cell_collision( total_cells, kInfinityCollision );
		std::vector<int> best_cell_state( total_cells, -1 );
		std::vector<int> discovered_cells;

		const int start_state_id = EncodeBranchStateId( start_linear_index, static_cast<std::uint8_t>( BranchMode::Direct ), 4 );
		best_collision[ start_state_id ] = 0;
		best_cell_collision[ start_linear_index ] = 0;
		best_cell_state[ start_linear_index ] = start_state_id;
		discovered_cells.push_back( start_linear_index );
		frontier.push_front( { start_state_id, 0 } );

		// 尝试把一个新的 Branch Star 状态压入前沿。
		// 这里统一处理边界检查、状态去重、parent 记录，以及 0/1 代价的前后入队。
		auto TryPushState = [ & ]( int next_row, int next_col, BranchMode next_mode, char next_direction, int parent_state_id, int next_collision, bool high_priority ) {
			// Legacy 版把状态压成一个整数 state_id，再用数组保存最优碰撞代价。
			// 好处是实现统一、兜底能力强；代价是状态空间更大，常数也更高。
			if ( !IsPassable( grid, next_row, next_col ) )
			{
				return false;
			}

			const int next_linear_index = ToLinearIndex( next_row, next_col, grid_width );
			const int next_state_id = EncodeBranchStateId( next_linear_index, static_cast<std::uint8_t>( next_mode ), static_cast<std::uint8_t>( DirectionToIndex( next_direction ) ) );
			if ( next_collision >= best_collision[ next_state_id ] )
			{
				return false;
			}

			best_collision[ next_state_id ] = next_collision;
			parent_state[ next_state_id ] = parent_state_id;
			if ( next_collision < best_cell_collision[ next_linear_index ] )
			{
				if ( best_cell_collision[ next_linear_index ] == kInfinityCollision )
				{
					discovered_cells.push_back( next_linear_index );
				}
				best_cell_collision[ next_linear_index ] = next_collision;
				best_cell_state[ next_linear_index ] = next_state_id;
			}
			if ( high_priority )
			{
				frontier.push_front( { next_state_id, next_collision } );
			}
			else
			{
				frontier.push_back( { next_state_id, next_collision } );
			}
			return true;
		};

		auto ExpandState = [ & ]( int current_state_id ) {
			const BranchStateNode current_state = DecodeBranchStateId( current_state_id );
			const int current_collision = best_collision[ current_state_id ];
			const Position current_position = FromLinearIndex( current_state.cell_linear_index, grid_width );

			const BranchMode mode = static_cast<BranchMode>( current_state.mode_index );
			const char current_direction = IndexToDirection( current_state.direction_index );
			const char greedy_direction = ChooseGreedyDirection( current_position.row, current_position.col, goal.row, goal.col );

			if ( mode == BranchMode::Direct )
			{
				const auto [ greedy_row_delta, greedy_col_delta ] = DirectionDelta( greedy_direction );
				const int greedy_row = current_position.row + greedy_row_delta;
				const int greedy_col = current_position.col + greedy_col_delta;

				if ( greedy_direction != 0 && IsPassable( grid, greedy_row, greedy_col ) )
				{
					// Legacy 版仍然保留 B* 的核心精神：自由状态下优先朝目标方向推进。
					TryPushState( greedy_row, greedy_col, BranchMode::Direct, 0, current_state_id, current_collision, true );
					return;
				}

				bool spawned_branch = false;
				const auto branch_directions = BuildInitialBranchDirections( greedy_direction, options.allow_reverse_when_crawling );
				for ( char direction_char : branch_directions )
				{
					if ( direction_char == 0 )
					{
						continue;
					}
					const auto [ row_delta, col_delta ] = DirectionDelta( direction_char );
					spawned_branch = TryPushState( current_position.row + row_delta, current_position.col + col_delta, BranchMode::FollowObstacle, direction_char, current_state_id, current_collision + 1, false )
						|| spawned_branch;
				}

				if ( !spawned_branch )
				{
					for ( char direction_char : BuildAllDirections() )
					{
						const auto [ row_delta, col_delta ] = DirectionDelta( direction_char );
						TryPushState( current_position.row + row_delta, current_position.col + col_delta, BranchMode::FollowObstacle, direction_char, current_state_id, current_collision + 1, false );
					}
				}
				return;
			}

			const auto [ greedy_row_delta, greedy_col_delta ] = DirectionDelta( greedy_direction );
			const int greedy_row = current_position.row + greedy_row_delta;
			const int greedy_col = current_position.col + greedy_col_delta;
			if ( greedy_direction != 0 && IsPassable( grid, greedy_row, greedy_col ) )
			{
				TryPushState( greedy_row, greedy_col, BranchMode::Direct, 0, current_state_id, current_collision, true );
				return;
			}

			if ( current_direction != 0 )
			{
				const auto [ continue_row_delta, continue_col_delta ] = DirectionDelta( current_direction );
				const int continue_row = current_position.row + continue_row_delta;
				const int continue_col = current_position.col + continue_col_delta;
				if ( IsPassable( grid, continue_row, continue_col ) )
				{
					TryPushState( continue_row, continue_col, BranchMode::FollowObstacle, current_direction, current_state_id, current_collision, true );
					return;
				}
			}

			bool spawned_turn = false;
			const auto ordered_directions = BuildFollowDirections( current_direction == 0 ? greedy_direction : current_direction, options.allow_reverse_when_crawling );
			for ( char direction_char : ordered_directions )
			{
				if ( direction_char == 0 )
				{
					continue;
				}
				const auto [ row_delta, col_delta ] = DirectionDelta( direction_char );
				spawned_turn = TryPushState( current_position.row + row_delta, current_position.col + col_delta, BranchMode::FollowObstacle, direction_char, current_state_id, current_collision + 1, false )
					|| spawned_turn;
			}

			if ( !spawned_turn )
			{
				for ( char direction_char : BuildAllDirections() )
				{
					const auto [ row_delta, col_delta ] = DirectionDelta( direction_char );
					TryPushState( current_position.row + row_delta, current_position.col + col_delta, BranchMode::FollowObstacle, direction_char, current_state_id, current_collision + 1, false );
				}
			}
		};

		auto ProcessFrontier = [ & ]( int& goal_state_id ) {
			while ( !frontier.empty() )
			{
				const auto [ current_state_id, popped_collision ] = frontier.front();
				frontier.pop_front();
				// deque 里同样可能有旧状态，这里通过 best_collision 做最后一道筛选。
				if ( popped_collision != best_collision[ current_state_id ] )
				{
					continue;
				}

				const BranchStateNode current_state = DecodeBranchStateId( current_state_id );
				result.statistics.expanded_node_count++;
				if ( current_state.cell_linear_index == goal_linear_index )
				{
					goal_state_id = current_state_id;
					return;
				}

				ExpandState( current_state_id );
			}
		};

		int goal_state_id = -1;
		while ( goal_state_id == -1 )
		{
			ProcessFrontier( goal_state_id );
			if ( goal_state_id != -1 )
			{
				break;
			}

			bool injected_any = false;
			const std::size_t discovered_count_snapshot = discovered_cells.size();
			for ( std::size_t discovered_index = 0; discovered_index < discovered_count_snapshot; ++discovered_index )
			{
				const int cell_linear_index = discovered_cells[ discovered_index ];
				const int parent_state_id = best_cell_state[ cell_linear_index ];
				const int base_collision = best_cell_collision[ cell_linear_index ];
				if ( parent_state_id == -1 || base_collision == kInfinityCollision )
				{
					continue;
				}

				const Position base_position = FromLinearIndex( cell_linear_index, grid_width );
				// Legacy 版的全局 reinjection 更重：
				// 它会从所有已发现格子重新注入 Direct 状态，因此鲁棒性更强，但速度通常更慢。
				for ( char direction_char : BuildAllDirections() )
				{
					const auto [ row_delta, col_delta ] = DirectionDelta( direction_char );
					injected_any = TryPushState( base_position.row + row_delta, base_position.col + col_delta, BranchMode::Direct, 0, parent_state_id, base_collision + 1, false ) || injected_any;
				}
			}

			if ( !injected_any )
			{
				break;
			}
		}

		if ( goal_state_id != -1 )
		{
			std::vector<Position> reconstructed_path;
			for ( int trace_state_id = goal_state_id; trace_state_id != -1; trace_state_id = parent_state[ trace_state_id ] )
			{
				const BranchStateNode trace_node = DecodeBranchStateId( trace_state_id );
				reconstructed_path.push_back( FromLinearIndex( trace_node.cell_linear_index, grid_width ) );
			}
			std::reverse( reconstructed_path.begin(), reconstructed_path.end() );
			result.path = std::move( reconstructed_path );
			result.meet_position = goal;
			result.statistics.final_path_length = static_cast<int>( result.path.size() );
			result.success = ValidatePathContiguity( grid, result.path );
		}

		result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
		return result;
	}

	// 把现有的 IBP-B* 核心实现包装成统一比较接口。
	// 这样它就能和 BFS、A*、Dijkstra、Branch Star 使用同一套输入输出结构。
	SearchResult RunIbpBStar( const Grid& grid, Position start, Position goal, const IbpBStarOptions& options )
	{
		// 这里单独包装你当前的 IBP-B* 实现，保证它和 Branch Star 在对比框架里是两个算法。
		SearchResult result = MakeEmptyResult( AlgorithmId::IbpBStar );
		const auto	started_at = TimerClock::now();
		if ( !IsSearchRequestValid( grid, start, goal ) )
		{
			result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
			return result;
		}

		const IBP_BStarAlgorithm::CellPosition legacy_start { start.row, start.col };
		const IBP_BStarAlgorithm::CellPosition legacy_goal { goal.row, goal.col };
		IBP_BStarAlgorithm::AlgorithmOptions legacy_options;
		legacy_options.enable_local_zigzag_expansion = options.enable_local_zigzag_expansion;
		legacy_options.zigzag_threshold = options.zigzag_threshold;

		IBP_BStarAlgorithm::SearchOutcome legacy_outcome;
		if ( options.enable_maze_rescue )
		{
			// 增强入口：主算法失败或路径无效时，允许回退到 zigzag rescue。
			legacy_outcome = IBP_BStarAlgorithm::RunIbpBStarZigzagEnhanced( grid, legacy_start, legacy_goal, options.wait_layers, legacy_options );
		}
		else
		{
			// 严格版入口：只运行 IBP-B* 核心，不启用独立的迷宫补救搜索。
			legacy_outcome = IBP_BStarAlgorithm::RunIbpBStar( grid, legacy_start, legacy_goal, options.wait_layers, legacy_options );
		}

		result.path.reserve( legacy_outcome.final_path.size() );
		for ( const auto& legacy_position : legacy_outcome.final_path )
		{
			result.path.push_back( { legacy_position.row, legacy_position.col } );
		}
		result.meet_position = { legacy_outcome.meet_position.row, legacy_outcome.meet_position.col };
		result.statistics.expanded_node_count = legacy_outcome.statistics.expanded_node_count;
		result.statistics.final_path_length = static_cast<int>( result.path.size() );
		result.success = legacy_outcome.success && ValidatePathContiguity( grid, result.path );
		result.statistics.elapsed_microseconds = MeasureElapsedMicroseconds( started_at );
		return result;
	}

	// 统一算法分发入口。
	// main 层只需要传入枚举值，不需要知道每种算法内部的具体实现细节。
	SearchResult RunAlgorithm( AlgorithmId algorithm_id, const Grid& grid, Position start, Position goal, const AlgorithmOptions& options )
	{
		switch ( algorithm_id )
		{
		case AlgorithmId::Bfs: return RunBfs( grid, start, goal );
		case AlgorithmId::AStar: return RunAStar( grid, start, goal );
		case AlgorithmId::Dijkstra: return RunDijkstra( grid, start, goal );
		case AlgorithmId::BranchStar: return RunBranchStar( grid, start, goal, options.branch_star );
		case AlgorithmId::BranchStarClassic: return RunBranchStarClassic( grid, start, goal, options.branch_star );
		case AlgorithmId::BranchStarLegacy: return RunBranchStarLegacy( grid, start, goal, options.branch_star );
		case AlgorithmId::IbpBStar: return RunIbpBStar( grid, start, goal, options.ibp_bstar );
		default: return MakeEmptyResult( algorithm_id );
		}
	}
}  // namespace pathfinding
