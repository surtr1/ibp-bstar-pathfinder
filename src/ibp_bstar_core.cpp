#include "ibp_bstar_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <string>

namespace IBP_BStarAlgorithm
{
	// ============================================================================
	// 算法核心实现说明
	//
	// 本文件只保留“真正属于算法”的内容：
	// 1. IBP-B* 双向搜索核心
	// 2. Zigzag 迷宫补救搜索
	// 3. 路径合法性校验与方向辅助函数
	//
	// 不放在这里的内容包括：
	// - 地图文件读取
	// - 随机地图生成
	// - 控制台渲染
	// - 命令行参数解析
	//
	// 这样拆分后，后续如果要做多算法 benchmark，可以直接复用本文件中的函数，
	// 而不需要带上 CLI 或渲染逻辑。
	// ============================================================================
	namespace
	{
		// 四个基本方向按固定索引存放，避免热路径里使用哈希查表。
		constexpr std::array<char, 4> kDirectionPriorityOrder = { 'U', 'D', 'L', 'R' };
		constexpr std::array<std::pair<int, int>, 4> kDirectionOffsets = { { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } } };

		constexpr int DirectionToIndex( char direction_char )
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

		constexpr char IndexToDirection( int direction_index )
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

		constexpr std::pair<int, int> DirectionDelta( char direction_char )
		{
			const int direction_index = DirectionToIndex( direction_char );
			if ( direction_index < 0 || direction_index >= static_cast<int>( kDirectionOffsets.size() ) )
			{
				return { 0, 0 };
			}
			return kDirectionOffsets[ direction_index ];
		}

		// 自由模式：按贪心方向推进。
		// Crawling 模式：已经撞上障碍，沿障碍边缘尝试绕行。
		enum class ExplorationMode : std::uint8_t
		{
			Free,
			Crawling
		};

		enum class ZigzagTurnBias : std::uint8_t
		{
			PreferLeft = 0,
			PreferRight = 1
		};

		enum class ZigzagPhase : std::uint8_t
		{
			Normal = 0,
			AfterCorner = 1
		};

		struct ZigzagStateNode
		{
			// cell_linear_index: 当前所在格子
			// last_direction_index: 上一步移动方向
			// turn_bias_index: 左优先 / 右优先
			// phase_index: 是否刚经过一个拐角
			int			 cell_linear_index = -1;
			std::uint8_t last_direction_index = 4;
			std::uint8_t turn_bias_index = 0;
			std::uint8_t phase_index = 0;
		};

		// 在一个小窗口内做局部 BFS，估计某个候选方向附近的可展开空间。
		// 这个分数不是全局可达性，只是用来比较“继续朝前”和“转向侧面”哪一边更像值得预探索的入口。
		int LocalWindowReachableCount( const Grid& grid, int start_row, int start_col, int center_row, int center_col, int window_radius )
		{
			if ( !IsCellPassable( grid, start_row, start_col ) )
			{
				return 0;
			}

			const int window_size = window_radius * 2 + 1;
			std::vector<std::uint8_t> visited( window_size * window_size, 0 );
			std::deque<std::pair<int, int>> queue;
			auto InWindow = [ & ]( int row, int col ) {
				return std::abs( row - center_row ) <= window_radius && std::abs( col - center_col ) <= window_radius;
			};
			auto ToVisitedIndex = [ & ]( int row, int col ) {
				return ( row - center_row + window_radius ) * window_size + ( col - center_col + window_radius );
			};

			queue.push_back( { start_row, start_col } );
			visited[ ToVisitedIndex( start_row, start_col ) ] = 1;

			int reachable_count = 0;
			while ( !queue.empty() )
			{
				const auto [ current_row, current_col ] = queue.front();
				queue.pop_front();
				++reachable_count;

				for ( const auto& [ row_offset, col_offset ] : kDirectionOffsets )
				{
					const int next_row = current_row + row_offset;
					const int next_col = current_col + col_offset;
					if ( !InWindow( next_row, next_col ) || !IsCellPassable( grid, next_row, next_col ) )
					{
						continue;
					}

					const int visited_index = ToVisitedIndex( next_row, next_col );
					if ( visited[ visited_index ] != 0 )
					{
						continue;
					}

					visited[ visited_index ] = 1;
					queue.push_back( { next_row, next_col } );
				}
			}
			return reachable_count;
		}

		// 先用常数级局部形状做一轮粗筛，避免在普通开阔区里频繁跑窗口 BFS。
		// 只有当“当前点较受约束，且下一格横向空间变多或正前方开始受限”时，才值得进入更重的局部可达性估计。
		bool ShouldProbeConcaveLocally( const Grid& grid, int current_row, int current_col, int next_row, int next_col, char greedy_direction )
		{
			if ( greedy_direction == 0 )
			{
				return false;
			}

			const auto [ left_dir_char, right_dir_char ] = LeftRightDirections( greedy_direction );
			const auto left_offset = DirectionDelta( left_dir_char );
			const auto right_offset = DirectionDelta( right_dir_char );
			const auto forward_offset = DirectionDelta( greedy_direction );

			const bool current_left_open = IsCellPassable( grid, current_row + left_offset.first, current_col + left_offset.second );
			const bool current_right_open = IsCellPassable( grid, current_row + right_offset.first, current_col + right_offset.second );
			const bool next_left_open = IsCellPassable( grid, next_row + left_offset.first, next_col + left_offset.second );
			const bool next_right_open = IsCellPassable( grid, next_row + right_offset.first, next_col + right_offset.second );
			const bool forward_open = IsCellPassable( grid, next_row + forward_offset.first, next_col + forward_offset.second );

			const int current_side_open_count = ( current_left_open ? 1 : 0 ) + ( current_right_open ? 1 : 0 );
			const int next_side_open_count = ( next_left_open ? 1 : 0 ) + ( next_right_open ? 1 : 0 );

			// 当前点至少要有一点“贴着障碍走”的受约束感，否则大多数普通开阔区都没必要做预探索。
			const bool current_is_constrained = current_side_open_count < 2;
			// 下一格横向必须有可展开空间，否则即使跑 BFS 也只会得到一个狭窄直通道。
			const bool next_has_side_space = next_side_open_count > 0;
			// 下一格比当前点更“张开”，说明这里更像一个入口或侧向口袋。
			const bool lateral_space_increases = next_side_open_count > current_side_open_count;
			// 如果正前方已经开始受限，也值得进一步确认这里是不是侧向岔口。
			const bool forward_becomes_constrained = !forward_open;

			return current_is_constrained && next_has_side_space && ( lateral_space_increases || forward_becomes_constrained );
		}

		// 用“粗筛 + 局部窗口 BFS”替代原来的“单格凹口”规则。
		// 只有当粗筛认为这里像入口样式时，才进一步比较前方与侧向的局部可展开空间。
		bool IsConcaveEntry( const Grid& grid, int current_row, int current_col, int next_row, int next_col, char greedy_direction )
		{
			if ( !ShouldProbeConcaveLocally( grid, current_row, current_col, next_row, next_col, greedy_direction ) )
			{
				return false;
			}

			const auto [ left_dir_char, right_dir_char ] = LeftRightDirections( greedy_direction );
			const auto left_offset = DirectionDelta( left_dir_char );
			const auto right_offset = DirectionDelta( right_dir_char );
			const auto forward_offset = DirectionDelta( greedy_direction );

			const int window_radius = 3;
			const int left_row = next_row + left_offset.first;
			const int left_col = next_col + left_offset.second;
			const int right_row = next_row + right_offset.first;
			const int right_col = next_col + right_offset.second;
			const int forward_row = next_row + forward_offset.first;
			const int forward_col = next_col + forward_offset.second;

			const int left_area = LocalWindowReachableCount( grid, left_row, left_col, next_row, next_col, window_radius );
			const int right_area = LocalWindowReachableCount( grid, right_row, right_col, next_row, next_col, window_radius );
			const int forward_area = LocalWindowReachableCount( grid, forward_row, forward_col, next_row, next_col, window_radius );
			const int side_area = std::max( left_area, right_area );

			return side_area >= 4 && side_area >= forward_area + 2;
		}

		// 预测前方几步是否会迅速收窄成窄道。
		// 这里只替换 rebirth 的触发判定，不改 rebirth 触发后的行为。
		bool HasNarrowPassageRisk( const Grid& grid, int row, int col, char direction_char )
		{
			if ( direction_char == 0 || !IsCellPassable( grid, row, col ) )
			{
				return false;
			}

			const auto [ left_dir_char, right_dir_char ] = LeftRightDirections( direction_char );
			const auto left_offset = DirectionDelta( left_dir_char );
			const auto right_offset = DirectionDelta( right_dir_char );
			const auto forward_offset = DirectionDelta( direction_char );

			int corridor_like_steps = 0;
			int sealed_steps = 0;
			int current_row = row;
			int current_col = col;
			for ( int step = 0; step < 3; ++step )
			{
				if ( !IsCellPassable( grid, current_row, current_col ) )
				{
					break;
				}

				const bool left_open = IsCellPassable( grid, current_row + left_offset.first, current_col + left_offset.second );
				const bool right_open = IsCellPassable( grid, current_row + right_offset.first, current_col + right_offset.second );
				if ( !left_open || !right_open )
				{
					++corridor_like_steps;
				}
				if ( !left_open && !right_open )
				{
					++sealed_steps;
				}

				current_row += forward_offset.first;
				current_col += forward_offset.second;
			}

			const bool forward_escape_is_limited = !IsCellPassable( grid, current_row, current_col );
			return corridor_like_steps >= 2 && ( sealed_steps >= 1 || forward_escape_is_limited );
		}

		std::array<char, 4> BuildZigzagStateDirectionPriority( char last_direction, ZigzagTurnBias turn_bias, ZigzagPhase phase,
															   int current_row, int current_col, int goal_row, int goal_col )
		{
			// Zigzag 补救搜索的关键之一，是把“下一步尝试顺序”做成一个状态相关的优先级表。
			// 与普通 BFS 不同，这里虽然仍按层扩展，但每个状态的出边顺序会体现“延续前进、偏向转角、再考虑回退”的倾向。
			std::array<char, 4> ordered_directions { 0, 0, 0, 0 };
			int					write_index = 0;
			auto PushUnique = [ & ]( char direction_char ) {
				if ( direction_char == 0 )
					return;
				for ( int idx = 0; idx < write_index; ++idx )
				{
					if ( ordered_directions[ idx ] == direction_char )
						return;
				}
				if ( write_index < static_cast<int>( ordered_directions.size() ) )
				{
					ordered_directions[ write_index++ ] = direction_char;
				}
			};

			const char greedy_direction = ChooseGreedyDirection( current_row, current_col, goal_row, goal_col );
			if ( last_direction == 0 )
			{
				auto [ left_dir_char, right_dir_char ] = LeftRightDirections( greedy_direction );
				PushUnique( greedy_direction );
				if ( turn_bias == ZigzagTurnBias::PreferLeft )
				{
					PushUnique( left_dir_char );
					PushUnique( right_dir_char );
				}
				else
				{
					PushUnique( right_dir_char );
					PushUnique( left_dir_char );
				}
				PushUnique( OppositeDirection( greedy_direction ) );
			}
			else
			{
				auto [ left_dir_char, right_dir_char ] = LeftRightDirections( last_direction );
				const char preferred_turn = ( turn_bias == ZigzagTurnBias::PreferLeft ) ? left_dir_char : right_dir_char;
				const char secondary_turn = ( turn_bias == ZigzagTurnBias::PreferLeft ) ? right_dir_char : left_dir_char;
				const char reverse_direction = OppositeDirection( last_direction );

				PushUnique( last_direction );
				if ( phase == ZigzagPhase::AfterCorner )
				{
					PushUnique( preferred_turn );
					PushUnique( secondary_turn );
					PushUnique( greedy_direction );
				}
				else
				{
					PushUnique( greedy_direction );
					PushUnique( preferred_turn );
					PushUnique( secondary_turn );
				}
				PushUnique( reverse_direction );
			}

			for ( char direction_char : kDirectionPriorityOrder )
			{
				PushUnique( direction_char );
			}
			return ordered_directions;
		}

		int EncodeZigzagStateId( int cell_linear_index, std::uint8_t last_direction_index, std::uint8_t turn_bias_index, std::uint8_t phase_index )
		{
			// 把复合状态压成一个整数，便于用一维数组维护距离和 parent。
			constexpr int kDirectionStateCount = 5;
			constexpr int kBiasStateCount = 2;
			constexpr int kPhaseStateCount = 2;
			return (((cell_linear_index * kDirectionStateCount + static_cast<int>( last_direction_index )) * kBiasStateCount + static_cast<int>( turn_bias_index )) * kPhaseStateCount
					+ static_cast<int>( phase_index ));
		}

		ZigzagStateNode DecodeZigzagStateId( int state_id )
		{
			// 与 Encode 配套，把压缩状态恢复成结构体，方便访问各个分量。
			constexpr int kPhaseStateCount = 2;
			constexpr int kBiasStateCount = 2;
			constexpr int kDirectionStateCount = 5;
			ZigzagStateNode node;
			node.phase_index = static_cast<std::uint8_t>( state_id % kPhaseStateCount );
			state_id /= kPhaseStateCount;
			node.turn_bias_index = static_cast<std::uint8_t>( state_id % kBiasStateCount );
			state_id /= kBiasStateCount;
			node.last_direction_index = static_cast<std::uint8_t>( state_id % kDirectionStateCount );
			state_id /= kDirectionStateCount;
			node.cell_linear_index = state_id;
			return node;
		}

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

		bool IsCellInsideGrid( const Grid& grid, const CellPosition& pos )
		{
			return pos.row >= 0 && pos.row < static_cast<int>( grid.size() ) && pos.col >= 0 && pos.col < static_cast<int>( grid.front().size() );
		}
	}  // namespace

	int ToLinearIndex( int row, int col, int grid_width )
	{
		// 二维坐标转为一维索引，方便使用连续数组而不是哈希表。
		return row * grid_width + col;
	}

	std::pair<int, int> FromLinearIndex( int linear_index, int grid_width )
	{
		return { linear_index / grid_width, linear_index % grid_width };
	}

	bool IsCellPassable( const Grid& grid, int row, int col )
	{
		// 越界也视为不可通行，这样调用方不用反复写边界判断。
		return row >= 0 && row < static_cast<int>( grid.size() ) && col >= 0 && col < static_cast<int>( grid[ 0 ].size() ) && grid[ row ][ col ] == 0;
	}

	bool IsCellBlocked( const Grid& grid, int row, int col )
	{
		return !IsCellPassable( grid, row, col );
	}

	char OppositeDirection( char direction_char )
	{
		switch ( direction_char )
		{
		case 'U': return 'D';
		case 'D': return 'U';
		case 'L': return 'R';
		default: return 'L';
		}
	}

	std::pair<char, char> LeftRightDirections( char direction_char )
	{
		switch ( direction_char )
		{
		case 'U': return { 'L', 'R' };
		case 'D': return { 'R', 'L' };
		case 'L': return { 'D', 'U' };
		default: return { 'U', 'D' };
		}
	}

	char ChooseGreedyDirection( int current_row, int current_col, int goal_row, int goal_col )
	{
		// 采用“绝对差更大者优先”的简单贪心策略。
		// 这不是最短路启发式，而是论文风格实现里的局部前进方向选择。
		const int delta_row = goal_row - current_row;
		const int delta_col = goal_col - current_col;
		if ( std::abs( delta_row ) >= std::abs( delta_col ) )
		{
			return ( delta_row > 0 ) ? 'D' : 'U';
		}
		return ( delta_col > 0 ) ? 'R' : 'L';
	}

	bool ValidatePathContiguity( const Grid& grid, const std::vector<CellPosition>& path )
	{
		// 该函数既可用于算法结果校验，也可用于 benchmark 阶段排查非法路径。
		if ( path.empty() )
			return false;

		for ( std::size_t idx = 0; idx < path.size(); ++idx )
		{
			const CellPosition& current = path[ idx ];
			if ( !IsCellPassable( grid, current.row, current.col ) )
				return false;
			if ( idx == 0 )
				continue;
			const CellPosition& previous = path[ idx - 1 ];
			const int			manhattan = std::abs( current.row - previous.row ) + std::abs( current.col - previous.col );
			if ( manhattan != 1 )
				return false;
		}
		return true;
	}

	SearchOutcome RunIbpBStar( const Grid& grid, CellPosition start_pos, CellPosition goal_pos, int wait_layers, const AlgorithmOptions& options )
	{
		// 核心流程：
		// 1. 起点和终点双向同时扩展
		// 2. 每侧都优先按贪心方向推进
		// 3. 撞障后转入 crawling 模式沿边绕行
		// 4. 两侧相遇后继续做有限 flush，再重建路径
		SearchOutcome outcome;
		if ( !IsGridShapeValid( grid ) || !IsCellInsideGrid( grid, start_pos ) || !IsCellInsideGrid( grid, goal_pos ) )
			return outcome;
		if ( !IsCellPassable( grid, start_pos.row, start_pos.col ) || !IsCellPassable( grid, goal_pos.row, goal_pos.col ) )
			return outcome;

		const int grid_height = static_cast<int>( grid.size() );
		const int grid_width = static_cast<int>( grid[ 0 ].size() );
		const int total_cells = grid_height * grid_width;

		auto ToIndex = [ & ]( int row, int col ) {
			// 局部封装，避免在下面的代码里反复传 grid_width。
			return ToLinearIndex( row, col, grid_width );
		};
		auto FromIndex = [ & ]( int linear_index ) {
			// 与 ToIndex 配对，用于把 parent 链重新转回坐标路径。
			auto [ row, col ] = FromLinearIndex( linear_index, grid_width );
			return CellPosition { row, col };
		};

		using DepthArray = std::vector<int>;
		using ParentArray = std::vector<int>;
		using NodeQueue = std::deque<int>;

		struct ObstacleState
		{
			// mode: 当前是否处于自由推进 / 沿障碍绕行
			// travel_direction: 若在绕障中，当前沿哪个方向继续贴边走
			// original_collision_direction: 最初撞到障碍时，本来想去的方向
			// rebirth_used: 是否已经对该结点做过一次“重生式重新扩展”
			// obstacle_hit_count: 连续障碍交互次数，用于决定何时触发局部 zigzag 扩展
			ExplorationMode mode = ExplorationMode::Free;
			char			travel_direction = 0;
			char			original_collision_direction = 0;
			bool			rebirth_used = false;
			int				obstacle_hit_count = 0;
		};

		DepthArray				 depth_from_start( total_cells, -1 );
		DepthArray				 depth_from_goal( total_cells, -1 );
		ParentArray				 parent_from_start( total_cells, -1 );
		ParentArray				 parent_from_goal( total_cells, -1 );

		// 两侧各维护一份 obstacle state。
		// 同一个格子从起点侧和终点侧过来时，绕障上下文可能完全不同，因此不能共用。
		std::vector<ObstacleState> state_from_start( total_cells );
		std::vector<ObstacleState> state_from_goal( total_cells );
		NodeQueue				 frontier_queue_from_start;
		NodeQueue				 frontier_queue_from_goal;

		const int start_linear_index = ToIndex( start_pos.row, start_pos.col );
		const int goal_linear_index = ToIndex( goal_pos.row, goal_pos.col );

		depth_from_start[ start_linear_index ] = 0;
		depth_from_goal[ goal_linear_index ] = 0;
		frontier_queue_from_start.push_back( start_linear_index );
		frontier_queue_from_goal.push_back( goal_linear_index );

		// meet_* 系列变量用于记录当前最优会合点。
		int				 meet_linear_index = -1;
		int				 meet_depth_from_start = -1;
		int				 meet_depth_from_goal = -1;
		int				 best_meet_total_depth = -1;
		SearchStatistics stats;

		auto UpdateMeet = [ & ]( int node_linear_index, DepthArray& this_side_depth, DepthArray& other_side_depth ) {
			// 只要当前点被另一侧访问过，就形成一个候选相遇点。
			// 这里维护总代价更小的相遇位置，而不是第一次相遇就立刻停止。
			if ( other_side_depth[ node_linear_index ] == -1 )
				return;
			const int this_depth_value = this_side_depth[ node_linear_index ];
			const int other_depth_value = other_side_depth[ node_linear_index ];
			const int total_depth_value = this_depth_value + other_depth_value;
			if ( meet_linear_index == -1 || total_depth_value < best_meet_total_depth )
			{
				meet_linear_index = node_linear_index;
				best_meet_total_depth = total_depth_value;
				if ( &this_side_depth == &depth_from_start )
				{
					meet_depth_from_start = this_depth_value;
					meet_depth_from_goal = other_depth_value;
				}
				else
				{
					meet_depth_from_goal = this_depth_value;
					meet_depth_from_start = other_depth_value;
				}
			}
		};

		auto SideShouldContinueFlush = [ & ]( const NodeQueue& queue, const DepthArray& depth_array, int meet_depth ) {
			// 论文风格实现中，在相遇之后并不会立刻停下，
			// 而是继续把相遇层附近的若干层 flush 掉，以获得更稳定的会合结果。
			if ( queue.empty() || meet_depth < 0 )
				return false;
			return depth_array[ queue.front() ] <= ( meet_depth + wait_layers );
		};

		auto TryVisit = [ & ]( int next_row, int next_col, int parent_index, int next_depth, const ObstacleState& next_state,
							   DepthArray& this_depth, ParentArray& this_parent, std::vector<ObstacleState>& this_state,
							   NodeQueue& this_queue, DepthArray& opposite_depth ) {
			// 所有入队都统一经过这里，确保“合法、未访问、写入 parent、更新 meet”的流程一致。
			if ( !IsCellPassable( grid, next_row, next_col ) )
				return false;
			const int next_index = ToIndex( next_row, next_col );
			if ( this_depth[ next_index ] != -1 )
				return false;

			this_depth[ next_index ] = next_depth;
			this_parent[ next_index ] = parent_index;
			this_state[ next_index ] = next_state;
			this_queue.push_back( next_index );
			UpdateMeet( next_index, this_depth, opposite_depth );
			return true;
		};

		auto ExpandFrontier = [ & ]( int current_linear_index, int greedy_goal_row, int greedy_goal_col,
									 DepthArray& this_depth, DepthArray& opposite_depth, ParentArray& this_parent,
									 std::vector<ObstacleState>& this_state, NodeQueue& this_queue ) {
			// 这是 IBP-B* 的核心扩展器：
			// 自由状态下优先向目标推进，遇障后切换为 crawling；
			// crawling 状态下优先尝试回归原始碰撞方向，否则继续贴边走，最后才考虑反向或 zigzag 扩展。
			stats.expanded_node_count++;

			const CellPosition current_pos = FromIndex( current_linear_index );
			const int		   current_row = current_pos.row;
			const int		   current_col = current_pos.col;
			const ObstacleState& current_state = this_state[ current_linear_index ];
			const int		     current_depth_value = this_depth[ current_linear_index ];
			const int		     next_depth_value = current_depth_value + 1;

			const char greedy_direction = ChooseGreedyDirection( current_row, current_col, greedy_goal_row, greedy_goal_col );
			const bool is_crawling = current_state.mode == ExplorationMode::Crawling && current_state.travel_direction != 0;

			// 若已经在 crawling，就延续贴边方向；否则采用新的贪心方向。
			const char movement_direction = is_crawling ? current_state.travel_direction : greedy_direction;
			const auto movement_offset = DirectionDelta( movement_direction );
			const int next_row = current_row + movement_offset.first;
			const int next_col = current_col + movement_offset.second;

			if ( current_state.mode == ExplorationMode::Free )
			{
				if ( IsCellPassable( grid, next_row, next_col ) )
				{
					// 正常贪心前进一步。
					// 如果局部预测显示前方几步会迅速收窄成窄道，则把当前点重新压回队列，
					// 给它一次“重生式”再扩展机会。
					if ( !current_state.rebirth_used && HasNarrowPassageRisk( grid, next_row, next_col, movement_direction ) )
					{
						this_state[ current_linear_index ].rebirth_used = true;
						this_queue.push_back( current_linear_index );
					}

					ObstacleState reset_state;
					TryVisit( next_row, next_col, current_linear_index, next_depth_value, reset_state, this_depth, this_parent, this_state, this_queue, opposite_depth );

					if ( IsConcaveEntry( grid, current_row, current_col, next_row, next_col, movement_direction ) )
					{
						// 凹入口预探索：
						// 一旦判断前方像一个凹口，就把侧向候选也提前加入搜索。
						for ( char scan_dir : kDirectionPriorityOrder )
						{
							if ( scan_dir == movement_direction )
								continue;
							const auto scan_offset = DirectionDelta( scan_dir );
							ObstacleState side_state;
							TryVisit( current_row + scan_offset.first, current_col + scan_offset.second, current_linear_index, next_depth_value, side_state, this_depth, this_parent,
									  this_state, this_queue, opposite_depth );
						}
					}
					return;
				}

				for ( char scan_dir : kDirectionPriorityOrder )
				{
					// 直接撞障后，从当前点往其它方向试探，并进入 crawling 状态。
					// 注意这里不是只试左右，而是按优先级尝试所有非原方向邻居。
					if ( scan_dir == movement_direction )
						continue;
					const auto scan_offset = DirectionDelta( scan_dir );
					ObstacleState side_state;
					side_state.mode = ExplorationMode::Crawling;
					side_state.travel_direction = scan_dir;
					side_state.original_collision_direction = movement_direction;
					side_state.obstacle_hit_count = 1;
					TryVisit( current_row + scan_offset.first, current_col + scan_offset.second, current_linear_index, next_depth_value, side_state, this_depth, this_parent,
							  this_state, this_queue, opposite_depth );
				}
				return;
			}

			bool expanded_any = false;
			const char original_collision_direction = current_state.original_collision_direction == 0 ? movement_direction : current_state.original_collision_direction;
			const auto original_offset = DirectionDelta( original_collision_direction );
			const int original_row = current_row + original_offset.first;
			const int original_col = current_col + original_offset.second;

			if ( IsCellPassable( grid, original_row, original_col ) )
			{
				// 若原先被障碍挡住的方向已经重新可走，则立刻回到自由推进模式。
				ObstacleState reset_state;
				expanded_any = TryVisit( original_row, original_col, current_linear_index, next_depth_value, reset_state, this_depth, this_parent, this_state, this_queue, opposite_depth );
			}
			else if ( IsCellPassable( grid, next_row, next_col ) )
			{
				// 原方向还是不通，但沿着当前贴边方向还能继续前进，就延续 crawling。
				ObstacleState continue_state = current_state;
				continue_state.mode = ExplorationMode::Crawling;
				continue_state.travel_direction = movement_direction;
				continue_state.obstacle_hit_count = current_state.obstacle_hit_count + 1;
				expanded_any = TryVisit( next_row, next_col, current_linear_index, next_depth_value, continue_state, this_depth, this_parent, this_state, this_queue, opposite_depth );
			}
			else
			{
				// 前进也不行时，尝试绕障反向移动，继续保持 crawling。
				const char opposite_original_direction = OppositeDirection( original_collision_direction );
				const auto opposite_original_offset = DirectionDelta( opposite_original_direction );
				ObstacleState reverse_state = current_state;
				reverse_state.mode = ExplorationMode::Crawling;
				reverse_state.travel_direction = opposite_original_direction;
				reverse_state.obstacle_hit_count = current_state.obstacle_hit_count + 1;
				expanded_any = TryVisit( current_row + opposite_original_offset.first, current_col + opposite_original_offset.second, current_linear_index, next_depth_value,
										 reverse_state, this_depth, this_parent, this_state, this_queue, opposite_depth )
					|| expanded_any;
			}

			if ( !expanded_any && options.enable_local_zigzag_expansion && current_state.obstacle_hit_count >= options.zigzag_threshold )
			{
				// 局部 zigzag 扩展：
				// 当连续碰撞次数超过阈值且常规绕障失败时，允许从左右两个侧向做更激进的额外试探。
				// 这一步仍然属于 IBP-B* 的“局部增强”，还不是独立的迷宫补救搜索。
				auto [ left_dir_char, right_dir_char ] = LeftRightDirections( movement_direction );
				for ( char side_dir_char : std::array<char, 2>{ left_dir_char, right_dir_char } )
				{
					const auto side_offset = DirectionDelta( side_dir_char );
					ObstacleState side_state = current_state;
					side_state.mode = ExplorationMode::Crawling;
					side_state.travel_direction = side_dir_char;
					side_state.obstacle_hit_count = current_state.obstacle_hit_count + 1;
					expanded_any = TryVisit( current_row + side_offset.first, current_col + side_offset.second, current_linear_index, next_depth_value, side_state, this_depth, this_parent,
											 this_state, this_queue, opposite_depth )
						|| expanded_any;
				}
			}
		};

		while ( !frontier_queue_from_start.empty() || !frontier_queue_from_goal.empty() )
		{
			// 双向交替扩展。
			// 一旦已经相遇，就只把 meet depth 附近的队列继续 flush 到位。
			if ( meet_linear_index != -1 )
			{
				const bool start_done = !SideShouldContinueFlush( frontier_queue_from_start, depth_from_start, meet_depth_from_start );
				const bool goal_done = !SideShouldContinueFlush( frontier_queue_from_goal, depth_from_goal, meet_depth_from_goal );
				if ( start_done && goal_done )
					break;
			}

			if ( !frontier_queue_from_start.empty() )
			{
				// 当前实现采用交替扩展，而不是一口气清空一整层。
				// 这种写法更接近此项目原先的搜索节奏，也便于保留双向交织感。
				const int current_front_index = frontier_queue_from_start.front();
				frontier_queue_from_start.pop_front();
				ExpandFrontier( current_front_index, goal_pos.row, goal_pos.col, depth_from_start, depth_from_goal, parent_from_start, state_from_start, frontier_queue_from_start );
			}

			if ( !frontier_queue_from_goal.empty() )
			{
				const int current_back_index = frontier_queue_from_goal.front();
				frontier_queue_from_goal.pop_front();
				ExpandFrontier( current_back_index, start_pos.row, start_pos.col, depth_from_goal, depth_from_start, parent_from_goal, state_from_goal, frontier_queue_from_goal );
			}
		}

		outcome.statistics = stats;
		if ( meet_linear_index == -1 )
			return outcome;

		std::vector<int> linear_index_path;
		// 路径重建分两段：
		// 1. 从 meet 点沿起点侧 parent 回溯并反转
		// 2. 再接上终点侧从 meet 之后的 parent 链
		for ( int trace_index = meet_linear_index; trace_index != -1; trace_index = parent_from_start[ trace_index ] )
		{
			linear_index_path.push_back( trace_index );
		}
		std::reverse( linear_index_path.begin(), linear_index_path.end() );
		for ( int trace_index = parent_from_goal[ meet_linear_index ]; trace_index != -1; trace_index = parent_from_goal[ trace_index ] )
		{
			linear_index_path.push_back( trace_index );
		}

		outcome.final_path.reserve( linear_index_path.size() );
		for ( int path_linear_index : linear_index_path )
		{
			outcome.final_path.push_back( FromIndex( path_linear_index ) );
		}

		outcome.meet_position = FromIndex( meet_linear_index );
		outcome.statistics.final_path_length = static_cast<int>( outcome.final_path.size() );
		outcome.success = ValidatePathContiguity( grid, outcome.final_path );
		return outcome;
	}

	SearchOutcome RunZigzagMazeRescue( const Grid& grid, CellPosition start_pos, CellPosition goal_pos )
	{
		// 该函数并不是普通意义上的“再跑一次 BFS”。
		// 它搜索的是“位置 + 方向记忆 + 转向偏好 + 拐角阶段”的扩展状态空间，
		// 专门用于迷宫、窄走廊、连续直角转弯等 IBP-B* 容易失手的场景。
		SearchOutcome outcome;
		if ( !IsGridShapeValid( grid ) )
			return outcome;
		if ( !IsCellInsideGrid( grid, start_pos ) || !IsCellInsideGrid( grid, goal_pos ) )
			return outcome;
		if ( !IsCellPassable( grid, start_pos.row, start_pos.col ) || !IsCellPassable( grid, goal_pos.row, goal_pos.col ) )
			return outcome;

		const int grid_height = static_cast<int>( grid.size() );
		const int grid_width = static_cast<int>( grid[ 0 ].size() );
		const int total_cells = grid_height * grid_width;
		constexpr int kDirectionStateCount = 5;
		constexpr int kBiasStateCount = 2;
		constexpr int kPhaseStateCount = 2;
		const int total_state_count = total_cells * kDirectionStateCount * kBiasStateCount * kPhaseStateCount;

		auto ToIndex = [ & ]( int row, int col ) {
			// 扩展状态图内部仍然复用基础格子的一维索引。
			return ToLinearIndex( row, col, grid_width );
		};
		auto FromIndex = [ & ]( int linear_index ) {
			auto [ row, col ] = FromLinearIndex( linear_index, grid_width );
			return CellPosition { row, col };
		};

		std::vector<int> depth_by_state( total_state_count, -1 );
		std::vector<int> parent_state( total_state_count, -1 );
		std::deque<int>	search_queue;
		const int		start_linear_index = ToIndex( start_pos.row, start_pos.col );
		const int		goal_linear_index = ToIndex( goal_pos.row, goal_pos.col );

		for ( std::uint8_t bias_index = 0; bias_index < 2; ++bias_index )
		{
			// 起点同时以“左偏好”和“右偏好”两个版本入队，
			// 让搜索在一开始就并行考虑两种绕弯风格。
			const int start_state_id = EncodeZigzagStateId( start_linear_index, 4, bias_index, 0 );
			depth_by_state[ start_state_id ] = 0;
			search_queue.push_back( start_state_id );
		}

		int goal_state_id = -1;
		while ( !search_queue.empty() )
		{
			// 这里仍然按 BFS 层次搜索扩展状态图，
			// 只是每个状态的邻接扩展顺序受方向记忆和拐角阶段影响。
			const int current_state_id = search_queue.front();
			search_queue.pop_front();
			outcome.statistics.expanded_node_count++;

			const ZigzagStateNode current_state = DecodeZigzagStateId( current_state_id );
			if ( current_state.cell_linear_index == goal_linear_index )
			{
				goal_state_id = current_state_id;
				break;
			}

			const CellPosition	current_pos = FromIndex( current_state.cell_linear_index );
			const char			last_direction = IndexToDirection( current_state.last_direction_index );
			const ZigzagTurnBias turn_bias = current_state.turn_bias_index == 0 ? ZigzagTurnBias::PreferLeft : ZigzagTurnBias::PreferRight;
			const ZigzagPhase	phase = current_state.phase_index == 0 ? ZigzagPhase::Normal : ZigzagPhase::AfterCorner;
			auto ordered_directions = BuildZigzagStateDirectionPriority( last_direction, turn_bias, phase, current_pos.row, current_pos.col, goal_pos.row, goal_pos.col );

			for ( char direction_char : ordered_directions )
			{
				// 有序扩展的意义不在于改变 BFS 的最短层数性质，
				// 而在于当存在多个同层备选状态时，让更像“zigzag/winding”的路线先进入搜索前沿。
				if ( direction_char == 0 )
					continue;
				const auto [ row_offset, col_offset ] = DirectionDelta( direction_char );
				const int next_row = current_pos.row + row_offset;
				const int next_col = current_pos.col + col_offset;
				if ( !IsCellPassable( grid, next_row, next_col ) )
					continue;

				std::uint8_t next_last_direction_index = static_cast<std::uint8_t>( DirectionToIndex( direction_char ) );
				std::uint8_t next_turn_bias_index = current_state.turn_bias_index;
				std::uint8_t next_phase_index = static_cast<std::uint8_t>( ZigzagPhase::Normal );
				if ( last_direction != 0 )
				{
					// 一旦发生真正的拐弯，就切换 phase，并翻转 turn bias，
					// 使得后续路径具有“之”字形 / winding 倾向。
					const char reverse_direction = OppositeDirection( last_direction );
					if ( direction_char != last_direction && direction_char != reverse_direction )
					{
						next_phase_index = static_cast<std::uint8_t>( ZigzagPhase::AfterCorner );
						next_turn_bias_index = 1u - current_state.turn_bias_index;
					}
				}

				const int next_linear_index = ToIndex( next_row, next_col );
				const int next_state_id = EncodeZigzagStateId( next_linear_index, next_last_direction_index, next_turn_bias_index, next_phase_index );
				if ( depth_by_state[ next_state_id ] != -1 )
					continue;

				depth_by_state[ next_state_id ] = depth_by_state[ current_state_id ] + 1;
				parent_state[ next_state_id ] = current_state_id;
				search_queue.push_back( next_state_id );
			}
		}

		if ( goal_state_id == -1 )
			return outcome;

		std::vector<int> state_trace;
		// 注意：state_trace 中可能出现“同一格子但状态不同”的连续节点，
		// 所以最终导出路径时还要去掉重复格子。
		for ( int trace_state_id = goal_state_id; trace_state_id != -1; trace_state_id = parent_state[ trace_state_id ] )
		{
			state_trace.push_back( trace_state_id );
		}
		std::reverse( state_trace.begin(), state_trace.end() );

		int previous_cell_linear_index = -1;
		for ( int trace_state_id : state_trace )
		{
			const ZigzagStateNode trace_node = DecodeZigzagStateId( trace_state_id );
			// 同一格子可能以不同“方向状态”重复出现，这里只保留格子路径本身。
			if ( trace_node.cell_linear_index == previous_cell_linear_index )
				continue;
			outcome.final_path.push_back( FromIndex( trace_node.cell_linear_index ) );
			previous_cell_linear_index = trace_node.cell_linear_index;
		}

		outcome.statistics.final_path_length = static_cast<int>( outcome.final_path.size() );
		outcome.meet_position = goal_pos;
		outcome.success = !outcome.final_path.empty() && ValidatePathContiguity( grid, outcome.final_path );
		return outcome;
	}

	SearchOutcome RunIbpBStarZigzagEnhanced( const Grid& grid, CellPosition start_pos, CellPosition goal_pos, int wait_layers, const AlgorithmOptions& options )
	{
		// 对外推荐的增强入口：
		// 优先使用更贴近论文风格的主算法，仅在失败或路径无效时再启用救援模式。
		SearchOutcome primary_outcome = RunIbpBStar( grid, start_pos, goal_pos, wait_layers, options );
		if ( primary_outcome.success && ValidatePathContiguity( grid, primary_outcome.final_path ) )
			return primary_outcome;

		SearchOutcome rescue_outcome = RunZigzagMazeRescue( grid, start_pos, goal_pos );
		if ( rescue_outcome.success )
			return rescue_outcome;
		return primary_outcome;
	}
}  // namespace IBP_BStarAlgorithm
