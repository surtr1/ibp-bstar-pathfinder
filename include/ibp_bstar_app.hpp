#pragma once

#include "ibp_bstar_core.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>

namespace IBP_BStarAlgorithm
{
	// 渲染样式完全属于应用层，而不是算法层。
	// 这样后续做 benchmark 时，可以直接复用算法核心，不必带上终端字符输出逻辑。
	struct RenderStyle
	{
		bool							  use_ascii_glyphs = true;
		bool							  print_arrows = false;
		std::string						  wall_unicode = u8"█";
		std::string						  empty_unicode = u8"·";
		std::string						  path_unicode = u8"○";
		std::string						  start_unicode = "S";
		std::string						  goal_unicode = "E";
		std::unordered_map<char, std::string> arrows_unicode = { { 'U', u8"↑" }, { 'D', u8"↓" }, { 'L', u8"←" }, { 'R', u8"→" } };
		std::string						  wall_ascii = "+";
		std::string						  empty_ascii = ".";
		std::string						  path_ascii = "*";
		std::string						  start_ascii = "S";
		std::string						  goal_ascii = "E";
		std::unordered_map<char, std::string> arrows_ascii = { { 'U', "^" }, { 'D', "v" }, { 'L', "<" }, { 'R', ">" } };
	};

	// 可执行程序层面的配置：
	// 包括地图来源、打印行为、demo 开关，以及要传入算法核心的算法参数。
	struct RunConfig
	{
		bool						use_random_map = true;
		int							random_map_width = 96;
		int							random_map_height = 96;
		double						random_wall_probability = 0.18;
		std::uint32_t				random_seed = 41548941u;
		std::string					map_file_path;
		std::optional<CellPosition> cli_start_override;
		std::optional<CellPosition> cli_goal_override;
		bool						reroll_until_solvable = true;
		int							reroll_max_attempts = 100;
		int							flush_wait_layers = 2;
		bool						print_path = true;
		bool						print_stats = true;
		bool						print_invalid_path = false;
		bool						run_maze_demo = false;
		RenderStyle					render_style;
		AlgorithmOptions			algorithm_options { true, 2 };
	};

	// 地图读写、随机图/迷宫生成等辅助能力，供可执行程序和未来的对比框架复用。
	Grid GenerateRandomGrid( int width, int height, double wall_probability, std::uint32_t seed, CellPosition& out_start, CellPosition& out_goal );
	std::tuple<Grid, CellPosition, CellPosition> LoadGridFromFile( const std::string& file_path, const RenderStyle& render_style = {} );
	Grid GeneratePerfectMazeGrid( int width, int height, std::uint32_t seed, CellPosition& out_start, CellPosition& out_goal );

	// 控制台可视化与命令行解析都放在 app 层，避免污染核心算法接口。
	void RenderGridWithPath( const Grid& grid, const std::vector<CellPosition>& path, CellPosition start_pos, CellPosition goal_pos, const RenderStyle& render_style );
	void ParseArgs( int argc, char** argv, RunConfig& config );

	// 简单 BFS 基线，主要用于 maze demo 和算法对比时给出一个最短路参考值。
	int	 ShortestPathLength_BFS( const Grid& grid, CellPosition start, CellPosition goal );
}  // namespace IBP_BStarAlgorithm
