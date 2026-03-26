#pragma once

#include "pathfinding_types.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>

namespace pathfinding
{
	// 控制台渲染样式属于“应用壳层”能力，不参与算法计算。
	// 把它独立在地图模块里，可以让算法核心接口保持干净。
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

	// 一张完整的测试地图，既包含网格，也包含起点和终点。
	// 用这个结构传参，比散落的三元组更不容易出错。
	struct MapInstance
	{
		Grid	 grid;
		Position start;
		Position goal;
	};

	bool		IsGridShapeValid( const Grid& grid );
	bool		IsInsideGrid( const Grid& grid, const Position& pos );
	bool		IsPassable( const Grid& grid, int row, int col );
	bool		ValidatePathContiguity( const Grid& grid, const std::vector<Position>& path );
	MapInstance GenerateRandomMap( int width, int height, double wall_probability, std::uint32_t seed );
	MapInstance GenerateStaggeredWallMap( int width, int height, double barrier_density, std::uint32_t seed );
	MapInstance LoadMapFromFile( const std::string& file_path, const RenderStyle& render_style = {} );
	MapInstance GeneratePerfectMaze( int width, int height, std::uint32_t seed );
	void		RenderGridWithPath( const Grid& grid, const std::vector<Position>& path, Position start, Position goal, const RenderStyle& render_style );
}  // namespace pathfinding
