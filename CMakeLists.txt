cmake_minimum_required(VERSION 3.16)
project(IBP_BStar_Pathfinding LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
	add_compile_options(/W4 /Zc:__cplusplus /utf-8 /std:c++17)
else()
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11")
			message(FATAL_ERROR "GNU CXX compiler version is too small !")
		endif()
		if(CMAKE_BUILD_TYPE STREQUAL "Release")
			add_compile_options(-Wall -O2 -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8)
		else()
			add_compile_options(-g -O0 -Wall -Wextra -Wpedantic -fsigned-char -finput-charset=UTF-8 -fexec-charset=UTF-8)
		endif()
	endif()
endif()

add_library(ibp_bstar_core_lib
	src/ibp_bstar_core.cpp
)
target_include_directories(ibp_bstar_core_lib PUBLIC include)

add_library(pathfinding_framework
	src/pathfinding_grid.cpp
	src/pathfinding_algorithms.cpp
	src/pathfinding_compare.cpp
)
target_include_directories(pathfinding_framework PUBLIC include)
target_link_libraries(pathfinding_framework PUBLIC ibp_bstar_core_lib)

add_library(ibp_bstar_app_support
	src/ibp_bstar_app.cpp
)
target_include_directories(ibp_bstar_app_support PUBLIC include)
target_link_libraries(ibp_bstar_app_support PUBLIC ibp_bstar_core_lib)

add_executable(pathfinding_compare
	apps/pathfinding_compare_main.cpp
)
target_link_libraries(pathfinding_compare PRIVATE pathfinding_framework)

add_executable(ibp_bstar_zigzag_mode
	apps/ibp_bstar_zigzag_demo_main.cpp
)
target_link_libraries(ibp_bstar_zigzag_mode PRIVATE ibp_bstar_app_support)
