cmake_minimum_required( VERSION 3.10 )

project( slam_utility )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )

aux_source_directory( ./test SRC_TEST )
add_executable( test_slam_utility ${SRC_TEST} )

# Add slam utility lib.
add_subdirectory( ./src )
target_link_libraries( test_slam_utility lib_slam_utility )
