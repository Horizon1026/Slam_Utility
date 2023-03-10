cmake_minimum_required( VERSION 3.10 )

project( slam_utility )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )

find_package( Eigen3 REQUIRED )

aux_source_directory( ./test SRC_TEST )

add_subdirectory( ./src )

add_executable( test_slam_utility ${SRC_TEST} )

target_link_libraries( test_slam_utility
    lib_slam_utility_datatype
    lib_slam_utility_log
    lib_slam_utility_math
    lib_slam_utility_memory
    lib_slam_utility_operate
)

target_include_directories( test_slam_utility PUBLIC ./src )
