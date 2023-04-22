cmake_minimum_required( VERSION 3.10 )

project( slam_utility )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add opencv.
find_package( OpenCV REQUIRED )

# Add slam utility.
if ( NOT TARGET lib_slam_utility )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_slam_utility )
endif()

add_executable( test_slam_utility
    test/test_slam_utility.cpp
)

target_link_libraries( test_slam_utility
    ${OpenCV_LIBS}
    lib_slam_utility
)
