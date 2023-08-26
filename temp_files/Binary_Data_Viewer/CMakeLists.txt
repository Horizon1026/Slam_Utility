cmake_minimum_required( VERSION 3.10 )

project( binary_data_viewer )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -lpthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add all 3rd libraries.
if ( NOT TARGET lib_imgui_glfw_opengl )
    add_subdirectory( 3rd_libraries/imgui ${PROJECT_SOURCE_DIR}/build/lib_imgui_glfw_opengl )
endif()

# Add all components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility/src )
if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ${SLAM_UTILITY_PATH}/datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()

# Create executable target to test viewer.
add_executable( test_viewer
    test/test_viewer.cpp
)
target_link_libraries( test_viewer
    lib_slam_utility_datatype
    lib_slam_utility_operate
    lib_slam_utility_log

    lib_imgui_glfw_opengl
)
