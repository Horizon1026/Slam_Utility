cmake_minimum_required( VERSION 3.10 )

project( slam_utility )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -g -Wall -pthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 20 )

# Add slam utility.
if( NOT TARGET lib_slam_utility )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_slam_utility )
endif()

# Add visualizor.
set( VISUALIZOR_2D_PATH ${PROJECT_SOURCE_DIR}/../Visualizor2D )
if( NOT TARGET lib_2d_visualizor )
    add_subdirectory( ${VISUALIZOR_2D_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_2d_visualizor )
endif()
set( VISUALIZOR_3D_PATH ${PROJECT_SOURCE_DIR}/../Visualizor3D )
if( NOT TARGET lib_3d_visualizor )
    add_subdirectory( ${VISUALIZOR_3D_PATH}/src ${PROJECT_SOURCE_DIR}/build/lib_3d_visualizor )
endif()

# Create executable target to test datatype image and pyramid.
add_executable( test_datatype_image
    test/test_datatype_image.cpp
)
target_link_libraries( test_datatype_image
    lib_slam_utility
    lib_2d_visualizor
)

# Create executable target to test covisible graph.
add_executable( test_covisible_graph
    test/test_covisible_graph.cpp
)
target_link_libraries( test_covisible_graph
    lib_slam_utility
)

# Create executable target to test object pool.
add_executable( test_object_pool
    test/test_object_pool.cpp
)
target_link_libraries( test_object_pool
    lib_slam_utility
)

# Create executable target to test circular buffer.
add_executable( test_circular_buffer
    test/test_circular_buffer.cpp
)
target_link_libraries( test_circular_buffer
    lib_slam_utility
)

# Create executable target to test stack trace back.
add_executable( test_backward
    test/test_backward.cpp
)
target_link_libraries( test_backward
    lib_slam_utility
)

# Create executable target to test basic math.
add_executable( test_basic_math
    test/test_basic_math.cpp
)
target_link_libraries( test_basic_math
    lib_slam_utility
)

# Create executable target to test kd tree.
add_executable( test_kd_tree
    test/test_kd_tree.cpp
)
target_link_libraries( test_kd_tree
    lib_slam_utility
    lib_3d_visualizor
)

# Create executable target to test 3d_gaussian.
add_executable( test_3d_gaussian
    test/test_3d_gaussian.cpp
)
target_link_libraries( test_3d_gaussian
    lib_slam_utility
    lib_2d_visualizor
)

# Create executable target to test line_segment.
add_executable( test_line_segment
    test/test_line_segment.cpp
)
target_link_libraries( test_line_segment
    lib_slam_utility
)

# Create executable target to test plane.
add_executable( test_plane
    test/test_plane.cpp
)
target_link_libraries( test_plane
    lib_slam_utility
)
