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

# Create executable target to test datatype image and pyramid.
add_executable( test_datatype_image
    test/test_datatype_image.cpp
)
target_link_libraries( test_datatype_image
    ${OpenCV_LIBS}
    lib_slam_utility
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

# Create executable target to test visualizor.
add_executable( test_visualizor
    test/test_visualizor.cpp
)
target_link_libraries( test_visualizor
    ${OpenCV_LIBS}
    lib_slam_utility
)
