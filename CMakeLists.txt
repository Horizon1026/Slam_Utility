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

# Add 3rd libraries.
# if( NOT TARGET lib_pytorch_for_cxx )
#     add_subdirectory( 3rd_libraries/libtorch ${PROJECT_SOURCE_DIR}/build/lib_pytorch_for_cxx )
# endif()
if( NOT TARGET lib_onnx_run_time_for_cxx )
    add_subdirectory( 3rd_libraries/onnx ${PROJECT_SOURCE_DIR}/build/lib_onnx_run_time_for_cxx )
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

# Compile all targets without 3rd_libraries.
set( UNIT_TEST_OF_SLAM_UTILITY_LIST
    test_datatype_image
    test_covisible_graph
    test_object_pool
    test_circular_buffer
    test_backward
    test_basic_math
    test_kd_tree
    test_3d_gaussian
    test_line_segment
    test_plane
    test_yaml_parser
    test_probability_distribution
)
foreach( unit_test_src ${UNIT_TEST_OF_SLAM_UTILITY_LIST} )
    add_executable( ${unit_test_src}
        test/${unit_test_src}.cpp
    )
    target_link_libraries( ${unit_test_src}
        lib_slam_utility
        lib_2d_visualizor
        lib_3d_visualizor
    )
endforeach()

# Compile all targets with 3rd_libraries.
add_executable( test_onnx
    test/test_onnx.cpp
)
target_link_libraries( test_onnx
    lib_slam_utility
    lib_2d_visualizor
    lib_3d_visualizor
    lib_onnx_run_time_for_cxx
)
