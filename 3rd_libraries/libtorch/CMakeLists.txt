# Find libtorch.
# list( APPEND CMAKE_PREFIX_PATH "/home/horizon_msi_wsl2/dependence/libtorch_linux_cpu_2.5.1" )
list( APPEND CMAKE_PREFIX_PATH "/home/horizon_msi_wsl2/dependence/libtorch_linux_cuda12.4_2.5.1" )
find_package( PythonInterp REQUIRED )
find_package( Torch REQUIRED )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if( NOT TARGET lib_slam_utility_tick_tock )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/tick_tock ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_tick_tock )
endif()

# Add all relative components of slam utility data type.
if( NOT TARGET lib_image )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/image ${PROJECT_SOURCE_DIR}/build/lib_image )
endif()
if( NOT TARGET lib_image_pyramid )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/image_pyramid ${PROJECT_SOURCE_DIR}/build/lib_image_pyramid )
endif()
if( NOT TARGET lib_3d_gaussian )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/3d_gaussian ${PROJECT_SOURCE_DIR}/build/lib_3d_gaussian )
endif()
if( NOT TARGET lib_line_segment )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/line_segment ${PROJECT_SOURCE_DIR}/build/lib_line_segment )
endif()

# Package target lib_pytorch_for_cxx.
aux_source_directory( . AUX_SRC_LIB_TORCH_FOR_CXX )
add_library( lib_pytorch_for_cxx
    ${AUX_SRC_LIB_TORCH_FOR_CXX}
)
target_link_libraries( lib_pytorch_for_cxx
    ${TORCH_LIBRARIES}

    lib_slam_utility_basic_type
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_tick_tock

    lib_image
    lib_image_pyramid
    lib_3d_gaussian
    lib_line_segment
)
target_include_directories( lib_pytorch_for_cxx PUBLIC
    .
)
