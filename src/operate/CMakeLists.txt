aux_source_directory( . AUX_SRC_SLAM_UTILITY_OPERATE )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()

add_library( lib_slam_utility_operate ${AUX_SRC_SLAM_UTILITY_OPERATE} )
target_include_directories( lib_slam_utility_operate PUBLIC . )
target_link_libraries( lib_slam_utility_operate
    lib_slam_utility_datatype
)
