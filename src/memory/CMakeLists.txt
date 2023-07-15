aux_source_directory( . AUX_SRC_SLAM_UTILITY_MEMORY )

if ( NOT TARGET lib_slam_utility_datatype_basic )
    add_subdirectory( ../datatype/basic ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype_basic )
endif()

add_library( lib_slam_utility_memory ${AUX_SRC_SLAM_UTILITY_MEMORY} )
target_include_directories( lib_slam_utility_memory PUBLIC . )
target_link_libraries( lib_slam_utility_memory
    lib_slam_utility_datatype_basic
)
