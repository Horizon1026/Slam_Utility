aux_source_directory( . AUX_SRC_SLAM_UTILITY_VISUALIZE )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../datatype ${PROJECT_SOURCE_DIR}/build )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()

add_library( lib_slam_utility_visualize ${AUX_SRC_SLAM_UTILITY_VISUALIZE} )
target_include_directories( lib_slam_utility_visualize PUBLIC . )
target_link_libraries( lib_slam_utility_visualize
    lib_slam_utility_datatype
    lib_slam_utility_log
)
