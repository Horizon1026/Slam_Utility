aux_source_directory( . AUX_SRC_LINE_SEGMENT )

# Add all relative components in slam utility.
if ( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ../../basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if ( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ../../math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ../../operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ../../log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()

add_library( lib_line_segment ${AUX_SRC_LINE_SEGMENT} )
target_include_directories( lib_line_segment PUBLIC . )
target_link_libraries( lib_line_segment
    lib_slam_utility_basic_type
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
)
