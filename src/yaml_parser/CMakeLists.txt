aux_source_directory( . AUX_SRC_SLAM_UTILITY_YAML_PARSER )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()

# Package lib slam_utility_yaml_parser.
add_library( lib_slam_utility_yaml_parser ${AUX_SRC_SLAM_UTILITY_YAML_PARSER} )
target_include_directories( lib_slam_utility_yaml_parser PUBLIC . )
target_link_libraries( lib_slam_utility_yaml_parser
    lib_slam_utility_log
)
