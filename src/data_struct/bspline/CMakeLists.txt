aux_source_directory( . AUX_SRC_BSPLINE )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()

add_library( lib_bspline ${AUX_SRC_BSPLINE} )
target_include_directories( lib_bspline PUBLIC . )
target_link_libraries( lib_bspline
    lib_slam_utility_basic_type
)
