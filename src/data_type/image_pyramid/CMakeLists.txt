aux_source_directory( . AUX_SRC_IMAGE_PYRAMID )

# Add all relative components in datatype.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if( NOT TARGET lib_image )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/image ${PROJECT_SOURCE_DIR}/build/lib_image )
endif()

# Add all relative components in slam utility.
if( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()

add_library( lib_image_pyramid ${AUX_SRC_IMAGE_PYRAMID} )
target_include_directories( lib_image_pyramid PUBLIC . )
target_link_libraries( lib_image_pyramid
    lib_image

    lib_slam_utility_basic_type
    lib_slam_utility_memory
)
