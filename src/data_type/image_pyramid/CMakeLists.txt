aux_source_directory( . AUX_SRC_IMAGE_PYRAMID )

# Add all relative components in datatype.
if ( NOT TARGET lib_image )
    add_subdirectory( ../image ${PROJECT_SOURCE_DIR}/build/lib_image )
endif()

# Add all relative components in slam utility.
if ( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ../../basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ../../memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()

add_library( lib_image_pyramid ${AUX_SRC_IMAGE_PYRAMID} )
target_include_directories( lib_image_pyramid PUBLIC . )
target_link_libraries( lib_image_pyramid
    lib_image

    lib_slam_utility_basic_type
    lib_slam_utility_memory
)
