aux_source_directory( . AUX_SRC_IMAGE )

if ( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ../../basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ../../memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()

add_library( lib_image ${AUX_SRC_IMAGE} )
target_include_directories( lib_image PUBLIC . )
target_link_libraries( lib_image
    lib_slam_utility_basic_type
    lib_slam_utility_memory
)
