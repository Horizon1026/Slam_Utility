aux_source_directory( . AUX_SRC_VISUALIZE_BASE_FONTS )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../../datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()

add_library( lib_visualize_base_fonts ${AUX_SRC_VISUALIZE_BASE_FONTS} )
target_include_directories( lib_visualize_base_fonts PUBLIC . )
target_link_libraries( lib_visualize_base_fonts
    lib_slam_utility_datatype
)
