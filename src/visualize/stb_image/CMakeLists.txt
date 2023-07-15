aux_source_directory( . AUX_SRC_VISUALIZE_BASE_STB )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../../datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()

add_library( lib_visualize_base_stb ${AUX_SRC_VISUALIZE_BASE_STB} )
target_include_directories( lib_visualize_base_stb PUBLIC . )
target_link_libraries( lib_visualize_base_stb
    lib_slam_utility_datatype
)
