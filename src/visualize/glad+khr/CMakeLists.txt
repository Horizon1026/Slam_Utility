aux_source_directory( . AUX_SRC_VISUALIZE_BASE_GLAD_KHR )
add_library( lib_visualize_base_glad_khr ${AUX_SRC_VISUALIZE_BASE_GLAD_KHR} )
target_include_directories( lib_visualize_base_glad_khr PUBLIC . )
target_link_libraries( lib_visualize_base_glad_khr )
