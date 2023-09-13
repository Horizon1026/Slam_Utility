aux_source_directory( . AUX_SRC_BACKWARD )
add_library( lib_stack_backward ${AUX_SRC_BACKWARD} )
target_include_directories( lib_stack_backward PUBLIC . )
target_link_libraries( lib_stack_backward )
