aux_source_directory( . AUX_SRC_BACKWARD )

# Try to find one of the dependence.
if( NOT DEFINED FIND_DEPENDENCE_FOR_BACKWARD )
find_library( Unwind )
endif()

add_library( lib_stack_backward ${AUX_SRC_BACKWARD} )
target_include_directories( lib_stack_backward PUBLIC . )
target_link_libraries( lib_stack_backward )
