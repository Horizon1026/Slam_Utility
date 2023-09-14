aux_source_directory( . AUX_SRC_BACKWARD )

# Try to find one of the dependence.
set( DEPENDENCE_LIBRARIES )

# Try to find library dw.
if( "${DEPENDENCE_LIBRARIES}" STREQUAL "" )
    find_library( Dw NAMES dw )
    if( ${Dw} STREQUAL "Dw-NOTFOUND" )
        message( ">> Cannot find dw." )
    else()
        message( ">> Find dw : ${Dw}" )
        set( DEPENDENCE_LIBRARIES -ldw)
        add_definitions( -DBACKWARD_HAS_DW=1 )
    endif()
endif()

# Try to find library bfd.
if( "${DEPENDENCE_LIBRARIES}" STREQUAL "" )
    find_library( Bfd NAMES bfd )
    if( ${Bfd} STREQUAL "Bfd-NOTFOUND" )
        message( ">> Cannot find bfd." )
    else()
        message( ">> Find bfd : ${Bfd}" )
        set( DEPENDENCE_LIBRARIES -lbfd)
        add_definitions( -DBACKWARD_HAS_BFD=1 )
    endif()
endif()

# Try to find library dwarf.
if( "${DEPENDENCE_LIBRARIES}" STREQUAL "" )
    find_library( Dwarf NAMES dwarf )
    if( ${Dwarf} STREQUAL "Dwarf-NOTFOUND" )
        message( ">> Cannot find dwarf." )
    else()
        message( ">> Find dwarf : ${Dwarf}" )
        set( DEPENDENCE_LIBRARIES -ldwarf)
        add_definitions( -DBACKWARD_HAS_DWARF=1 )
    endif()
endif()

# Try to find library unwind.
if( "${DEPENDENCE_LIBRARIES}" STREQUAL "" )
    find_library( Unwind NAMES unwind )
    if( ${Unwind} STREQUAL "Unwind-NOTFOUND" )
        message( ">> Cannot find unwind." )
    else()
        message( ">> Find unwind : ${Unwind}" )
        set( DEPENDENCE_LIBRARIES -lunwind)
        add_definitions( -DBACKWARD_HAS_LIBUNWIND=1 )
    endif()
endif()

# Print message of dependence.
if( "${DEPENDENCE_LIBRARIES}" STREQUAL "" )
    message( ">> Backward has no dependence. It will not be actived." )
else()
    message( ">> Backward will depend on ${DEPENDENCE_LIBRARIES}." )
endif()

# Create library of stack backward.
add_library( lib_stack_backward ${AUX_SRC_BACKWARD} )
target_include_directories( lib_stack_backward PUBLIC . )
target_link_libraries( lib_stack_backward
    ${DEPENDENCE_LIBRARIES}
)
