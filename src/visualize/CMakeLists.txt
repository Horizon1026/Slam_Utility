aux_source_directory( . AUX_SRC_SLAM_UTILITY_VISUALIZE )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../datatype ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_datatype )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ../log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ../memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if ( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ../operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()

if ( NOT TARGET lib_visualize_base_glad_khr )
    add_subdirectory( ../visualize/glad+khr ${PROJECT_SOURCE_DIR}/build/lib_visualize_base_glad_khr )
endif()
if ( NOT TARGET lib_visualize_base_fonts )
    add_subdirectory( ../visualize/fonts ${PROJECT_SOURCE_DIR}/build/lib_visualize_base_fonts )
endif()
if ( NOT TARGET lib_visualize_base_stb )
    add_subdirectory( ../visualize/stb_image ${PROJECT_SOURCE_DIR}/build/lib_visualize_base_stb )
endif()

add_library( lib_slam_utility_visualize ${AUX_SRC_SLAM_UTILITY_VISUALIZE} )

target_include_directories( lib_slam_utility_visualize PUBLIC
    .
    fonts
    glad+khr
    stb_image
)
target_link_libraries( lib_slam_utility_visualize
    lib_slam_utility_datatype
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_operate

    lib_visualize_base_glad_khr
    lib_visualize_base_fonts
    lib_visualize_base_stb
)

# Check compile environment. If environment is windows, add additional dependence.
if ( "${COMPILE_ENV}" STREQUAL "windows" )
    target_include_directories( lib_slam_utility_visualize PUBLIC
        "E:/glfw-3.3.8.bin.WIN64/include"
    )
    target_link_libraries( lib_slam_utility_visualize
        "E:/glfw-3.3.8.bin.WIN64/lib-mingw-w64/glfw3.dll"
    )
endif()
