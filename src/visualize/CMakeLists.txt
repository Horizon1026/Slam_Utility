aux_source_directory( . AUX_SRC_SLAM_UTILITY_VISUALIZE )

if ( NOT TARGET lib_slam_utility_datatype )
    add_subdirectory( ../datatype ${PROJECT_SOURCE_DIR}/build )
endif()
if ( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ../log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if ( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ../memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if ( NOT TARGET lib_imgui_glfw_opengl )
    add_subdirectory( ../visualize/imgui ${PROJECT_SOURCE_DIR}/build/lib_imgui_glfw_opengl )
endif()

# Add opencv.
find_package( OpenCV REQUIRED )

add_library( lib_slam_utility_visualize ${AUX_SRC_SLAM_UTILITY_VISUALIZE} )
target_include_directories( lib_slam_utility_visualize PUBLIC . )
target_link_libraries( lib_slam_utility_visualize
    lib_slam_utility_datatype
    lib_slam_utility_log
    lib_slam_utility_memory

    lib_imgui_glfw_opengl

    ${OpenCV_LIBS}
)
