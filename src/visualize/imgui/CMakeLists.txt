aux_source_directory( . AUX_SRC_IMGUI )
aux_source_directory( ./backend AUX_SRC_IMGUI_BACKEND )

add_library( lib_imgui_glfw_opengl
    ${AUX_SRC_IMGUI}
    ${AUX_SRC_IMGUI_BACKEND}
)

target_include_directories( lib_imgui_glfw_opengl PUBLIC
    .
    ./backend
)

target_link_libraries( lib_imgui_glfw_opengl
    -lglfw
    -lGL
)
