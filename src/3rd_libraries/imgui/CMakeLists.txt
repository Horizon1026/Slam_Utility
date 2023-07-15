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

# Check compile environment.
if ( "${COMPILE_ENV}" STREQUAL "windows" )
    # If environment is windows, add special dependence.
    target_include_directories( lib_imgui_glfw_opengl PUBLIC
        "E:/glfw-3.3.8.bin.WIN64/include"
    )
    target_link_libraries( lib_imgui_glfw_opengl
        "E:/glfw-3.3.8.bin.WIN64/lib-mingw-w64/glfw3.dll"
    )
else()
    # If environment is windows, add special dependence.
    target_link_libraries( lib_imgui_glfw_opengl
        -lglfw
        -lGL
        -ldl
    )
endif()
