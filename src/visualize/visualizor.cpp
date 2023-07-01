#include "visualizor.h"
#include "log_report.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "thread"

namespace SLAM_UTILITY {


Visualizor::Visualizor() {
    // Initialize resource for glfw.
    glfwInit();

    // Config the version and mode of opengl we used.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
}

Visualizor::~Visualizor() {
    // Delete resources of glfw.
    glfwTerminate();
}

bool Visualizor::ShowImage(const std::string &window_title, const Image &image) {
    if (image.data() == nullptr || image.rows() < 1 || image.cols() < 1) {
        ReportWarn("[Visualizor] Image to be shown is invalid.");
        return false;
    }

    // Create new window.
    GLFWwindow *window = glfwCreateWindow(image.cols(), image.rows(), window_title.data(), nullptr, nullptr);
    if (window == nullptr) {
        ReportError("[Visualizor] Failed to create new window.");
        return false;
    }

    // Focus on this window. Then it can be operated.
    glfwMakeContextCurrent(window);

    // Refresh window util closed.
    while (!glfwWindowShouldClose(window)) {
        // Poll and handle events.
        glfwPollEvents();

        // If any key is pressed, this window should be closed.
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GL_TRUE);
        }

        // Config the color of background. Then set it to be background.
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Refresh buffers.
        glfwSwapBuffers(window);
    }

    // Destory this window.
    glfwDestroyWindow(window);
    return true;
}
}
