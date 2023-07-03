#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"

#include "thread"

#include "opencv2/opencv.hpp"

namespace SLAM_UTILITY {

Visualizor &Visualizor::GetInstance() {
    static Visualizor instance;
    return instance;
}

void Visualizor::GlfwErrorCallback(int error, const char *description) {
    ReportError("[Visualizor] GLFW error " << error << " : " << description);
}

Visualizor::Visualizor() {
    glfwSetErrorCallback(&GlfwErrorCallback);

    // Initialize resource for glfw.
    if (!glfwInit()) {
        ReportError("[Visualizor] GLFW init failed.");
        return;
    }

    // Config the version and mode of opengl we used.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    main_window_ = CreateNewWindow(1280, 720, "SLAM Visualizor");
    if (main_window_ == nullptr) {
        ReportError("[Visualizor] GLFW cannot create new window.");
        return;
    }
    InitializeImgui(main_window_);
}

Visualizor::~Visualizor() {
    // Clean up settings of ImGui.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // Delete resources of glfw.
    glfwDestroyWindow(main_window_);
    glfwTerminate();
}

bool Visualizor::ShowImage(const std::string &window_title, const Image &image) {
    if (image.data() == nullptr || image.cols() < 1 || image.rows() < 1) {
        return false;
    }

    auto item = image_objects_.find(window_title);
    if (item == image_objects_.end()) {
        Texture texture;
        texture.rows = image.rows();
        texture.cols = image.cols();
        ConvertImageToTexture(image, texture);
        image_objects_.insert(std::make_pair(window_title, texture));
    } else {
        Texture &texture = item->second;
        texture.rows = image.rows();
        texture.cols = image.cols();
        ConvertImageToTexture(image, texture);
    }

    return true;
}

GLFWwindow *Visualizor::CreateNewWindow(int width, int height, const char *title) {
    GLFWwindow *window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (window == nullptr) {
        return nullptr;
    }

    // Focus on this window. Then it can be operated.
    glfwMakeContextCurrent(window);
    // Enable vsync.
    glfwSwapInterval(1);

    return window;
}

void Visualizor::InitializeImgui(GLFWwindow *window, const char *glsl_version) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void Visualizor::RenderMainWindow(GLFWwindow *window) {
    while (!glfwWindowShouldClose(window)) {
        // Poll and handle events.
        glfwPollEvents();

        // Start the Dear ImGui frame.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Refresh all components of main window.
        RefreshMainWindow(window);

        // Rendering the frame.
        ImGui::Render();

        // Config the affine transform from NDC to screen.
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        // Config the color of background. Then set it to be background.
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Refresh buffers.
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        ProcessKeyboardMessage(window);
    }
}

void Visualizor::RefreshMainWindow(GLFWwindow *window) {
    for (auto &item : image_objects_) {
        ImGui::Begin(item.first.data(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Image(item.second.id, ImVec2(item.second.cols, item.second.rows), ImVec2(0, 0));
        ImGui::End();
    }
}

void Visualizor::ProcessKeyboardMessage(GLFWwindow *window) {
    // If any key is pressed, this window should be closed.
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

void Visualizor::ConvertUint8ToRGBA(const uint8_t *gray, uint8_t *rgba, int32_t gray_size) {
    for (int32_t i = 0; i < gray_size; ++i) {
        const int32_t idx = i * 3;
        std::fill_n(rgba + idx, 3, gray[i]);
    }
}

void Visualizor::ConvertImageToTexture(const Image &image, Texture &texture) {
    if (image.data() == nullptr) {
        return;
    }

    // If the texture is not exist, create a new one.
    if (texture.id == nullptr) {
        // Generate a new texture, return its id.
        GLuint temp_id = 0;
        glGenTextures(1, &temp_id);
        texture.id = (ImTextureID)(intptr_t)temp_id;
    }

    // Create buffer of texture.
    const int32_t size = image.rows() * image.cols();
    if (texture.buf != nullptr) {
        SlamMemory::Free(texture.buf);
    }
    texture.buf = new uint8_t[size << 2];
    ConvertUint8ToRGBA(image.data(), texture.buf, size);

    // Bind the operations below with this texture.
    glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)texture.id);

    // Load image into texture.
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols(), image.rows(), 0, GL_BGR, GL_UNSIGNED_BYTE, texture.buf);

    glBindTexture(GL_TEXTURE_2D, 0);
}

}
