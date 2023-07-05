#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"

#include "thread"
#include "unistd.h"

namespace SLAM_UTILITY {

Visualizor &Visualizor::GetInstance() {
    static Visualizor instance;
    return instance;
}

void Visualizor::GlfwErrorCallback(int error, const char *description) {
    ReportError("[Visualizor] GLFW error " << error << " : " << description);
}

Visualizor::Visualizor() {
    glfwTerminate();
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

    std::unique_lock l(image_deque_lock_);
    image_deque_.emplace_back(std::make_pair(window_title, image));
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

void Visualizor::RenderMainWindow() {
    while (RenderMainWindowOnce()) {}
}

bool Visualizor::RenderMainWindowOnce() {
    if (!glfwWindowShouldClose(main_window_)) {
        // Poll and handle events.
        glfwPollEvents();

        // Start the Dear ImGui frame.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Refresh all components of main window.
        RefreshMainWindow(main_window_);

        // Rendering the frame.
        ImGui::Render();

        // Config the affine transform from NDC to screen.
        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(main_window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        // Config the color of background. Then set it to be background.
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Refresh buffers.
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(main_window_);

        ProcessKeyboardMessage(main_window_);

        // Wait 10ms every loop.
        usleep(1000);

        return true;
    }

    return false;
}

void Visualizor::RefreshMainWindow(GLFWwindow *window) {
    // Load items in deque.
    ProcessDeques();

    // Iterate all items to be shown.
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

void Visualizor::ProcessDeques() {
    // Process image deque.
    {
        std::unique_lock l(image_deque_lock_);
        while (!image_deque_.empty()) {
            ProcessImageDequeItem(image_deque_.front().first, image_deque_.front().second);
            image_deque_.pop_front();
        }
    }
}

void Visualizor::ProcessImageDequeItem(const std::string &window_title, const Image &image) {
    if (image.data() == nullptr || image.cols() < 1 || image.rows() < 1) {
        return;
    }

    // If this is new window, create new texture by this image.
    // Or only update the exist texture.
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
}

}
