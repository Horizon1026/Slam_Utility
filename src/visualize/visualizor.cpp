#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"

#include "thread"
#include "unistd.h"

namespace SLAM_UTILITY {

std::map<std::string, VisualizorWindow> Visualizor::windows_;

Visualizor &Visualizor::GetInstance() {
    static Visualizor instance;
    return instance;
}

bool Visualizor::ShowImage(const std::string &window_title, const Image &image, bool resizable) {
    if (image.data() == nullptr || image.rows() < 1 || image.cols() < 1) {
        ReportError("[Visualizor] ShowImage() got an invalid image.");
        return false;
    }

    glfwSetErrorCallback(Visualizor::ErrorCallback);

    if (!glfwInit()) {
        ReportError("[Visualizor] GLFW initialize failed.");
        return false;
    }

    if (resizable) {
        glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    } else {
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    }

    VisualizorWindow *window = GetWindowPointer(window_title, image.cols(), image.rows());

    glfwMakeContextCurrent(window->glfw_window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);
    glfwSetWindowPos(window->glfw_window, 0, 0);
    glfwShowWindow(window->glfw_window);
    glfwSetKeyCallback(window->glfw_window, Visualizor::KeyboardCallback);

    window->texture_id = Visualizor::CreateTextureByImage(image);

    glfwMakeContextCurrent(window->glfw_window);
    glColor3f(1.0f, 1.0f, 1.0f);
    glfwHideWindow(window->glfw_window);

    return true;
}

void Visualizor::WaitKey(int32_t delay_s) {
    // Display add window hidden in ShowImage().
    for (auto &item : Visualizor::windows_) {
        auto &glfw_window = item.second.glfw_window;
        if (!glfwWindowShouldClose(glfw_window)) {
            glfwShowWindow(glfw_window);
        } else {
            glfwHideWindow(glfw_window);
        }
    }

    while (!Visualizor::windows_.empty()) {
        for (auto &item : Visualizor::windows_) {
            auto &window = item.second;

            if (!glfwWindowShouldClose(window.glfw_window)) {
                glfwMakeContextCurrent(window.glfw_window);
                Visualizor::DrawTextureInCurrentWindow(window.texture_id);
                glfwSwapBuffers(window.glfw_window);
            } else {
                glfwHideWindow(window.glfw_window);
            }

            if (delay_s > 0) {
                // Waiting for key event during specified duration.
                sleep(delay_s);
                glfwWaitEvents();
                break;
            } else {
                // Waiting for key event until all windows are hidden.
                glfwPollEvents();
            }
        }
    }

    // Clear all windows and recovery resources.
    Visualizor::windows_.clear();
    glfwTerminate();

}

void Visualizor::ErrorCallback(int32_t error, const char *description) {
    ReportError("[Visualizor] Error detected :" << description);
}

void Visualizor::KeyboardCallback(GLFWwindow *window, int32_t key, int32_t scan_code, int32_t action, int32_t mods) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

VisualizorWindow *Visualizor::GetWindowPointer(const std::string &title, int32_t width, int32_t height) {
    auto item = Visualizor::windows_.find(title);
    if (item == Visualizor::windows_.end()) {
        // If window with selected title is not found, create a new window.
        auto iter = Visualizor::windows_.insert(std::make_pair(title, VisualizorWindow()));
        iter.first->second.glfw_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);

        // If insert failed, clear it.
        if (iter.first->second.glfw_window == nullptr) {
            Visualizor::windows_.erase(title);
            return nullptr;
        }
        return &(iter.first->second);
    } else {
        // Return the exist window.
        return &(item->second);
    }
}

GLuint Visualizor::CreateTextureByImage(const Image &image) {
    GLuint texture_id = 0;
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    const int32_t size = image.rows() * image.cols();
    uint8_t image_buff[size * 3];
    for (int32_t i = 0; i < size; ++i) {
        const int32_t idx = i * 3;
        image_buff[idx] = image.data()[i];
        image_buff[idx + 1] = image.data()[i];
        image_buff[idx + 2] = image.data()[i];
    }
    glTexImage2D(GL_TEXTURE_2D, 0, GL_BGR, image.cols(), image.rows(), 0, GL_RGB, GL_UNSIGNED_BYTE, image_buff);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return texture_id;
}

void Visualizor::DrawTextureInCurrentWindow(GLuint texture_id) {
    int32_t width, height;
    glfwGetFramebufferSize(glfwGetCurrentContext(), &width, &height);

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.f, 1.f, 0.f, 1.f, 0.f, 1.f);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    glBegin(GL_QUADS);

    glTexCoord2f(0.f, 0.f);
    glVertex2f(0.f, 0.f);

    glTexCoord2f(1.f, 0.f);
    glVertex2f(1.f, 0.f);

    glTexCoord2f(1.f, 1.f);
    glVertex2f(1.f, 1.f);

    glTexCoord2f(0.f, 1.f);
    glVertex2f(0.f, 1.f);

    glEnd();
}

}
