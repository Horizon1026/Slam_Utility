#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "thread"


namespace SLAM_UTILITY {

// Declare static member variables.
GLint Visualizor::image_cols_  = 0;
GLint Visualizor::image_rows_  = 0;
GLint Visualizor::image_pixel_length_  = 0;
GLubyte *Visualizor::image_data_ = nullptr;

Visualizor::Visualizor(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(0, 0);
}

Visualizor::~Visualizor() {
    if (image_data_ != nullptr) {
        SlamMemory::Free(image_data_);
    }
}

bool Visualizor::ShowImage(const std::string &window_title, const Image &image) {
    if (image.data() == nullptr || image.rows() < 1 || image.cols() < 1) {
        ReportWarn("[Visualizor] Image to be shown is invalid.");
        return false;
    }

    image_rows_ = image.rows();
    image_cols_ = image.cols();
    image_pixel_length_ = image.rows() * image.cols();

    if (image_data_ != nullptr) {
        SlamMemory::Free(image_data_);
    }
    image_data_ = (GLubyte *)SlamMemory::Malloc(image_pixel_length_ * 4 * sizeof(GLubyte));

    // Convert uint8 image to rgb image.
    for (int32_t row = 0; row < image_rows_; ++row) {
        for (int32_t col = 0; col < image_cols_; ++col) {
            const uint32_t i = (image_rows_ - row - 1) * image_cols_ + col;
            const uint32_t idx = (row * image_cols_ + col) << 2;
            image_data_[idx] = image.data()[i];
            image_data_[idx + 1] = image_data_[idx];
            image_data_[idx + 2] = image_data_[idx];
            image_data_[idx + 3] = 255;
        }
    }

    glutInitWindowSize(image.cols(), image.rows());
    glutCreateWindow(window_title.data());
    glutDisplayFunc(&RefreshBuffer);
    glutMainLoop();

    return true;
}

void Visualizor::RefreshBuffer() {
    glDrawPixels(image_cols_, image_rows_, GL_RGBA, GL_UNSIGNED_BYTE, image_data_);
    glFlush();
    glutSwapBuffers();
}

}
