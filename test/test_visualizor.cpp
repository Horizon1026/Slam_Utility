#include "datatype_basic.h"
#include "datatype_image.h"
#include "visualizor.h"
#include "log_report.h"
#include "slam_operations.h"

#include "iostream"
#include "dirent.h"
#include "vector"
#include "cstring"

#include "opencv2/opencv.hpp"

using namespace SLAM_UTILITY;

namespace {
    constexpr int32_t kScale = 3;
    constexpr int32_t kMatrixRow = 90;
    constexpr int32_t kMatrixCol = 180;
}

bool GetFilesInPath(std::string dir, std::vector<std::string> &filenames) {
    DIR *ptr_dir;
    struct dirent *ptr;
    if (!(ptr_dir = opendir(dir.c_str()))) {
        ReportError("Cannot open dir " << dir);
        return false;
    }

    filenames.reserve(1000);

    while ((ptr = readdir(ptr_dir)) != 0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
            filenames.emplace_back(dir + "/" + ptr->d_name);
        }
    }

    closedir(ptr_dir);

    return true;
}

void TestVisualizorStatic() {
    // Create image of matrix.
    Mat matrix = Mat::Identity(kMatrixRow, kMatrixCol) * 10.0f;
    matrix += Mat::Random(kMatrixRow, kMatrixCol);
    uint8_t *buf = (uint8_t *)malloc(matrix.rows() * matrix.cols() * kScale * kScale * sizeof(uint8_t));
    Image image_matrix(buf, matrix.rows() * kScale, matrix.cols() * kScale);

    // Create image of png file.
    cv::Mat cv_image = cv::imread("../example/image.png", 0);
    Image image_png(cv_image.data, cv_image.rows, cv_image.cols);

    // Show images.
    Visualizor &visualizor = Visualizor::GetInstance();
    visualizor.ConvertMatrixToImage<float>(matrix, image_matrix, matrix.maxCoeff(), kScale);
    visualizor.ShowImage("Matrix", image_matrix);
    visualizor.ShowImage("PNG file", image_png);
    visualizor.RenderMainWindow(visualizor.main_window());
}

void TestVisualizorDynamic() {
    std::vector<std::string> cam0_filenames;
    RETURN_IF(!GetFilesInPath("/home/horizon/Desktop/date_sets/euroc/MH_01_easy/mav0/cam0/data", cam0_filenames));
    std::sort(cam0_filenames.begin(), cam0_filenames.end());

    std::vector<std::string> cam1_filenames;
    RETURN_IF(!GetFilesInPath("/home/horizon/Desktop/date_sets/euroc/MH_01_easy/mav0/cam1/data", cam1_filenames));
    std::sort(cam1_filenames.begin(), cam1_filenames.end());

    // Initialize visualizor.
    Visualizor &visualizor = Visualizor::GetInstance();

    const int32_t max_size = std::min(cam0_filenames.size(), cam1_filenames.size());
    for (int32_t i = 0; i < max_size; ++i) {
        cv::Mat cv_image_left = cv::imread(cam0_filenames[i], 0);
        cv::Mat cv_image_right = cv::imread(cam1_filenames[i], 0);
        Image image_left(cv_image_left.data, cv_image_left.rows, cv_image_left.cols);
        Image image_right(cv_image_right.data, cv_image_right.rows, cv_image_right.cols);

        visualizor.ShowImage("Camera Left", image_left);
        visualizor.ShowImage("Camera Right", image_right);

        BREAK_IF(!visualizor.RenderMainWindowOnce(visualizor.main_window()));
    }
    visualizor.RenderMainWindow(visualizor.main_window());
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test visualizor based on ImGui." RESET_COLOR);

    if (argc == 1) {
        ReportInfo("Argc is 1, run static image test.");
        TestVisualizorStatic();
    } else {
        ReportInfo("Argc is not 1, run dynamic image test.");
        TestVisualizorDynamic();
    }

    return 0;
}
