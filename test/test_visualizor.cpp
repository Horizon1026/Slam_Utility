#include "datatype_basic.h"
#include "datatype_image.h"
#include "visualizor.h"
#include "log_report.h"
#include "slam_operations.h"

#include "iostream"
#include "dirent.h"
#include "vector"
#include "cstring"

#include "thread"
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
    ReportInfo(YELLOW ">> Test visualizor show static image." RESET_COLOR);

    // Create image of matrix.
    Mat matrix = Mat::Identity(kMatrixRow, kMatrixCol) * 10.0f;
    matrix += Mat::Random(kMatrixRow, kMatrixCol);
    uint8_t *buf = (uint8_t *)malloc(matrix.rows() * matrix.cols() * kScale * kScale * sizeof(uint8_t));
    GrayImage image_matrix(buf, matrix.rows() * kScale, matrix.cols() * kScale);
    Visualizor::ConvertMatrixToImage<float>(matrix, image_matrix, 15.0f, kScale);
    Visualizor::DrawSolidRectangle(image_matrix, 10, 10, 200, 200, static_cast<uint8_t>(0));
    for (int32_t i = 0; i < 10; ++i) {
        Visualizor::DrawBressenhanLine(image_matrix, 111 - 10 * i, 10 * i, 100, 100, static_cast<uint8_t>(255));
    }
    Visualizor::DrawSolidCircle(image_matrix, 130, 200, 10, static_cast<uint8_t>(127));
    Visualizor::DrawString(image_matrix, "This is a string.", 240, 100 - 16, static_cast<uint8_t>(0), 99);
    Visualizor::DrawString(image_matrix, "This is a string.", 240, 100, static_cast<uint8_t>(127), 16);

    // Create image of png file.
    cv::Mat cv_image = cv::imread("../example/image.png");
    RgbImage image_png(cv_image.data, cv_image.rows, cv_image.cols);
    Visualizor::DrawHollowRectangle(image_png, 20, 20, 200, 200, RgbPixel{.r = 255, .g = 255, .b = 10});
    for (int32_t i = 0; i < 10; ++i) {
        Visualizor::DrawNaiveLine(image_png, 111 - 10 * i, 10 * i, 100, 100, RgbPixel{.r = 10, .g = 255, .b = 10});
    }
    Visualizor::DrawHollowCircle(image_png, 130, 200, 10, RgbPixel{.r = 10, .g = 10, .b = 255});
    Visualizor::DrawString(image_png, "This is a string.", 0, 0, RgbPixel{.r = 255, .g = 255, .b = 10}, 24);

    // Test visualizor.
    Visualizor::ShowImage("Matrix", image_matrix);
    Visualizor::WaitKey(1);
    Visualizor::ShowImage("PNG GrayImage", image_png);
    Visualizor::WaitKey(0);

}

void TestVisualizorDynamic() {
    ReportInfo(YELLOW ">> Test visualizor show dynamic image." RESET_COLOR);

    std::vector<std::string> cam0_filenames;
    RETURN_IF(!GetFilesInPath("/home/horizon/Desktop/date_sets/euroc/MH_01_easy/mav0/cam0/data", cam0_filenames));
    std::sort(cam0_filenames.begin(), cam0_filenames.end());

    std::vector<std::string> cam1_filenames;
    RETURN_IF(!GetFilesInPath("/home/horizon/Desktop/date_sets/euroc/MH_01_easy/mav0/cam1/data", cam1_filenames));
    std::sort(cam1_filenames.begin(), cam1_filenames.end());

    for (uint32_t i = 0; i < 100; ++i) {
        cv::Mat cv_image_left = cv::imread(cam0_filenames[i], 0);
        cv::Mat cv_image_right = cv::imread(cam1_filenames[i], 0);
        GrayImage image_left(cv_image_left.data, cv_image_left.rows, cv_image_left.cols);
        GrayImage image_right(cv_image_right.data, cv_image_right.rows, cv_image_right.cols);
        Visualizor::ShowImage("Left", image_left);
        Visualizor::ShowImage("Right", image_right);
        Visualizor::WaitKey(20);
    }
    Visualizor::WaitKey(0);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test visualizor." RESET_COLOR);

    if (argc == 1) {
        ReportInfo("Argc is 1, run static image test.");
        TestVisualizorStatic();
    } else {
        ReportInfo("Argc is not 1, run dynamic image test.");
        TestVisualizorDynamic();
    }

    return 0;
}
