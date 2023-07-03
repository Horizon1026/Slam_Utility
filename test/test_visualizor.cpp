#include "datatype_basic.h"
#include "datatype_image.h"
#include "visualizor.h"

#include "opencv2/opencv.hpp"

using namespace SLAM_UTILITY;

namespace {
    constexpr int32_t kScale = 3;
    constexpr int32_t kMatrixRow = 90;
    constexpr int32_t kMatrixCol = 180;
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test visualizor." RESET_COLOR);

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

    return 0;
}
