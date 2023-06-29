#include "datatype_basic.h"
#include "datatype_image.h"
#include "visualizor.h"

#include "opencv2/opencv.hpp"

using namespace SLAM_UTILITY;

namespace {
    constexpr int32_t kScale = 2;
    constexpr int32_t kMatrixRow = 123;
    constexpr int32_t kMatrixCol = 153;
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test visualizor." RESET_COLOR);

    Mat matrix = Mat::Identity(kMatrixRow, kMatrixCol) * 10.0f;
    matrix += Mat::Random(kMatrixRow, kMatrixCol);

    uint8_t *buf = (uint8_t *)malloc(matrix.rows() * matrix.cols() * kScale * kScale * sizeof(uint8_t));
    Image image(buf, matrix.rows() * kScale, matrix.cols() * kScale);

    Visualizor visualizor;
    visualizor.ConvertMatrixToImage<float>(matrix, image, matrix.maxCoeff(), kScale);

    visualizor.ShowImage("Title", image);

    cv::Mat cv_image(image.rows(), image.cols(), CV_8UC1, image.data());
    cv::imshow("Matrix shown", cv_image);
    cv::waitKey(0);

    return 0;
}
