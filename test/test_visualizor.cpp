#include "datatype_basic.h"
#include "datatype_image.h"
#include "visualizor.h"

#include "opencv2/opencv.hpp"

using namespace SLAM_UTILITY;

namespace {
    constexpr int32_t kScale = 2;
    constexpr int32_t kMatrixRow = 64;
    constexpr int32_t kMatrixCol = 64;
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test visualizor." RESET_COLOR);

    Mat matrix = Mat::Identity(kMatrixRow, kMatrixCol) * 10.0f;
    matrix += Mat::Random(kMatrixRow, kMatrixCol);

    uint8_t *buf = (uint8_t *)malloc(matrix.rows() * matrix.cols() * kScale * kScale * sizeof(uint8_t));
    Image image(buf, matrix.rows() * kScale, matrix.cols() * kScale);

    Visualizor &visualizor = Visualizor::GetInstance();
    visualizor.ConvertMatrixToImage<float>(matrix, image, matrix.maxCoeff(), kScale);
    visualizor.ShowImage("Matrix", image);

    // Debug.
    // cv::Mat show_image(merged_image.rows, merged_image.cols, CV_8UC3);
    // cv::cvtColor(merged_image, show_image, cv::COLOR_GRAY2BGR);
    auto texture = visualizor.image_objects()["Matrix"];
    cv::Mat rgba_image(image.rows(), image.cols(), CV_8UC4, texture.buf);
    cv::imshow("Matrix shown", rgba_image);
    cv::waitKey(0);

    visualizor.RenderMainWindow(visualizor.main_window());

    return 0;
}
