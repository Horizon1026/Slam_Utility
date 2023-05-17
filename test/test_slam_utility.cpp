#include "datatype_image.h"
#include "datatype_image_pyramid.h"
#include "log_report.h"
#include "math_kinematics.h"
#include "slam_operations.h"
#include "slam_memory.h"

#include "opencv2/opencv.hpp"

std::string test_image_file_name = "../example/image.png";

void TestImage() {
    cv::Mat cv_image;
    cv_image = cv::imread(test_image_file_name, 0);

    Image image;
    image.SetImage(cv_image.data, cv_image.rows, cv_image.cols);
    std::cout << image.rows() << std::endl;
    std::cout << image.cols() << std::endl;

    float value;
    uint16_t int_value;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            image.GetPixelValue(i, j, &int_value);
            image.GetPixelValue(i, j, &value);
            std::cout << "test image.GetPixelValue is " << int_value << ", " << value << std::endl;
        }
    }

    cv::Mat show_image(image.rows(), image.cols(), CV_8UC1, image.data());
    cv::imshow("convert cv_image to image", show_image);
    cv::waitKey(0);
}

void TestPyramid() {
    cv::Mat cv_image;
    cv_image = cv::imread(test_image_file_name, 0);

    ImagePyramid pyramid;
    pyramid.SetPyramidBuff((uint8_t *)malloc(sizeof(uint8_t) * cv_image.rows * cv_image.cols));
    pyramid.SetRawImage(cv_image.data, cv_image.rows, cv_image.cols);
    pyramid.CreateImagePyramid(5);

    for (uint32_t i = 0; i < pyramid.level(); ++i) {
        Image one_level = pyramid.GetImage(i);
        cv::Mat image(one_level.rows(), one_level.cols(), CV_8UC1, one_level.data());
        cv::imshow(std::to_string(i), image);
        cv::waitKey(1);
    }
    cv::waitKey(0);
}

int main(int argc, char **argv) {
    ReportInfo("Slam Utility Test.");

    TestImage();
    TestPyramid();

    return 0;
}