#include "datatype_image.h"
#include "datatype_image_pyramid.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_memory.h"
#include "slam_operations.h"
#include "visualizor_2d.h"

using namespace slam_visualizor;

std::string test_image_file_name = "../examples/image.png";

void TestImage() {
    GrayImage image;
    Visualizor2D::LoadImage(test_image_file_name, image);
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

    Visualizor2D::ShowImage("Loaded png image", image);
    Visualizor2D::WaitKey(0);
}

void TestPyramid() {
    GrayImage image;
    Visualizor2D::LoadImage(test_image_file_name, image);

    ImagePyramid pyramid;
    pyramid.SetPyramidBuff((uint8_t *)malloc(sizeof(uint8_t) * image.rows() * image.cols()));
    pyramid.SetRawImage(image.data(), image.rows(), image.cols());
    pyramid.CreateImagePyramid(5);

    for (uint32_t i = 0; i < pyramid.level(); ++i) {
        GrayImage one_level = pyramid.GetImage(i);
        Visualizor2D::ShowImage(std::to_string(i), one_level);
        Visualizor2D::WaitKey(1);
    }
    Visualizor2D::WaitKey(0);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW "Slam Utility Test." RESET_COLOR);

    TestImage();
    TestPyramid();

    return 0;
}