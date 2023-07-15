#include "visualizor.h"
#include "slam_memory.h"

#define STB_IMAGE_IMPLEMENTATION
// #define STB_IMAGE_STATIC
#define STBI_NO_PSD
#define STBI_NO_TGA
#define STBI_NO_GIF
#define STBI_NO_HDR
#define STBI_NO_PIC
#include "stb_image.h"
#include "stb_image_write.h"

namespace SLAM_UTILITY {

template <>
bool Visualizor::LoadImage<GrayImage>(const std::string &image_file, GrayImage &image) {
    int32_t width = 0;
    int32_t height = 0;
    int32_t channel = 0;
    uint8_t *raw_data = stbi_load(image_file.c_str(), &width, &height, &channel, 0);
    if (raw_data == nullptr) {
        ReportError("[Visualizor] Cannot load image " << image_file);
        return false;
    }

    const int32_t size = width * height;
    uint8_t *gray_data = (uint8_t *)SlamMemory::Malloc(size * sizeof(uint8_t));

    switch (channel) {
        case 3: {
            Visualizor::ConvertRgbToUint8(raw_data, gray_data, size);
            break;
        }
        case 1: {
            std::copy_n(raw_data, size, gray_data);
            break;
        }
        default: {
            ReportError("[Visualizor] Cannot decode image " << image_file);
            SlamMemory::Free(raw_data);
            SlamMemory::Free(gray_data);
            return false;
        }
    }

    // Memory recovery is done in destory function.
    SlamMemory::Free(raw_data);
    image.SetImage(gray_data, height, width, true);
    return true;
}

template <>
bool Visualizor::LoadImage<RgbImage>(const std::string &image_file, RgbImage &image) {
    int32_t width = 0;
    int32_t height = 0;
    int32_t channel = 0;
    uint8_t *raw_data = stbi_load(image_file.c_str(), &width, &height, &channel, 0);
    if (raw_data == nullptr) {
        ReportError("[Visualizor] Cannot load image " << image_file);
        return false;
    }

    const int32_t size = width * height * 3;
    uint8_t *rgb_data = (uint8_t *)SlamMemory::Malloc(size * sizeof(uint8_t));

    switch (channel) {
        case 3: {
            std::copy_n(raw_data, size, rgb_data);
            break;
        }
        case 1: {
            Visualizor::ConvertUint8ToRgb(raw_data, rgb_data, size / 3);
            break;
        }
        default: {
            ReportError("[Visualizor] Cannot decode image " << image_file);
            SlamMemory::Free(raw_data);
            SlamMemory::Free(rgb_data);
            return false;
        }
    }

    // Memory recovery is done in destory function.
    SlamMemory::Free(raw_data);
    image.SetImage(rgb_data, height, width, true);
    return true;
}

}
