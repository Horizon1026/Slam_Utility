#include "visualizor.h"
#include "stb_image.h"

namespace SLAM_UTILITY {

template <typename ImageType>
bool Visualizor::LoadImage(const std::string &image_file, ImageType &image) {
    int32_t width = 0;
    int32_t height = 0;
    int32_t channel = 0;
    uint8_t *raw_data = stbi_load(image_file.c_str(), &width, &height, &channel, 0);
    if (raw_data == nullptr) {
        ReportError("[Visualizor] Cannot load image " << image_file);
        return false;
    }

    return true;
}

}
