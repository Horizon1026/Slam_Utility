#include "visualizor.h"
#include "log_report.h"
#include "slam_memory.h"

namespace SLAM_UTILITY {

void Visualizor::ConvertUint8ToRGBA(const uint8_t *gray, uint8_t *rgba, int32_t gray_size) {
    for (int32_t i = 0; i < gray_size; ++i) {
        const int32_t idx = i * 3;
        std::fill_n(rgba + idx, 3, gray[i]);
    }
}

void Visualizor::ConvertImageToTexture(const Image &image, Texture &texture) {
    if (image.data() == nullptr) {
        return;
    }

    // If the texture is not exist, create a new one.
    if (texture.id == nullptr) {
        // Generate a new texture, return its id.
        GLuint temp_id = 0;
        glGenTextures(1, &temp_id);
        texture.id = (ImTextureID)(intptr_t)temp_id;
    }

    // Create buffer of texture.
    const int32_t size = image.rows() * image.cols();
    if (texture.buf != nullptr) {
        SlamMemory::Free(texture.buf);
    }
    texture.buf = (uint8_t *)SlamMemory::Malloc(size * 3 * sizeof(uint8_t));
    ConvertUint8ToRGBA(image.data(), texture.buf, size);

    // Bind the operations below with this texture.
    glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)texture.id);

    // Load image into texture.
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols(), image.rows(), 0, GL_BGR, GL_UNSIGNED_BYTE, texture.buf);

    glBindTexture(GL_TEXTURE_2D, 0);
}

}
