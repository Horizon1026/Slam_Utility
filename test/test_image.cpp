#include "datatype_image.h"
#include "log_api.h"

int main() {
    Image image1(15, 20);
    LogInfo(YELLOW "After Image image1(15, 20)");
    LogInfo("image 1 data ptr is " << LogPtr(image1.image_data()) << ", has_memory is " << image1.has_memory() << ", size is " << image1.rows() << "," << image1.cols());

    {
        Image image2 = image1;

        LogInfo(YELLOW "After image2 = image1");
        LogInfo("image 1 data ptr is " << LogPtr(image1.image_data()) << ", has_memory is " << image1.has_memory() << ", size is " << image1.rows() << "," << image1.cols());
        LogInfo("image 2 data ptr is " << LogPtr(image2.image_data()) << ", has_memory is " << image2.has_memory() << ", size is " << image2.rows() << "," << image2.cols());
    }

    Image image3;
    LogInfo(YELLOW "After Image image3;");
    LogInfo("image 3 data ptr is " << LogPtr(image3.image_data()) << ", has_memory is " << image3.has_memory() << ", size is " << image3.rows() << "," << image3.cols());

    image3 < image1;
    LogInfo(YELLOW "After image3 < image1");
    LogInfo("image 1 data ptr is " << LogPtr(image1.image_data()) << ", has_memory is " << image1.has_memory() << ", size is " << image1.rows() << "," << image1.cols());
    LogInfo("image 3 data ptr is " << LogPtr(image3.image_data()) << ", has_memory is " << image3.has_memory() << ", size is " << image3.rows() << "," << image3.cols());

    return 0;
}