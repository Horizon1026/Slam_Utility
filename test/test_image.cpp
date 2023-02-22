#include "datatype_image.h"
#include "log_api.h"

int main() {
    Image image1(20, 20);
    Image image2;

    LogInfo("image 1 data ptr is " << image1.image_data() << ", has_memory is " << image1.has_memory() << ", size is " << image1.rows() << "," << image1.cols());
    LogInfo("image 2 data ptr is " << image2.image_data() << ", has_memory is " << image2.has_memory() << ", size is " << image2.rows() << "," << image2.cols());

    image2 = image1;


    return 0;
}