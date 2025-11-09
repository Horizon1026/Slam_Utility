#include "basic_type.h"
#include "object_pool.h"
#include "slam_log_reporter.h"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test object pool." RESET_COLOR);

    ObjectPool<Mat> pool(10);

    {
        std::vector<ObjectPtr<Mat>> objects;
        for (int32_t i = 0; i < 10; ++i) {
            auto ptr = pool.Get();
            objects.emplace_back(std::move(ptr));
            *objects.back().get() = Mat::Identity(i + 1, i + 1) * (1 + i);
        }
    }

    std::vector<ObjectPtr<Mat>> objects;
    for (int32_t i = 0; i < 10; ++i) {
        objects.emplace_back(pool.Get());
        ReportInfo("Item in object pool is\n" << *objects.back().get());
    }

    return 0;
}
