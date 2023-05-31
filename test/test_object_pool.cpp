#include "object_pool.h"

using namespace SLAM_UTILITY;

int main() {

    ObjectPool<double> pool(100);

    std::unique_ptr<double, std::function<void(double *)>> ptr = pool.Get();

    return 0;
}