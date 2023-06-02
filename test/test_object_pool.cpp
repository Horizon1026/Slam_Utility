#include "log_report.h"
#include "object_pool.h"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test object pool." RESET_COLOR);

    ObjectPool<double> pool(10);
    {
    	std::vector<ObjectPtr<double>> objects;
		for (int32_t i = 0; i < 10; ++i) {
            auto ptr = pool.Get();
    	    objects.emplace_back(std::move(ptr));
		    *objects.back().get() = 1.0 + static_cast<double>(i);
    	}
    }

    std::vector<ObjectPtr<double>> objects;
	for (int32_t i = 0; i < 10; ++i) {
        objects.emplace_back(pool.Get());
        ReportInfo("Item in object pool is " << *objects.back().get());
    }

    return 0;
}