#include "log_report.h"
#include "object_pool.h"
#include "datatype_basic.h"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test object pool." RESET_COLOR);

    ObjectPool<Mat1> pool(10);
    {
    	std::vector<ObjectPtr<Mat1>> objects;
		for (int32_t i = 0; i < 10; ++i) {
            auto ptr = pool.Get();
    	    objects.emplace_back(std::move(ptr));
		    *objects.back().get() = Mat1::Identity() * (1 + i);
    	}
    }

    std::vector<ObjectPtr<Mat1>> objects;
	for (int32_t i = 0; i < 10; ++i) {
        objects.emplace_back(pool.Get());
        ReportInfo("Item in object pool is " << *objects.back().get());
    }

    return 0;
}