#include "datatype_basic.h"
#include "math_kinematics.h"
#include "log_report.h"
#include "vector"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test math kinematics." RESET_COLOR);

    const Vec3 vec(1, 0, 0);
    ReportInfo("vec is " << LogVec(vec));
    ReportInfo("base on tangent of vec is\n" << Utility::TangentBase(vec));

    return 0;
}
