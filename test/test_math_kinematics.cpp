#include "datatype_basic.h"
#include "math_kinematics.h"
#include "log_report.h"
#include "vector"

using namespace SLAM_UTILITY;

void TestTangentBase() {
    ReportInfo(YELLOW ">> Test Utility::TangentBase()" RESET_COLOR);
    const Vec3 vec(1, 1, 0);
    ReportInfo("vec is " << LogVec(vec));
    ReportInfo("base on tangent of vec is\n" << Utility::TangentBase(vec));
}

void TestConvertionBetweenEulerAndQuaternion() {
    ReportInfo(YELLOW ">> Test Utility::EulerToQuaternion() and QuaternionToEuler()" RESET_COLOR);
    Vec3 pry = Vec3(90, 50, 60);
    ReportInfo("Truth pitch, roll, yaw is " << LogVec(pry));
    Quat q = Utility::EulerToQuaternion(pry);
    ReportInfo("q is " << LogQuat(q));
    pry = Utility::QuaternionToEuler(q);
    ReportInfo("pitch, roll, yaw is " << LogVec(pry));
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test math kinematics." RESET_COLOR);

    TestTangentBase();
    TestConvertionBetweenEulerAndQuaternion();

    return 0;
}
