#include "basic_type.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "vector"

using namespace SLAM_UTILITY;

void TestTangentBase() {
    ReportColorInfo(">> Test Utility::TangentBase()");
    const Vec3 vec(1, 1, 0);
    ReportInfo("vec is " << LogVec(vec));
    ReportInfo("base on tangent of vec is\n" << Utility::TangentBase(vec));
}

void TestConvertionBetweenEulerAndQuaternion() {
    ReportColorInfo(">> Test Utility::EulerToQuaternion() and QuaternionToEuler()");
    Vec3 pry = Vec3(90, 50, 60);
    ReportInfo("Truth pitch, roll, yaw is " << LogVec(pry));
    const Quat q = Utility::EulerToQuaternion(pry);
    ReportInfo("q is " << LogQuat(q));
    pry = Utility::QuaternionToEuler(q);
    ReportInfo("pitch, roll, yaw is " << LogVec(pry));
}

void TestLogAndExp() {
    ReportColorInfo(">> Test Utility::Logarithm() and Utility::Exponent()");
    const Quat q = Quat(0, 0.92388, 0.382683, 0).normalized();
    const Vec3 omega = Utility::Logarithm(q);
    const Quat new_q = Utility::Exponent(omega);
    ReportInfo("Truth q is " << LogQuat(q));
    ReportInfo("omega is " << LogVec(omega));
    ReportInfo("New q is " << LogQuat(new_q));
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test math kinematics." RESET_COLOR);
    TestTangentBase();
    TestConvertionBetweenEulerAndQuaternion();
    TestLogAndExp();
    return 0;
}
