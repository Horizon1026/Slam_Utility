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

void TestTransformToTranslationAndRotation() {
    ReportColorInfo(">> Test T to R/t");
    Mat4 T = Mat4::Identity();
    T << 1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1;
    const Mat3 R = T.topLeftCorner<3, 3>();
    const Vec3 p = T.topRightCorner<3, 1>();
    const Quat q = Quat(R);
    ReportInfo("T " << LogMat(T));
    ReportInfo("R " << LogMat(R));
    ReportInfo("p " << LogVec(p));
    ReportInfo("q " << LogQuat(q));

    const Mat4 inv_T = T.inverse();
    const Mat3 inv_R = inv_T.topLeftCorner<3, 3>();
    const Vec3 inv_p = inv_T.topRightCorner<3, 1>();
    const Quat inv_q = Quat(inv_R);
    ReportInfo("inv_T " << LogMat(inv_T));
    ReportInfo("inv_R " << LogMat(inv_R));
    ReportInfo("inv_p " << LogVec(inv_p));
    ReportInfo("inv_q " << LogQuat(inv_q));
}

void TestExtrinsicRotateAlignAxis() {
    ReportColorInfo(">> Test extrinsic rotating align axis.");
    Mat3 R_ic0 = Mat3::Identity();
    R_ic0 << 1, 0, 0, 0, 0.866025, -0.5, 0, 0.5, 0.866025;
    const Quat q_ic0 = Quat(R_ic0);
    const float angle_deg = 30.0f;
    const Mat3 R_c0c1 = Eigen::AngleAxisf(angle_deg * kDegToRad, Vec3::UnitY()).toRotationMatrix();
    const Mat3 R_ic1 = R_ic0 * R_c0c1;
    const Quat q_ic1 = Quat(R_ic1);
    ReportInfo("R_ic0 " << LogMat(R_ic0));
    ReportInfo("q_ic0 " << LogQuat(q_ic0));
    ReportInfo("R_ic1 " << LogMat(R_ic1));
    ReportInfo("q_ic1 " << LogQuat(q_ic1));
}

void TestLeftAndRightJacobian() {
    ReportColorInfo(">> Test left/right jacobian.");
    const Vec3 omega = Vec3::Random();
    const Mat3 Jr = Utility::RightJacobian(omega);
    const Mat3 Jl = Utility::LeftJacobian(omega);
    ReportInfo("Jr " << LogMat(Jr));
    ReportInfo("Jl " << LogMat(Jl));
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test math kinematics." RESET_COLOR);
    TestTangentBase();
    TestConvertionBetweenEulerAndQuaternion();
    TestLogAndExp();
    TestTransformToTranslationAndRotation();
    TestExtrinsicRotateAlignAxis();
    TestLeftAndRightJacobian();
    return 0;
}
