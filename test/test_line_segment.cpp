#include "basic_type.h"
#include "line_segment.h"
#include "slam_log_reporter.h"

using namespace slam_utility;

void TestTranslationOfPlucker() {
    ReportColorInfo(">> Test translation of plucker line.");

    LinePlucker3D plucker_w;
    plucker_w.Normalize();
    ReportInfo("plucker_w is " << LogVec(plucker_w.param()) << ", self check " << plucker_w.SelfCheck());
    Vec6 param = Vec6::Ones();
    param << 1, 1, 0, 1, -1, 0;
    plucker_w.param() = param;
    plucker_w.Normalize();
    ReportInfo("plucker_w is " << LogVec(plucker_w.param()) << ", self check " << plucker_w.SelfCheck());

    const Quat q_wc1 = Quat::Identity();
    const Vec3 p_wc1 = Vec3::Random();
    const LinePlucker3D plucker_c1 = plucker_w.TransformTo(p_wc1, q_wc1);
    ReportInfo("plucker_c1 is " << LogVec(plucker_c1.param()));
    const LinePlucker3D plucker_w_back = plucker_c1.TransformTo(-(q_wc1.inverse() * p_wc1), q_wc1.inverse());
    ReportInfo("plucker_w_back is " << LogVec(plucker_w_back.param()));
}

void TestProjectPointOnLine() {
    ReportColorInfo(">> Test project point on line.");

    Vec6 param = Vec6::Ones();
    param << 1, 1, 0, 1, -1, 0;
    LinePlucker3D plucker_w(param);
    ReportInfo("plucker_w is " << LogVec(plucker_w.param()) << ", self check " << plucker_w.SelfCheck());

    const Vec3 truth_p_on_line = plucker_w.GetPointOnLine(-3);
    const Vec3 p_w = truth_p_on_line + plucker_w.normal_vector();
    const Vec3 p_on_line = plucker_w.ProjectPointOnLine(p_w);

    ReportInfo("truth_p_on_line " << LogVec(truth_p_on_line));
    ReportInfo("p_w " << LogVec(p_w));
    ReportInfo("p_on_line " << LogVec(p_on_line));
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test line segment." RESET_COLOR);
    TestTranslationOfPlucker();
    TestProjectPointOnLine();
    return 0;
}
