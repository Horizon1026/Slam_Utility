#include "log_report.h"
#include "line_segment.h"
#include "datatype_basic.h"

using namespace SLAM_UTILITY;

void TestTranslationOfPlucker() {
    ReportInfo(YELLOW ">> Test translation of plucker line." RESET_COLOR);

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
    const LinePlucker3D plucker_c1 = plucker_w.TransformTo(q_wc1, p_wc1);
    ReportInfo("plucker_c1 is " << LogVec(plucker_c1.param()));
    const LinePlucker3D plucker_w_back = plucker_c1.TransformTo(q_wc1.inverse(), -(q_wc1.inverse() * p_wc1));
    ReportInfo("plucker_w_back is " << LogVec(plucker_w_back.param()));
}

void TestTransformBetweenPluckerAndOrthonormal() {
    ReportInfo(YELLOW ">> Test transform between plucker and orthonormal." RESET_COLOR);

    Vec6 param = Vec6::Ones();
    param << 1, 1, 0, 1, -1, 0;
    LinePlucker3D plucker_w(param);
    ReportInfo("plucker_w is " << LogVec(plucker_w.param()) << ", self check " << plucker_w.SelfCheck());

    const LineOrthonormal3D orthonormal_w(plucker_w);
    ReportInfo("orthonormal_w is " << LogVec(orthonormal_w.param()));
    ReportInfo("orthonormal_w matrix U is\n" << orthonormal_w.matrix_U());
    ReportInfo("orthonormal_w matrix W is\n" << orthonormal_w.matrix_W());
    const LinePlucker3D plucker_w_back = LinePlucker3D(orthonormal_w);
    ReportInfo("plucker_w_back is " << LogVec(plucker_w_back.param()));
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test line segment." RESET_COLOR);
    TestTranslationOfPlucker();
    TestTransformBetweenPluckerAndOrthonormal();
    return 0;
}
