#include "log_report.h"
#include "line_segment.h"
#include "datatype_basic.h"

using namespace SLAM_UTILITY;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test line segment." RESET_COLOR);

    LinePlucker3D plucker_w;
    plucker_w.Normalize();
    ReportInfo("plucker_w is " << LogVec(plucker_w.param()));

    const Quat q_wc1 = Quat::Identity();
    const Vec3 p_wc1 = Vec3::Random();
    LinePlucker3D plucker_c1 = plucker_w.TransformTo(q_wc1, p_wc1);
    ReportInfo("plucker_c1 is " << LogVec(plucker_c1.param()));
    LinePlucker3D plucker_w_back = plucker_c1.TransformTo(q_wc1.inverse(), -(q_wc1.inverse() * p_wc1));
    ReportInfo("plucker_w_back is " << LogVec(plucker_w_back.param()));

    return 0;
}
