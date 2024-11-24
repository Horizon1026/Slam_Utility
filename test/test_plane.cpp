#include "slam_log_reporter.h"
#include "plane.h"
#include "basic_type.h"

using namespace SLAM_UTILITY;

void TestComputePlaneModelWithThreePoints() {
    ReportColorInfo(">> Test plane fitting with three points.");
    const Vec3 p1_w = Vec3(1, 1, 0);
    const Vec3 p2_w = Vec3(1, 1, 1);
    const Vec3 p3_w = Vec3(0, 0, 0);
    Plane3D plane;
    plane.FitPlaneModel(p1_w, p2_w, p3_w);
    ReportInfo("   Plane model is " << LogVec(plane.param()));
    ReportInfo("   Distance of p1_w to plane is " << plane.GetDistanceToPlane(p1_w));
    ReportInfo("   Distance of p2_w to plane is " << plane.GetDistanceToPlane(p2_w));
    ReportInfo("   Distance of p3_w to plane is " << plane.GetDistanceToPlane(p3_w));
}

void TestComputePlaneModelWithSeveralPoints() {
    ReportColorInfo(">> Test plane fitting with several points.");
    const std::vector<Vec3> points = std::vector<Vec3>{
        Vec3(1, 1, 0),
        Vec3(1, 1, 1),
        Vec3(0, 0, 0),
        Vec3(0, 0, 1),
    };

    Plane3D plane;
    plane.FitPlaneModel(points);
    ReportInfo("   Plane model is " << LogVec(plane.param()));
    for (const auto &point : points) {
        ReportInfo("   Distance of point to plane is " << plane.GetDistanceToPlane(point));
    }
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test plane." RESET_COLOR);
    TestComputePlaneModelWithThreePoints();
    TestComputePlaneModelWithSeveralPoints();
    return 0;
}
