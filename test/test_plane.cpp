#include "basic_type.h"
#include "plane.h"
#include "slam_log_reporter.h"

using namespace slam_utility;

void TestComputePlaneModelWithThreePoints() {
    ReportColorInfo(">> Test plane fitting with three points.");
    const Vec3 p1_w = Vec3(1, 1, 0);
    const Vec3 p2_w = Vec3(1, 1, 1);
    const Vec3 p3_w = Vec3(0, 0, 0);
    Plane3D plane;
    plane.FitPlaneModel(p1_w, p2_w, p3_w);
    ReportInfo("   Plane model is " << LogVec(plane.param()));
    ReportInfo("   Covariance of normal distribution is\n" << LogMat(plane.normal_distribution().covariance()));
    ReportInfo("   Distance of p1_w to plane is " << plane.GetDistanceToPlane(p1_w));
    ReportInfo("   Distance of p2_w to plane is " << plane.GetDistanceToPlane(p2_w));
    ReportInfo("   Distance of p3_w to plane is " << plane.GetDistanceToPlane(p3_w));
}

void TestComputePlaneModelLseWithSeveralPoints(const std::vector<Vec3> &points) {
    ReportColorInfo(">> Test plane fitting(LSE) with several points.");
    Plane3D plane;
    plane.FitPlaneModelLse(points);
    ReportInfo("   Plane model is " << LogVec(plane.param()));
    ReportInfo("   Covariance of normal distribution is\n" << LogMat(plane.normal_distribution().covariance()));
    for (const auto &point: points) {
        ReportInfo("   Distance of point to plane is " << plane.GetDistanceToPlane(point));
    }
}

void TestComputePlaneModelPcaWithSeveralPoints(const std::vector<Vec3> &points) {
    ReportColorInfo(">> Test plane fitting(PCA) with several points.");
    Plane3D plane;
    plane.FitPlaneModelPca(points);
    ReportInfo("   Plane model is " << LogVec(plane.param()));
    ReportInfo("   Covariance of normal distribution is\n" << LogMat(plane.normal_distribution().covariance()));
    for (const auto &point: points) {
        ReportInfo("   Distance of point to plane is " << plane.GetDistanceToPlane(point));
    }
}

void TestComputePlaneModelIncrementally(const std::vector<Vec3> &points) {
    ReportColorInfo(">> Test compute plane model incrementally.");
    Plane3D plane;
    for (const auto &point: points) {
        plane.AddNewPointToFitPlaneModel(point);
    }
    plane.GeneratePlaneModelParameters();
    ReportInfo("   Plane model is " << LogVec(plane.param()));
    ReportInfo("   Covariance of normal distribution is\n" << LogMat(plane.normal_distribution().covariance()));
    for (const auto &point: points) {
        ReportInfo("   Distance of point to plane is " << plane.GetDistanceToPlane(point));
    }
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test plane." RESET_COLOR);
    const std::vector<Vec3> points = std::vector<Vec3> {
        Vec3(1, 1, 0), Vec3(1, 1, 1), Vec3(0, 0, 0), Vec3(0, 0, 1), Vec3(0.5, 0.5, 0.5),
        Vec3(1, 0, 0), Vec3(0, 1, 1), Vec3(1, 0, 1), Vec3(0, 1, 0), Vec3(0.2, 0.8, 0.2),
    };
    TestComputePlaneModelWithThreePoints();
    TestComputePlaneModelLseWithSeveralPoints(points);
    TestComputePlaneModelPcaWithSeveralPoints(points);
    TestComputePlaneModelIncrementally(points);
    return 0;
}
