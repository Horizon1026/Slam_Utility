#include "basic_type.h"
#include "quadratic_plane.h"
#include "slam_log_reporter.h"

using namespace slam_utility;

void TestQuadraticPlane(const std::string &name, const std::vector<Vec3> &pts, const Vec3 &eval_pt) {
    QuadraticPlane quad;
    if (!quad.FitModel(pts)) {
        ReportError("Fit " << name << " failed.");
        return;
    }
    QuadraticPlane::Curvatures cur;
    if (!quad.ComputeCurvaturesAtPoint(eval_pt, cur)) {
        ReportError("Calc curvature of " << name << " failed.");
        return;
    }

    ReportInfo("\n[" << name << "] Evaluation point: (" << eval_pt.x() << ", " << eval_pt.y() << ", " << eval_pt.z() << ")");
    ReportInfo("  [K] Gaussian Curvature  : " << cur.gaussian_curvature);
    ReportInfo("  [H] Mean Curvature      : " << cur.mean_curvature);
    ReportInfo("  [k1] Max Principal Curv : " << cur.max_principal_curvature);
    ReportInfo("  [k2] Min Principal Curv : " << cur.min_principal_curvature);
}

int main(int argc, char **argv) {
    ReportColorWarn(">> Test quadratic plane with dense & stable points.");

    // Plane: z=0
    // Ground truth: K=0, H=0
    std::vector<Vec3> plane_pts;
    plane_pts.emplace_back(0, 0, 0);
    plane_pts.emplace_back(1, 0, 0);
    plane_pts.emplace_back(0, 1, 0);
    plane_pts.emplace_back(1, 1, 0);
    plane_pts.emplace_back(-1, 0, 0);
    plane_pts.emplace_back(0, -1, 0);
    plane_pts.emplace_back(0.5, 0.5, 0);
    plane_pts.emplace_back(-0.5, 0.3, 0);
    plane_pts.emplace_back(0.7, -0.2, 0);
    plane_pts.emplace_back(-0.9, 0.6, 0);
    TestQuadraticPlane("Plane", plane_pts, Vec3(0,0,0));

    // Unit sphere: x²+y²+z²=1
    // Ground truth: K=1, H=1, k1=k2=1
    std::vector<Vec3> sphere_pts;
    sphere_pts.emplace_back(1, 0, 0);
    sphere_pts.emplace_back(0, 1, 0);
    sphere_pts.emplace_back(0, 0, 1);
    sphere_pts.emplace_back(-1, 0, 0);
    sphere_pts.emplace_back(0, -1, 0);
    sphere_pts.emplace_back(0, 0, -1);
    sphere_pts.emplace_back(0.5f, 0.5f, std::sqrt(0.5f));
    sphere_pts.emplace_back(0.5f, -0.5f, std::sqrt(0.5f));
    sphere_pts.emplace_back(-0.5f, 0.5f, std::sqrt(0.5f));
    sphere_pts.emplace_back(0.2f, 0.6f, std::sqrt(1.0f - 0.04f - 0.36f));
    sphere_pts.emplace_back(0.6f, 0.3f, std::sqrt(1.0f - 0.36f - 0.09f));
    TestQuadraticPlane("Unit Sphere", sphere_pts, Vec3(0,0,1));

    // Saddle surface: z = x² - y²
    // Ground truth: H=0, K<0
    std::vector<Vec3> saddle_pts;
    saddle_pts.emplace_back(0, 0, 0);
    saddle_pts.emplace_back(1, 0, 1);
    saddle_pts.emplace_back(0, 1, -1);
    saddle_pts.emplace_back(-1, 0, 1);
    saddle_pts.emplace_back(0, -1, -1);
    saddle_pts.emplace_back(1, 1, 0);
    saddle_pts.emplace_back(0.5, 0, 0.25);
    saddle_pts.emplace_back(0, 0.5, -0.25);
    saddle_pts.emplace_back(-0.5, 0.5, 0.0);
    saddle_pts.emplace_back(0.8, 0.2, 0.6);
    saddle_pts.emplace_back(0.2, 0.8, -0.6);
    saddle_pts.emplace_back(-0.6, 0.4, 0.2);
    TestQuadraticPlane("Saddle Surface z=x²-y²", saddle_pts, Vec3(0,0,0));

    // Parabolic surface: z = x² + y²
    // Ground truth: K=4, H=2 at (0,0,0) ? No, wait.
    // z = x^2 + y^2
    // F(x,y,z) = x^2 + y^2 - z = 0
    // Fx = 2x, Fy = 2y, Fz = -1
    // At (0,0,0): Fx=0, Fy=0, Fz=-1. Normal = (0,0,1).
    // Fxx = 2, Fyy = 2, Fzz = 0, Fxy = 0, Fxz = 0, Fyz = 0
    // Gaussian Curvature K = (Fxx*Fyy - Fxy^2) / (Fx^2+Fy^2+Fz^2)^2 = (2*2 - 0) / 1^2 = 4
    // Mean Curvature H = ... at origin it should be (Fxx+Fyy)/2 = 2.
    std::vector<Vec3> parabolic_pts;
    parabolic_pts.emplace_back(0, 0, 0);
    parabolic_pts.emplace_back(1, 0, 1);
    parabolic_pts.emplace_back(0, 1, 1);
    parabolic_pts.emplace_back(-1, 0, 1);
    parabolic_pts.emplace_back(0, -1, 1);
    parabolic_pts.emplace_back(0.5, 0.5, 0.5);
    parabolic_pts.emplace_back(-0.5, 0.5, 0.5);
    parabolic_pts.emplace_back(0.5, -0.5, 0.5);
    parabolic_pts.emplace_back(-0.5, -0.5, 0.5);

    QuadraticPlane quad_p;
    if (quad_p.FitParabolicModel(parabolic_pts)) {
        QuadraticPlane::Curvatures cur;
        if (quad_p.ComputeCurvaturesAtPoint(Vec3(0, 0, 0), cur)) {
            ReportInfo("\n[Parabolic Surface z=x²+y²] Evaluation point: (0, 0, 0)");
            ReportInfo("  [K] Gaussian Curvature  : " << cur.gaussian_curvature);
            ReportInfo("  [H] Mean Curvature      : " << cur.mean_curvature);
        }
    }

    return 0;
}
