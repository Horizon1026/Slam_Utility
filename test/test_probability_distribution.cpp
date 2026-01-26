#include "gaussian_mixture_model.h"
#include "slam_log_reporter.h"
#include <random>

using namespace slam_utility;

/**
 * @brief Test Gaussian Mixture Model fitting with synthetic 2D data.
 */
void TestGaussianMixtureModel2D() {
    ReportColorInfo(">> Test GaussianMixtureModel 2D fitting.");

    // 1. Generate synthetic data from 2 Gaussian distributions
    const uint32_t num_points = 2000;
    const float weight1 = 0.3f;
    // Ground truth parameters
    const TVec<float, 2> mu1(5.0f, 2.0f);
    const TVec<float, 2> mu2(-2.0f, -2.0f);
    const float sigma1 = 0.5f;
    const float sigma2 = 3.0f;

    std::vector<TVec<float, 2>> points;
    points.reserve(num_points);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist1(0.0f, sigma1);
    std::normal_distribution<float> dist2(0.0f, sigma2);
    std::uniform_real_distribution<float> weight_dist(0.0f, 1.0f);

    for (uint32_t i = 0; i < num_points; ++i) {
        if (weight_dist(gen) < weight1) {
            points.emplace_back(mu1 + TVec<float, 2>(dist1(gen), dist1(gen)));
        } else {
            points.emplace_back(mu2 + TVec<float, 2>(dist2(gen), dist2(gen)));
        }
    }

    ReportInfo("Generated " << points.size() << " points from 2 Gaussians.");
    ReportInfo("GT Kernel 1: mu=" << LogVec(mu1) << ", sigma=" << sigma1 << ", weight=" << weight1);
    ReportInfo("GT Kernel 2: mu=" << LogVec(mu2) << ", sigma=" << sigma2 << ", weight=" << 1.0f - weight1);

    // 2. Fit GMM
    GaussianMixtureModel<2> gmm;
    gmm.options().kNumberOfNormalDistributions = 2;
    gmm.options().kMaxIterations = 50;
    gmm.options().kMaxLogLikelihoodChangeTolerance = 1e-6f;

    if (gmm.DirectlyFitDistribution(points)) {
        ReportColorInfo("GMM fitting succeeded.");
        const auto &distributions = gmm.distributions();
        const auto &weights = gmm.weights();

        for (uint32_t k = 0; k < distributions.size(); ++k) {
            ReportInfo("Estimated Kernel " << k << ":");
            ReportInfo("  Weight: " << weights[k]);
            ReportInfo("  Mean: " << LogVec(distributions[k].mid_point()));
            ReportInfo("  Covariance:\n" << distributions[k].covariance());
        }
    } else {
        ReportError("GMM fitting failed.");
    }
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test Probability Distribution." RESET_COLOR);
    TestGaussianMixtureModel2D();
    return 0;
}
