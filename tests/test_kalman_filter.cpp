#include "radar/kalman_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace radar {
namespace {

Mat4 diag4(double a, double b, double c, double d) {
    Mat4 m;
    m(0, 0) = a;
    m(1, 1) = b;
    m(2, 2) = c;
    m(3, 3) = d;
    return m;
}

double maxAbsDiff(const Mat4& a, const Mat4& b) {
    double worst = 0.0;
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            worst = std::max(worst, std::abs(a(r, c) - b(r, c)));
        }
    }
    return worst;
}

double maxAsymmetry(const Mat4& m) {
    double worst = 0.0;
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = r + 1; c < 4; ++c) {
            worst = std::max(worst, std::abs(m(r, c) - m(c, r)));
        }
    }
    return worst;
}

/// Lower Cholesky factor of a symmetric positive definite 2x2 matrix.
Mat2 cholesky(const Mat2& m) {
    const double l11 = std::sqrt(m(0, 0));
    const double l21 = m(1, 0) / l11;
    const double l22 = std::sqrt(m(1, 1) - l21 * l21);
    return Mat2{l11, 0.0, l21, l22};
}

Vec2 sampleGaussian(const Mat2& chol, std::mt19937& rng) {
    std::normal_distribution<double> n(0.0, 1.0);
    const Vec2 u{n(rng), n(rng)};
    return chol * u;
}

/// Draws process noise exactly distributed as N(0, Q(dt)) for the CV model.
/// Q is block-diagonal per axis over (position, velocity).
Vec4 sampleProcessNoise(double dt, double q, std::mt19937& rng) {
    const Mat2 axis_q{q * dt * dt * dt / 3.0, q * dt * dt / 2.0,  //
                      q * dt * dt / 2.0, q * dt};
    const Mat2 L = cholesky(axis_q);
    const Vec2 wx = sampleGaussian(L, rng);
    const Vec2 wy = sampleGaussian(L, rng);
    return Vec4{wx[0], wy[0], wx[1], wy[1]};
}

TEST(KalmanFilter, TransitionMatrixIsConstantVelocity) {
    const Mat4 F = KalmanFilter::transitionMatrix(2.5);
    const Vec4 x{100.0, -50.0, 10.0, 4.0};
    const Vec4 out = F * x;
    EXPECT_DOUBLE_EQ(out[0], 125.0);
    EXPECT_DOUBLE_EQ(out[1], -40.0);
    EXPECT_DOUBLE_EQ(out[2], 10.0);
    EXPECT_DOUBLE_EQ(out[3], 4.0);
}

TEST(KalmanFilter, ProcessNoiseIsSymmetricAndScalesWithQ) {
    const Mat4 Q = KalmanFilter::processNoise(2.0, 5.0);
    EXPECT_EQ(maxAsymmetry(Q), 0.0);
    EXPECT_DOUBLE_EQ(Q(0, 0), 5.0 * 8.0 / 3.0);
    EXPECT_DOUBLE_EQ(Q(0, 2), 5.0 * 4.0 / 2.0);
    EXPECT_DOUBLE_EQ(Q(2, 2), 5.0 * 2.0);
    EXPECT_EQ(Q(0, 1), 0.0);  // axes are independent
    EXPECT_EQ(maxAbsDiff(KalmanFilter::processNoise(2.0, 10.0), Q * 2.0), 0.0);
}

TEST(KalmanFilter, PredictMovesStateAndGrowsCovariance) {
    const Vec4 x0{1000.0, 2000.0, 100.0, -50.0};
    const Mat4 P0 = diag4(100.0, 100.0, 25.0, 25.0);
    KalmanFilter kf(x0, P0, 1.0);

    kf.predict(3.0);
    EXPECT_DOUBLE_EQ(kf.state()[0], 1300.0);
    EXPECT_DOUBLE_EQ(kf.state()[1], 1850.0);
    EXPECT_DOUBLE_EQ(kf.velocity()[0], 100.0);
    EXPECT_DOUBLE_EQ(kf.velocity()[1], -50.0);

    const Mat4& P = kf.covariance();
    // P = F P0 F^T + Q: position variance gains dt^2 * var(v) plus q dt^3 / 3.
    EXPECT_NEAR(P(0, 0), 100.0 + 9.0 * 25.0 + 9.0, 1e-9);
    EXPECT_NEAR(P(2, 2), 25.0 + 3.0, 1e-9);
    EXPECT_GT(P(0, 2), 0.0);  // position and velocity become correlated
    EXPECT_GT(P(1, 1), P0(1, 1));
    EXPECT_GT(P(3, 3), P0(3, 3));
}

TEST(KalmanFilter, NonPositiveDtIsNoOp) {
    const Vec4 x0{1.0, 2.0, 3.0, 4.0};
    const Mat4 P0 = diag4(1.0, 1.0, 1.0, 1.0);
    KalmanFilter kf(x0, P0, 1.0);
    kf.predict(0.0);
    kf.predict(-1.0);
    EXPECT_EQ(kf.state(), x0);
    EXPECT_EQ(kf.covariance(), P0);
}

TEST(KalmanFilter, ProcessNoiseComposesAcrossSteps) {
    // Continuous white-noise acceleration: two half steps == one full step.
    Mat4 P0 = diag4(400.0, 900.0, 64.0, 36.0);
    P0(0, 2) = P0(2, 0) = 30.0;
    P0(0, 1) = P0(1, 0) = -50.0;
    const Vec4 x0{0.0, 0.0, 200.0, -100.0};
    const double q = 20.0;

    KalmanFilter one(x0, P0, q);
    one.predict(2.0);

    KalmanFilter two(x0, P0, q);
    two.predict(1.0);
    two.predict(1.0);

    KalmanFilter many(x0, P0, q);
    for (int i = 0; i < 72; ++i) {
        many.predict(2.0 / 72.0);
    }

    const double scale = one.covariance()(0, 0);
    EXPECT_LT(maxAbsDiff(one.covariance(), two.covariance()), 1e-9 * scale);
    EXPECT_LT(maxAbsDiff(one.covariance(), many.covariance()), 1e-9 * scale);
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(one.state()[i], many.state()[i], 1e-9);
    }
}

TEST(KalmanFilter, UpdatePullsTowardsMeasurementAndShrinksCovariance) {
    KalmanFilter kf(Vec4{0.0, 0.0, 0.0, 0.0}, diag4(100.0, 100.0, 10.0, 10.0), 1.0);
    const Mat2 R{100.0, 0.0, 0.0, 100.0};
    ASSERT_TRUE(kf.update(Vec2{10.0, -20.0}, R));
    // Equal prior and measurement variance: estimate lands half way.
    EXPECT_NEAR(kf.position()[0], 5.0, 1e-9);
    EXPECT_NEAR(kf.position()[1], -10.0, 1e-9);
    EXPECT_NEAR(kf.covariance()(0, 0), 50.0, 1e-9);
    EXPECT_NEAR(kf.covariance()(1, 1), 50.0, 1e-9);
}

TEST(KalmanFilter, UpdateWithSingularInnovationIsRejected) {
    const Vec4 x0{1.0, 2.0, 3.0, 4.0};
    KalmanFilter kf(x0, Mat4::zero(), 0.0);
    EXPECT_FALSE(kf.update(Vec2{5.0, 5.0}, Mat2::zero()));
    EXPECT_EQ(kf.state(), x0);
    EXPECT_EQ(kf.covariance(), Mat4::zero());
}

TEST(KalmanFilter, GatingDistance) {
    KalmanFilter kf(Vec4{1000.0, 1000.0, 0.0, 0.0}, diag4(100.0, 100.0, 1.0, 1.0), 1.0);
    const Mat2 R{300.0, 0.0, 0.0, 300.0};  // S = diag(400, 400)

    const Innovation exact = kf.innovation(Vec2{1000.0, 1000.0}, R);
    EXPECT_NEAR(exact.mahalanobis_sq, 0.0, 1e-12);

    const Innovation near = kf.innovation(Vec2{1020.0, 1000.0}, R);  // 1 sigma
    EXPECT_NEAR(near.residual[0], 20.0, 1e-12);
    EXPECT_NEAR(near.covariance(0, 0), 400.0, 1e-12);
    EXPECT_NEAR(near.mahalanobis_sq, 1.0, 1e-12);
    EXPECT_LT(near.mahalanobis_sq, 13.82);

    const Innovation far = kf.innovation(Vec2{1500.0, 800.0}, R);
    EXPECT_NEAR(far.mahalanobis_sq, (500.0 * 500.0 + 200.0 * 200.0) / 400.0, 1e-9);
    EXPECT_GT(far.mahalanobis_sq, 13.82);
}

TEST(KalmanFilter, ConvergesOnNoisyConstantVelocityTarget) {
    std::mt19937 rng(1234);
    const double sigma = 100.0;
    const Mat2 R{sigma * sigma, 0.0, 0.0, sigma * sigma};
    const Mat2 L = cholesky(R);
    const double dt = 2.0;

    Vec4 truth{-20000.0, 5000.0, 220.0, -80.0};
    const Vec2 first = Vec2{truth[0], truth[1]} + sampleGaussian(L, rng);
    Mat4 P0 = diag4(R(0, 0), R(1, 1), 300.0 * 300.0, 300.0 * 300.0);
    KalmanFilter kf(Vec4{first[0], first[1], 0.0, 0.0}, P0, 0.05);

    double pos_sq = 0.0;
    double vel_sq = 0.0;
    int samples = 0;
    for (int k = 1; k <= 150; ++k) {
        truth = KalmanFilter::transitionMatrix(dt) * truth;
        kf.predict(dt);
        const Vec2 z = Vec2{truth[0], truth[1]} + sampleGaussian(L, rng);
        ASSERT_TRUE(kf.update(z, R));
        EXPECT_LT(maxAsymmetry(kf.covariance()), 1e-6 * kf.covariance()(0, 0));
        if (k > 30) {
            const Vec4 e = kf.state() - truth;
            pos_sq += e[0] * e[0] + e[1] * e[1];
            vel_sq += e[2] * e[2] + e[3] * e[3];
            ++samples;
        }
    }
    const double pos_rmse = std::sqrt(pos_sq / samples);
    const double vel_rmse = std::sqrt(vel_sq / samples);
    // Single-measurement 2D error would be sigma * sqrt(2) ~ 141 m.
    EXPECT_LT(pos_rmse, 0.5 * sigma);
    EXPECT_LT(vel_rmse, 5.0);
    EXPECT_NEAR(kf.velocity()[0], 220.0, 5.0);
    EXPECT_NEAR(kf.velocity()[1], -80.0, 5.0);
}

TEST(KalmanFilter, CovarianceStaysSymmetricPositiveDefinite) {
    std::mt19937 rng(77);
    // Strongly correlated measurement noise, as for a plot far off-axis.
    const Mat2 R{5000.0, 4000.0, 4000.0, 5000.0};
    const Mat2 L = cholesky(R);
    KalmanFilter kf(Vec4{0.0, 0.0, 0.0, 0.0}, diag4(R(0, 0), R(1, 1), 4e4, 4e4), 20.0);
    std::uniform_real_distribution<double> step(0.01, 2.0);
    for (int k = 0; k < 500; ++k) {
        kf.predict(step(rng));
        ASSERT_TRUE(kf.update(sampleGaussian(L, rng), R));
        const Mat4& P = kf.covariance();
        ASSERT_LT(maxAsymmetry(P), 1e-9 * (1.0 + P(0, 0)));
        for (std::size_t i = 0; i < 4; ++i) {
            ASSERT_GT(P(i, i), 0.0);
        }
        // 2x2 leading minor must stay positive.
        ASSERT_GT(P(0, 0) * P(1, 1) - P(0, 1) * P(1, 0), 0.0);
    }
}

TEST(KalmanFilter, NormalisedEstimationErrorIsConsistent) {
    // Monte Carlo NEES: with truth generated from the filter's own model the
    // average of e^T P^-1 e over runs must be close to the state dimension.
    std::mt19937 rng(2024);
    const double q = 2.0;
    const double dt = 1.0;
    const Mat2 R{900.0, 300.0, 300.0, 1600.0};
    const Mat2 L = cholesky(R);
    const Mat4 P0 = diag4(900.0, 1600.0, 400.0, 400.0);
    std::normal_distribution<double> n(0.0, 1.0);

    constexpr int kRuns = 40;
    constexpr int kSteps = 60;
    double nees_sum = 0.0;
    int samples = 0;

    for (int run = 0; run < kRuns; ++run) {
        Vec4 truth{0.0, 0.0, 150.0, 50.0};
        // Initial estimate drawn from N(truth, P0) (P0 is diagonal).
        Vec4 x0 = truth;
        for (std::size_t i = 0; i < 4; ++i) {
            x0[i] += std::sqrt(P0(i, i)) * n(rng);
        }
        KalmanFilter kf(x0, P0, q);

        for (int k = 0; k < kSteps; ++k) {
            truth = KalmanFilter::transitionMatrix(dt) * truth + sampleProcessNoise(dt, q, rng);
            kf.predict(dt);
            ASSERT_TRUE(kf.update(Vec2{truth[0], truth[1]} + sampleGaussian(L, rng), R));

            // NEES on the position block, chi-square with 2 dof.
            const Vec4 e = kf.state() - truth;
            const Vec2 ep{e[0], e[1]};
            const Mat2 Pp{kf.covariance()(0, 0), kf.covariance()(0, 1),  //
                          kf.covariance()(1, 0), kf.covariance()(1, 1)};
            const auto Pinv = inverse(Pp);
            ASSERT_TRUE(Pinv.has_value());
            nees_sum += (ep.transposed() * *Pinv * ep)(0, 0);
            ++samples;
        }
    }
    const double mean = nees_sum / samples;
    // E[chi2(2)] = 2. With 2400 (correlated) samples, +-20 % is a loose bound.
    EXPECT_GT(mean, 1.6);
    EXPECT_LT(mean, 2.4);
}

}  // namespace
}  // namespace radar
