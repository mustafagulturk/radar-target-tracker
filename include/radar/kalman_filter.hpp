#pragma once

#include "radar/matrix.hpp"

namespace radar {

/// Innovation (measurement residual) statistics used for gating.
struct Innovation {
    Vec2 residual;                ///< y = z - H x
    Mat2 covariance;              ///< S = H P H^T + R
    double mahalanobis_sq = 0.0;  ///< d^2 = y^T S^-1 y (chi-square, 2 dof)
};

/// Linear Kalman filter with a 2D constant-velocity motion model.
///
/// State vector: [x, y, vx, vy] (metres, metres per second, east/north).
/// Process noise follows the continuous white-noise-acceleration model, so
/// predicting in several small steps gives the same covariance as one large
/// step. That property matters because the tracker predicts every track at
/// every incoming sector, not once per antenna revolution.
class KalmanFilter {
public:
    /// @param accel_noise_psd Power spectral density q of the acceleration
    ///        noise in m^2/s^3. Larger values let the filter follow manoeuvres
    ///        faster at the cost of noisier estimates.
    KalmanFilter(const Vec4& initial_state, const Mat4& initial_covariance, double accel_noise_psd);

    /// Propagates state and covariance forward by `dt_s` seconds.
    void predict(double dt_s);

    /// Residual and its covariance for a position measurement `z` with
    /// noise covariance `R`, evaluated against the current (predicted) state.
    Innovation innovation(const Vec2& z, const Mat2& R) const;

    /// Corrects the state with a position measurement. Uses the Joseph form
    /// of the covariance update to keep P symmetric positive definite.
    /// Returns false (and leaves the filter untouched) if S is singular.
    bool update(const Vec2& z, const Mat2& R);

    const Vec4& state() const { return x_; }
    const Mat4& covariance() const { return P_; }

    Vec2 position() const { return Vec2{x_[0], x_[1]}; }
    Vec2 velocity() const { return Vec2{x_[2], x_[3]}; }

    static Mat4 transitionMatrix(double dt_s);
    static Mat4 processNoise(double dt_s, double accel_noise_psd);

private:
    Vec4 x_;
    Mat4 P_;
    double q_;
};

}  // namespace radar
