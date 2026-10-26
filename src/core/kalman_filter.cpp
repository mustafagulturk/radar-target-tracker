#include "radar/kalman_filter.hpp"

namespace radar {

namespace {

// Measurement model: the radar observes position only.
const Matrix<2, 4> kH{1.0, 0.0, 0.0, 0.0,  //
                      0.0, 1.0, 0.0, 0.0};

}  // namespace

KalmanFilter::KalmanFilter(const Vec4& initial_state, const Mat4& initial_covariance,
                           double accel_noise_psd)
    : x_(initial_state), P_(initial_covariance), q_(accel_noise_psd) {}

Mat4 KalmanFilter::transitionMatrix(double dt_s) {
    Mat4 F = Mat4::identity();
    F(0, 2) = dt_s;
    F(1, 3) = dt_s;
    return F;
}

Mat4 KalmanFilter::processNoise(double dt_s, double accel_noise_psd) {
    const double dt2 = dt_s * dt_s;
    const double dt3 = dt2 * dt_s;
    const double pp = accel_noise_psd * dt3 / 3.0;
    const double pv = accel_noise_psd * dt2 / 2.0;
    const double vv = accel_noise_psd * dt_s;

    Mat4 Q;
    Q(0, 0) = pp;
    Q(0, 2) = pv;
    Q(2, 0) = pv;
    Q(2, 2) = vv;
    Q(1, 1) = pp;
    Q(1, 3) = pv;
    Q(3, 1) = pv;
    Q(3, 3) = vv;
    return Q;
}

void KalmanFilter::predict(double dt_s) {
    if (dt_s <= 0.0) {
        return;
    }
    const Mat4 F = transitionMatrix(dt_s);
    x_ = F * x_;
    P_ = F * P_ * F.transposed() + processNoise(dt_s, q_);
}

Innovation KalmanFilter::innovation(const Vec2& z, const Mat2& R) const {
    Innovation inn;
    inn.residual = z - kH * x_;
    inn.covariance = kH * P_ * kH.transposed() + R;
    if (const auto S_inv = inverse(inn.covariance)) {
        inn.mahalanobis_sq = (inn.residual.transposed() * *S_inv * inn.residual)(0, 0);
    } else {
        inn.mahalanobis_sq = 1e18;
    }
    return inn;
}

bool KalmanFilter::update(const Vec2& z, const Mat2& R) {
    const Innovation inn = innovation(z, R);
    const auto S_inv = inverse(inn.covariance);
    if (!S_inv) {
        return false;
    }

    const Matrix<4, 2> K = P_ * kH.transposed() * *S_inv;
    x_ += K * inn.residual;

    const Mat4 I_KH = Mat4::identity() - K * kH;
    P_ = I_KH * P_ * I_KH.transposed() + K * R * K.transposed();
    return true;
}

}  // namespace radar
