#include "radar/coordinates.hpp"

#include <cmath>

namespace radar {

double normalizeAngle(double rad) {
    double a = std::fmod(rad, kTwoPi);
    if (a < 0.0) {
        a += kTwoPi;
    }
    // fmod can return exactly 2*pi after the correction due to rounding.
    return a >= kTwoPi ? 0.0 : a;
}

bool angleInSector(double angle, double start, double end) {
    const double a = normalizeAngle(angle - start);
    const double width = normalizeAngle(end - start);
    return a <= width;
}

Vec2 polarToCartesian(double range_m, double azimuth_rad) {
    return Vec2{range_m * std::sin(azimuth_rad), range_m * std::cos(azimuth_rad)};
}

PolarMeasurement cartesianToPolar(const Vec2& position) {
    PolarMeasurement m;
    m.range_m = std::hypot(position[0], position[1]);
    m.azimuth_rad = normalizeAngle(std::atan2(position[0], position[1]));
    return m;
}

double headingOf(double vx, double vy) {
    return normalizeAngle(std::atan2(vx, vy));
}

CartesianMeasurement convertMeasurement(const PolarMeasurement& m, double range_sigma_m,
                                        double azimuth_sigma_rad) {
    const double s = std::sin(m.azimuth_rad);
    const double c = std::cos(m.azimuth_rad);
    const double r = m.range_m;

    // Jacobian of (x, y) = (r sin a, r cos a) with respect to (r, a).
    const Mat2 J{s, r * c,  //
                 c, -r * s};
    const Mat2 polar_cov{range_sigma_m * range_sigma_m, 0.0,  //
                         0.0, azimuth_sigma_rad * azimuth_sigma_rad};

    return CartesianMeasurement{polarToCartesian(r, m.azimuth_rad), J * polar_cov * J.transposed()};
}

}  // namespace radar
