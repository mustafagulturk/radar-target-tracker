#pragma once

#include "radar/matrix.hpp"
#include "radar/types.hpp"

namespace radar {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

constexpr double degToRad(double deg) {
    return deg * kPi / 180.0;
}
constexpr double radToDeg(double rad) {
    return rad * 180.0 / kPi;
}

/// Wraps any angle into [0, 2*pi).
double normalizeAngle(double rad);

/// True when `angle` lies on the clockwise arc from `start` to `end`.
/// Handles sectors that wrap through north (e.g. 350 deg -> 10 deg).
bool angleInSector(double angle, double start, double end);

/// East/north position relative to the radar, metres.
/// x = range * sin(azimuth), y = range * cos(azimuth).
Vec2 polarToCartesian(double range_m, double azimuth_rad);

/// Inverse of polarToCartesian; azimuth is returned in [0, 2*pi).
PolarMeasurement cartesianToPolar(const Vec2& position);

/// Bearing of a velocity vector (clockwise from north) in [0, 2*pi).
double headingOf(double vx, double vy);

/// Position measurement converted to Cartesian space together with the
/// linearised measurement covariance R = J * diag(sr^2, sa^2) * J^T.
struct CartesianMeasurement {
    Vec2 z;
    Mat2 R;
};

CartesianMeasurement convertMeasurement(const PolarMeasurement& m, double range_sigma_m,
                                        double azimuth_sigma_rad);

}  // namespace radar
