#include "radar/coordinates.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace radar {
namespace {

TEST(Coordinates, DegRadConversion) {
    EXPECT_DOUBLE_EQ(degToRad(180.0), kPi);
    EXPECT_DOUBLE_EQ(radToDeg(kPi / 2.0), 90.0);
    EXPECT_DOUBLE_EQ(radToDeg(degToRad(37.5)), 37.5);
}

TEST(Coordinates, PolarToCartesianUsesNavigationConvention) {
    // Azimuth 0 is north (+y), 90 deg is east (+x).
    const Vec2 north = polarToCartesian(1000.0, 0.0);
    EXPECT_NEAR(north[0], 0.0, 1e-9);
    EXPECT_NEAR(north[1], 1000.0, 1e-9);

    const Vec2 east = polarToCartesian(1000.0, degToRad(90.0));
    EXPECT_NEAR(east[0], 1000.0, 1e-9);
    EXPECT_NEAR(east[1], 0.0, 1e-9);

    const Vec2 south = polarToCartesian(1000.0, degToRad(180.0));
    EXPECT_NEAR(south[0], 0.0, 1e-9);
    EXPECT_NEAR(south[1], -1000.0, 1e-9);

    const Vec2 west = polarToCartesian(1000.0, degToRad(270.0));
    EXPECT_NEAR(west[0], -1000.0, 1e-9);
    EXPECT_NEAR(west[1], 0.0, 1e-9);
}

TEST(Coordinates, PolarCartesianRoundTripAllQuadrants) {
    for (double az_deg = 0.0; az_deg < 360.0; az_deg += 7.5) {
        for (double range : {1.0, 250.0, 12345.6, 40000.0}) {
            const double az = degToRad(az_deg);
            const PolarMeasurement back = cartesianToPolar(polarToCartesian(range, az));
            EXPECT_NEAR(back.range_m, range, 1e-9 * range);
            // Compare on the circle so 0 and 2*pi count as the same angle.
            const double diff = normalizeAngle(back.azimuth_rad - az + kPi) - kPi;
            EXPECT_NEAR(diff, 0.0, 1e-12) << "az_deg=" << az_deg;
            EXPECT_GE(back.azimuth_rad, 0.0);
            EXPECT_LT(back.azimuth_rad, kTwoPi);
        }
    }
}

TEST(Coordinates, CartesianToPolarQuadrants) {
    EXPECT_NEAR(radToDeg(cartesianToPolar(Vec2{1.0, 1.0}).azimuth_rad), 45.0, 1e-9);
    EXPECT_NEAR(radToDeg(cartesianToPolar(Vec2{1.0, -1.0}).azimuth_rad), 135.0, 1e-9);
    EXPECT_NEAR(radToDeg(cartesianToPolar(Vec2{-1.0, -1.0}).azimuth_rad), 225.0, 1e-9);
    EXPECT_NEAR(radToDeg(cartesianToPolar(Vec2{-1.0, 1.0}).azimuth_rad), 315.0, 1e-9);
    EXPECT_NEAR(cartesianToPolar(Vec2{3.0, -4.0}).range_m, 5.0, 1e-12);
}

TEST(Coordinates, NormalizeAngle) {
    EXPECT_DOUBLE_EQ(normalizeAngle(0.0), 0.0);
    EXPECT_NEAR(normalizeAngle(-kPi / 2.0), 1.5 * kPi, 1e-12);
    EXPECT_NEAR(normalizeAngle(-kTwoPi - 0.25), kTwoPi - 0.25, 1e-12);
    EXPECT_NEAR(normalizeAngle(kTwoPi + 0.25), 0.25, 1e-12);
    EXPECT_NEAR(normalizeAngle(100.0 * kTwoPi + 1.0), 1.0, 1e-9);
    EXPECT_NEAR(normalizeAngle(-37.0 * kTwoPi + 1.0), 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(normalizeAngle(kTwoPi), 0.0);

    // Always in [0, 2*pi), including values just below zero.
    for (double a : {-1e-17, -1e-12, -kTwoPi, 1e6, -1e6, 7.0 * kPi}) {
        const double n = normalizeAngle(a);
        EXPECT_GE(n, 0.0) << a;
        EXPECT_LT(n, kTwoPi) << a;
    }
}

TEST(Coordinates, AngleInSectorSimple) {
    const double start = degToRad(10.0);
    const double end = degToRad(20.0);
    EXPECT_TRUE(angleInSector(degToRad(15.0), start, end));
    EXPECT_TRUE(angleInSector(start, start, end));
    EXPECT_TRUE(angleInSector(end, start, end));
    EXPECT_FALSE(angleInSector(degToRad(9.0), start, end));
    EXPECT_FALSE(angleInSector(degToRad(21.0), start, end));
    EXPECT_FALSE(angleInSector(degToRad(195.0), start, end));
}

TEST(Coordinates, AngleInSectorWrapsThroughNorth) {
    const double start = degToRad(350.0);
    const double end = degToRad(10.0);
    EXPECT_TRUE(angleInSector(degToRad(355.0), start, end));
    EXPECT_TRUE(angleInSector(0.0, start, end));
    EXPECT_TRUE(angleInSector(degToRad(5.0), start, end));
    EXPECT_TRUE(angleInSector(degToRad(-5.0), start, end));  // unnormalised input
    EXPECT_FALSE(angleInSector(degToRad(180.0), start, end));
    EXPECT_FALSE(angleInSector(degToRad(345.0), start, end));
    EXPECT_FALSE(angleInSector(degToRad(15.0), start, end));
}

TEST(Coordinates, AngleInSectorEndAtTwoPi) {
    // Last sector of a revolution as the simulator produces it.
    const double start = degToRad(355.0);
    const double end = kTwoPi;
    EXPECT_TRUE(angleInSector(degToRad(357.0), start, end));
    EXPECT_FALSE(angleInSector(degToRad(100.0), start, end));
}

TEST(Coordinates, HeadingOf) {
    EXPECT_NEAR(headingOf(0.0, 10.0), 0.0, 1e-12);
    EXPECT_NEAR(radToDeg(headingOf(10.0, 0.0)), 90.0, 1e-9);
    EXPECT_NEAR(radToDeg(headingOf(0.0, -10.0)), 180.0, 1e-9);
    EXPECT_NEAR(radToDeg(headingOf(-10.0, 0.0)), 270.0, 1e-9);
    EXPECT_NEAR(radToDeg(headingOf(-1.0, 1.0)), 315.0, 1e-9);
}

TEST(Coordinates, ConvertMeasurementAtNorth) {
    const double r = 20000.0;
    const double sr = 30.0;
    const double sa = degToRad(0.3);
    const CartesianMeasurement m = convertMeasurement({r, 0.0, 0.0}, sr, sa);

    EXPECT_NEAR(m.z[0], 0.0, 1e-9);
    EXPECT_NEAR(m.z[1], r, 1e-9);
    // Along north, cross-range error is east (x), range error is north (y).
    EXPECT_NEAR(m.R(0, 0), (r * sa) * (r * sa), 1e-6);
    EXPECT_NEAR(m.R(1, 1), sr * sr, 1e-6);
    EXPECT_NEAR(m.R(0, 1), 0.0, 1e-6);
    EXPECT_NEAR(m.R(1, 0), 0.0, 1e-6);
}

TEST(Coordinates, ConvertMeasurementAtEast) {
    const double r = 10000.0;
    const double sr = 50.0;
    const double sa = degToRad(1.0);
    const CartesianMeasurement m = convertMeasurement({r, degToRad(90.0), 0.0}, sr, sa);
    EXPECT_NEAR(m.R(0, 0), sr * sr, 1e-6);
    EXPECT_NEAR(m.R(1, 1), (r * sa) * (r * sa), 1e-6);
    EXPECT_NEAR(m.R(0, 1), 0.0, 1e-6);
}

TEST(Coordinates, ConvertMeasurementCovarianceIsSymmetricWithPolarEigenvalues) {
    const double sr = 30.0;
    const double sa = degToRad(0.3);
    for (double az_deg = 0.0; az_deg < 360.0; az_deg += 22.5) {
        const double r = 15000.0;
        const CartesianMeasurement m = convertMeasurement({r, degToRad(az_deg), 0.0}, sr, sa);
        EXPECT_DOUBLE_EQ(m.R(0, 1), m.R(1, 0));
        // A rotation of diag(sr^2, (r sa)^2): trace and determinant are invariant.
        const double lr = sr * sr;
        const double la = (r * sa) * (r * sa);
        EXPECT_NEAR(m.R(0, 0) + m.R(1, 1), lr + la, 1e-6);
        const double det = m.R(0, 0) * m.R(1, 1) - m.R(0, 1) * m.R(1, 0);
        EXPECT_NEAR(det, lr * la, 1e-6 * lr * la);
        EXPECT_GT(m.R(0, 0), 0.0);
        EXPECT_GT(m.R(1, 1), 0.0);
    }
}

TEST(Coordinates, ConvertMeasurementDiagonalCovarianceRotates) {
    // At 45 deg the range direction is (1, 1)/sqrt(2): variance along it is sr^2.
    const double sr = 100.0;
    const double sa = 0.01;
    const double r = 5000.0;
    const CartesianMeasurement m = convertMeasurement({r, degToRad(45.0), 0.0}, sr, sa);
    const double half_sum = 0.5 * (sr * sr + (r * sa) * (r * sa));
    const double half_diff = 0.5 * (sr * sr - (r * sa) * (r * sa));
    EXPECT_NEAR(m.R(0, 0), half_sum, 1e-6);
    EXPECT_NEAR(m.R(1, 1), half_sum, 1e-6);
    EXPECT_NEAR(m.R(0, 1), half_diff, 1e-6);
}

}  // namespace
}  // namespace radar
