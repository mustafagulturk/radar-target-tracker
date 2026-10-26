#include <gtest/gtest.h>

#include <cmath>

#include "radar/coordinates.hpp"
#include "radar/sim/radar_sensor.hpp"
#include "radar/sim/scenario.hpp"
#include "radar/sim/target.hpp"
#include "sim_harness.hpp"

namespace radar::sim {
namespace {

using test::makeTarget;

TEST(Target, StraightFlight) {
    Target t;
    t.x = 100.0;
    t.y = -200.0;
    t.vx = 50.0;
    t.vy = 10.0;
    t.advance(4.0);
    EXPECT_DOUBLE_EQ(t.x, 300.0);
    EXPECT_DOUBLE_EQ(t.y, -160.0);
    EXPECT_DOUBLE_EQ(t.vx, 50.0);
    EXPECT_DOUBLE_EQ(t.vy, 10.0);
    EXPECT_NEAR(t.speed(), std::hypot(50.0, 10.0), 1e-12);
}

TEST(Target, PositiveTurnRateTurnsClockwise) {
    Target t = makeTarget(10.0, 0.0, 200.0, 0.0, 3.0);  // heading north
    t.advance(10.0);                                    // 30 deg of turn
    EXPECT_NEAR(radToDeg(headingOf(t.vx, t.vy)), 30.0, 1e-9);
    EXPECT_GT(t.x, 0.0);  // drifted east
}

TEST(Target, CoordinatedTurnPreservesSpeedAndTurnRadius) {
    const double speed = 280.0;
    const double rate_deg = 3.0;
    Target t = makeTarget(18.0, 150.0, speed, 0.0, rate_deg);
    const double x0 = t.x;
    const double y0 = t.y;
    // Turn centre lies one radius to starboard (east, when heading north).
    const double radius = speed / degToRad(rate_deg);
    const double cx = x0 + radius;
    const double cy = y0;

    for (int i = 0; i < 1000; ++i) {
        t.advance(0.037);
        ASSERT_NEAR(t.speed(), speed, 1e-6);
        ASSERT_NEAR(std::hypot(t.x - cx, t.y - cy), radius, 1e-6);
    }
}

TEST(Target, FullTurnReturnsToStart) {
    const double rate_deg = 4.5;
    Target t = makeTarget(12.0, 300.0, 240.0, 75.0, rate_deg);
    const Target start = t;
    const double period_s = 360.0 / rate_deg;  // 80 s
    // Uneven steps exercise the exact integration.
    const int steps = 7;
    for (int i = 0; i < steps; ++i) {
        t.advance(period_s / steps);
    }
    EXPECT_NEAR(t.x, start.x, 1e-6);
    EXPECT_NEAR(t.y, start.y, 1e-6);
    EXPECT_NEAR(t.vx, start.vx, 1e-9);
    EXPECT_NEAR(t.vy, start.vy, 1e-9);

    // A half turn reverses the velocity.
    t.advance(period_s / 2.0);
    EXPECT_NEAR(t.vx, -start.vx, 1e-9);
    EXPECT_NEAR(t.vy, -start.vy, 1e-9);
}

TEST(RadarSensor, SameSeedGivesIdenticalOutput) {
    SensorConfig cfg;
    const Scenario scenario = Scenario::standard(cfg.max_range_m, 4, 42);
    RadarSensor a(cfg, 7);
    RadarSensor b(cfg, 7);
    RadarSensor c(cfg, 8);
    bool any_difference = false;
    const double width = kTwoPi / cfg.sectors_per_revolution;
    for (int s = 0; s < 3 * cfg.sectors_per_revolution; ++s) {
        const auto t_us = static_cast<std::uint64_t>(s) * 27'778;
        const double start = (s % cfg.sectors_per_revolution) * width;
        const ScanBatch ba = a.scanSector(scenario.targets(), t_us, start, start + width);
        const ScanBatch bb = b.scanSector(scenario.targets(), t_us, start, start + width);
        const ScanBatch bc = c.scanSector(scenario.targets(), t_us, start, start + width);
        ASSERT_EQ(ba.sequence, bb.sequence);
        ASSERT_EQ(ba.plots.size(), bb.plots.size());
        for (std::size_t i = 0; i < ba.plots.size(); ++i) {
            ASSERT_EQ(ba.plots[i].range_m, bb.plots[i].range_m);
            ASSERT_EQ(ba.plots[i].azimuth_rad, bb.plots[i].azimuth_rad);
            ASSERT_EQ(ba.plots[i].amplitude_db, bb.plots[i].amplitude_db);
        }
        if (ba.plots.size() != bc.plots.size() ||
            (!ba.plots.empty() && ba.plots[0].range_m != bc.plots[0].range_m)) {
            any_difference = true;
        }
    }
    EXPECT_TRUE(any_difference) << "different seeds should give different noise";
}

TEST(RadarSensor, SequenceNumbersIncrease) {
    RadarSensor sensor(SensorConfig{}, 1);
    for (std::uint32_t i = 0; i < 10; ++i) {
        EXPECT_EQ(sensor.scanSector({}, i, 0.0, 0.1).sequence, i);
    }
}

TEST(RadarSensor, DetectionProbabilityIsHonoured) {
    SensorConfig cfg;
    cfg.detection_probability = 0.7;
    cfg.clutter_per_revolution = 0.0;
    RadarSensor sensor(cfg, 2024);
    const std::vector<Target> targets{makeTarget(15.0, 2.5, 0.0, 0.0)};

    constexpr int kTrials = 4000;
    int detections = 0;
    for (int i = 0; i < kTrials; ++i) {
        detections += static_cast<int>(
            sensor.scanSector(targets, static_cast<std::uint64_t>(i), 0.0, degToRad(5.0))
                .plots.size());
    }
    const double pd = static_cast<double>(detections) / kTrials;
    // Binomial standard deviation is ~0.007; allow > 4 sigma.
    EXPECT_NEAR(pd, 0.7, 0.03);
}

TEST(RadarSensor, PlotNoiseMatchesConfiguredSigma) {
    SensorConfig cfg;
    cfg.detection_probability = 1.0;
    cfg.clutter_per_revolution = 0.0;
    cfg.range_sigma_m = 40.0;
    RadarSensor sensor(cfg, 9);
    const std::vector<Target> targets{makeTarget(20.0, 2.5, 0.0, 0.0)};
    double sum = 0.0;
    double sum_sq = 0.0;
    constexpr int kTrials = 3000;
    for (int i = 0; i < kTrials; ++i) {
        const ScanBatch b =
            sensor.scanSector(targets, static_cast<std::uint64_t>(i), 0.0, degToRad(5.0));
        ASSERT_EQ(b.plots.size(), 1U);
        const double e = b.plots[0].range_m - 20000.0;
        sum += e;
        sum_sq += e * e;
    }
    const double mean = sum / kTrials;
    const double sigma = std::sqrt(sum_sq / kTrials - mean * mean);
    EXPECT_NEAR(mean, 0.0, 4.0);
    EXPECT_NEAR(sigma, 40.0, 3.0);
}

TEST(RadarSensor, TargetsOutsideSectorOrRangeAreNotReported) {
    SensorConfig cfg;
    cfg.detection_probability = 1.0;
    cfg.clutter_per_revolution = 0.0;
    RadarSensor sensor(cfg, 4);
    const std::vector<Target> targets{makeTarget(15.0, 90.0, 0.0, 0.0),
                                      makeTarget(50.0, 2.0, 0.0, 0.0)};  // beyond 40 km
    EXPECT_TRUE(sensor.scanSector(targets, 0, 0.0, degToRad(5.0)).plots.empty());
    EXPECT_EQ(sensor.scanSector(targets, 0, degToRad(85.0), degToRad(95.0)).plots.size(), 1U);
}

TEST(RadarSensor, ClutterStaysInsideSectorIncludingWrap) {
    SensorConfig cfg;
    cfg.clutter_per_revolution = 2000.0;
    RadarSensor sensor(cfg, 77);
    struct Sector {
        double start;
        double end;
    };
    const Sector sectors[] = {{degToRad(355.0), degToRad(5.0)},
                              {degToRad(355.0), kTwoPi},
                              {0.0, degToRad(5.0)},
                              {degToRad(120.0), degToRad(125.0)}};
    for (const Sector& s : sectors) {
        const ScanBatch b = sensor.scanSector({}, 0, s.start, s.end);
        EXPECT_FALSE(b.plots.empty());
        for (std::size_t i = 0; i < b.plots.size(); ++i) {
            const PolarMeasurement& p = b.plots[i];
            EXPECT_TRUE(angleInSector(p.azimuth_rad, s.start, s.end)) << p.azimuth_rad;
            EXPECT_GE(p.range_m, 0.0);
            EXPECT_LE(p.range_m, cfg.max_range_m);
            if (i > 0) {
                EXPECT_LE(b.plots[i - 1].range_m, p.range_m);  // sorted by range
            }
        }
    }
}

TEST(RadarSensor, ClutterRateMatchesConfiguration) {
    SensorConfig cfg;
    cfg.clutter_per_revolution = 20.0;
    test::SimHarness sim(cfg, 123);
    constexpr int kRevolutions = 200;
    std::size_t plots = 0;
    for (int rev = 0; rev < kRevolutions; ++rev) {
        for (const ScanBatch& b : sim.nextRevolution()) {
            plots += b.plots.size();
        }
    }
    const double per_rev = static_cast<double>(plots) / kRevolutions;
    EXPECT_NEAR(per_rev, 20.0, 1.5);  // Poisson sd over 200 scans ~ 0.32
}

TEST(Scenario, StandardIsDeterministicAndPopulated) {
    const Scenario a = Scenario::standard(40000.0, 6, 42);
    const Scenario b = Scenario::standard(40000.0, 6, 42);
    ASSERT_EQ(a.targets().size(), 11U);
    ASSERT_EQ(a.targets().size(), b.targets().size());
    for (std::size_t i = 0; i < a.targets().size(); ++i) {
        EXPECT_EQ(a.targets()[i].x, b.targets()[i].x);
        EXPECT_EQ(a.targets()[i].vy, b.targets()[i].vy);
        EXPECT_LE(a.targets()[i].range(), 40000.0);
    }
}

TEST(Scenario, ReplacesTargetsThatLeaveCoverage) {
    Scenario s = Scenario::random(0, 10000.0, 1);
    Target outbound = makeTarget(9.9, 45.0, 300.0, 45.0);
    s.addTarget(outbound);
    const std::uint32_t first_id = s.targets()[0].id;
    for (int i = 0; i < 10; ++i) {
        s.advance(1.0);
    }
    ASSERT_EQ(s.targets().size(), 1U);
    EXPECT_NE(s.targets()[0].id, first_id);
    EXPECT_LE(s.targets()[0].range(), 10000.0 * 1.05);
}

}  // namespace
}  // namespace radar::sim
