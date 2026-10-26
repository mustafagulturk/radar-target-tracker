#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "radar/coordinates.hpp"
#include "radar/sim/radar_sensor.hpp"
#include "radar/sim/scenario.hpp"
#include "radar/types.hpp"

namespace radar::test {

/// Drives a Scenario and a RadarSensor sector by sector, exactly like
/// apps/radar_simulator: advance the targets by one sector dwell, then sweep
/// the next sector and report what the beam saw.
class SimHarness {
public:
    SimHarness(sim::SensorConfig config, std::uint32_t seed)
        : config_(config),
          sensor_(config, seed),
          scenario_(sim::Scenario::random(0, config.max_range_m, seed)),
          sector_dt_s_(config.rotation_period_s / config.sectors_per_revolution),
          sector_width_rad_(kTwoPi / config.sectors_per_revolution) {}

    void addTarget(const sim::Target& target) { scenario_.addTarget(target); }

    /// When false, the sensor no longer sees any target (e.g. it landed),
    /// while clutter keeps coming.
    void setTargetsVisible(bool visible) { targets_visible_ = visible; }

    ScanBatch nextSector() {
        scenario_.advance(sector_dt_s_);
        ++step_;
        const auto time_us =
            static_cast<std::uint64_t>(static_cast<double>(step_) * sector_dt_s_ * 1e6);
        const int sector = static_cast<int>((step_ - 1) % config_.sectors_per_revolution);
        const double start = sector * sector_width_rad_;
        static const std::vector<sim::Target> kNone;
        return sensor_.scanSector(targets_visible_ ? scenario_.targets() : kNone, time_us, start,
                                  start + sector_width_rad_);
    }

    /// Batches covering one full antenna revolution.
    std::vector<ScanBatch> nextRevolution() {
        std::vector<ScanBatch> batches;
        batches.reserve(static_cast<std::size_t>(config_.sectors_per_revolution));
        for (int i = 0; i < config_.sectors_per_revolution; ++i) {
            batches.push_back(nextSector());
        }
        return batches;
    }

    const std::vector<sim::Target>& targets() const { return scenario_.targets(); }
    const sim::SensorConfig& config() const { return config_; }
    double timeS() const { return static_cast<double>(step_) * sector_dt_s_; }

private:
    sim::SensorConfig config_;
    sim::RadarSensor sensor_;
    sim::Scenario scenario_;
    double sector_dt_s_;
    double sector_width_rad_;
    long step_ = 0;
    bool targets_visible_ = true;
};

/// Builds a target from range/bearing (km, deg) and speed/heading (m/s, deg).
inline sim::Target makeTarget(double range_km, double bearing_deg, double speed_mps,
                              double heading_deg, double turn_deg_per_s = 0.0) {
    sim::Target t;
    const Vec2 p = polarToCartesian(range_km * 1000.0, degToRad(bearing_deg));
    t.x = p[0];
    t.y = p[1];
    t.vx = speed_mps * std::sin(degToRad(heading_deg));
    t.vy = speed_mps * std::cos(degToRad(heading_deg));
    t.turn_rate_rps = degToRad(turn_deg_per_s);
    return t;
}

}  // namespace radar::test
