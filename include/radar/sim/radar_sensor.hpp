#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "radar/coordinates.hpp"
#include "radar/sim/target.hpp"
#include "radar/types.hpp"

namespace radar::sim {

struct SensorConfig {
    double max_range_m = 40000.0;
    double rotation_period_s = 2.0;   ///< 30 RPM antenna.
    int sectors_per_revolution = 72;  ///< 5 degree sectors.
    double range_sigma_m = 30.0;
    double azimuth_sigma_rad = degToRad(0.3);
    double detection_probability = 0.9;
    double clutter_per_revolution = 20.0;  ///< Mean false alarms per scan (Poisson).
};

/// Rotating 2D surveillance radar model. Each call to scanSector() reports the
/// targets (and false alarms) inside the sector the beam just swept, with
/// Gaussian range/bearing noise and random missed detections.
class RadarSensor {
public:
    RadarSensor(SensorConfig config, std::uint32_t seed);

    ScanBatch scanSector(const std::vector<Target>& targets, std::uint64_t timestamp_us,
                         double sector_start_rad, double sector_end_rad);

    const SensorConfig& config() const { return config_; }

private:
    SensorConfig config_;
    std::mt19937 rng_;
    std::uint32_t sequence_ = 0;
};

}  // namespace radar::sim
