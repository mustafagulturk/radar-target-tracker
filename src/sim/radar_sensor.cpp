#include "radar/sim/radar_sensor.hpp"

#include <algorithm>

namespace radar::sim {

RadarSensor::RadarSensor(SensorConfig config, std::uint32_t seed) : config_(config), rng_(seed) {}

ScanBatch RadarSensor::scanSector(const std::vector<Target>& targets, std::uint64_t timestamp_us,
                                  double sector_start_rad, double sector_end_rad) {
    ScanBatch batch;
    batch.sequence = sequence_++;
    batch.timestamp_us = timestamp_us;
    batch.sector_start_rad = normalizeAngle(sector_start_rad);
    batch.sector_end_rad = normalizeAngle(sector_end_rad);

    std::normal_distribution<double> range_noise(0.0, config_.range_sigma_m);
    std::normal_distribution<double> azimuth_noise(0.0, config_.azimuth_sigma_rad);
    std::normal_distribution<double> amplitude_noise(0.0, 2.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    for (const Target& target : targets) {
        const PolarMeasurement truth = cartesianToPolar(Vec2{target.x, target.y});
        if (truth.range_m > config_.max_range_m ||
            !angleInSector(truth.azimuth_rad, batch.sector_start_rad, batch.sector_end_rad)) {
            continue;
        }
        if (unit(rng_) > config_.detection_probability) {
            continue;
        }
        PolarMeasurement plot;
        plot.range_m = std::max(0.0, truth.range_m + range_noise(rng_));
        plot.azimuth_rad = normalizeAngle(truth.azimuth_rad + azimuth_noise(rng_));
        plot.amplitude_db = target.rcs_db + amplitude_noise(rng_);
        batch.plots.push_back(plot);
    }

    // False alarms: Poisson count per sector, uniform over the sector area
    // (range drawn as sqrt(u) * R_max so density is uniform per square metre).
    const double sector_width = normalizeAngle(batch.sector_end_rad - batch.sector_start_rad);
    const double expected = config_.clutter_per_revolution * sector_width / kTwoPi;
    std::poisson_distribution<int> clutter_count(expected);
    const int n_clutter = expected > 0.0 ? clutter_count(rng_) : 0;
    for (int i = 0; i < n_clutter; ++i) {
        PolarMeasurement plot;
        plot.range_m = std::sqrt(unit(rng_)) * config_.max_range_m;
        plot.azimuth_rad = normalizeAngle(batch.sector_start_rad + unit(rng_) * sector_width);
        plot.amplitude_db = 3.0 + amplitude_noise(rng_);
        batch.plots.push_back(plot);
    }

    // Real radars report plots in range order; also hides which ones are clutter.
    std::sort(
        batch.plots.begin(), batch.plots.end(),
        [](const PolarMeasurement& a, const PolarMeasurement& b) { return a.range_m < b.range_m; });
    return batch;
}

}  // namespace radar::sim
