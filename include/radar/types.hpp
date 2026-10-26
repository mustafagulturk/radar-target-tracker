#pragma once

#include <cstdint>
#include <vector>

namespace radar {

/// A single radar detection ("plot") in sensor-centred polar coordinates.
///
/// Azimuth follows the navigation convention used on radar displays:
/// 0 rad points north and the angle grows clockwise.
struct PolarMeasurement {
    double range_m = 0.0;
    double azimuth_rad = 0.0;
    double amplitude_db = 0.0;
};

/// All detections reported while the antenna swept one azimuth sector.
/// This is the unit that travels over UDP and through the processing queue.
struct ScanBatch {
    std::uint32_t sequence = 0;
    std::uint64_t timestamp_us = 0;  ///< Sensor time at the end of the sector.
    double sector_start_rad = 0.0;   ///< Sector covered by this batch, clockwise
    double sector_end_rad = 0.0;     ///< from start to end (may wrap past north).
    std::vector<PolarMeasurement> plots;
};

}  // namespace radar
