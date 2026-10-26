#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "radar/coordinates.hpp"
#include "radar/kalman_filter.hpp"
#include "radar/types.hpp"

namespace radar {

struct TrackerConfig {
    /// Kalman process noise q, m^2/s^3. Sized for manoeuvring targets
    /// (q ~ a^2 * T with a ~ 12 m/s^2 and a 2 s revisit) so a 1.5 g turn stays
    /// inside the gate; the price is a noisier estimate on straight legs.
    double accel_noise_psd = 300.0;
    double range_sigma_m = 30.0;               ///< Sensor range accuracy (1 sigma).
    double azimuth_sigma_rad = degToRad(0.3);  ///< Sensor bearing accuracy (1 sigma).
    double initial_speed_sigma_mps = 200.0;    ///< Velocity uncertainty of a new track.
    double gate_threshold = 16.0;              ///< Chi-square, 2 dof, ~99.97 % gate.
    int confirm_hits = 3;                      ///< Hits needed to confirm a track.
    int max_misses_tentative = 1;              ///< Tentative track dropped after this many misses.
    int max_misses_confirmed = 4;              ///< Confirmed track dropped after this many misses.
    double max_position_sigma_m = 3000.0;      ///< Drop tracks whose position becomes this vague.
    std::size_t history_length = 60;           ///< Past positions kept for display.
};

enum class TrackStatus { Tentative, Confirmed };

const char* toString(TrackStatus status);

struct TrackPoint {
    double x = 0.0;
    double y = 0.0;
};

/// Read-only view of a track, safe to copy across threads.
struct TrackState {
    std::uint32_t id = 0;
    TrackStatus status = TrackStatus::Tentative;
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double position_sigma_m = 0.0;  ///< sqrt of the larger position variance.
    int hits = 0;
    int consecutive_misses = 0;
    std::uint64_t last_update_us = 0;
    std::vector<TrackPoint> history;  ///< Oldest first.

    double range() const;
    double azimuth() const;
    double speed() const;
    double heading() const;
};

/// Multi-target tracker: predicts every track to the time of each incoming
/// sector, associates plots with tracks using a Mahalanobis gate and greedy
/// global-nearest-neighbour assignment, and manages the track life cycle
/// (tentative -> confirmed -> deleted).
///
/// Not thread-safe; owned by the single tracker thread of TrackingPipeline.
class TrackManager {
public:
    explicit TrackManager(TrackerConfig config = {});

    void process(const ScanBatch& batch);

    std::vector<TrackState> tracks() const;
    std::size_t trackCount() const { return tracks_.size(); }
    std::size_t confirmedCount() const;
    const TrackerConfig& config() const { return config_; }

private:
    struct Track {
        std::uint32_t id;
        TrackStatus status;
        KalmanFilter filter;
        std::uint64_t time_us;
        std::uint64_t last_update_us;
        int hits;
        int consecutive_misses;
        std::deque<TrackPoint> history;
    };

    void predictAll(std::uint64_t timestamp_us);
    void startTrack(const CartesianMeasurement& m, std::uint64_t timestamp_us);
    bool shouldDelete(const Track& track) const;
    static TrackState toState(const Track& track);

    TrackerConfig config_;
    std::vector<Track> tracks_;
    std::uint32_t next_id_ = 1;
};

}  // namespace radar
