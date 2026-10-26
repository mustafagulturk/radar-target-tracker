#include "radar/track_manager.hpp"

#include <algorithm>
#include <cmath>

namespace radar {

const char* toString(TrackStatus status) {
    switch (status) {
        case TrackStatus::Tentative:
            return "Tentative";
        case TrackStatus::Confirmed:
            return "Confirmed";
    }
    return "Unknown";
}

double TrackState::range() const {
    return std::hypot(x, y);
}
double TrackState::azimuth() const {
    return cartesianToPolar(Vec2{x, y}).azimuth_rad;
}
double TrackState::speed() const {
    return std::hypot(vx, vy);
}
double TrackState::heading() const {
    return headingOf(vx, vy);
}

TrackManager::TrackManager(TrackerConfig config) : config_(config) {}

std::size_t TrackManager::confirmedCount() const {
    return static_cast<std::size_t>(
        std::count_if(tracks_.begin(), tracks_.end(),
                      [](const Track& t) { return t.status == TrackStatus::Confirmed; }));
}

void TrackManager::predictAll(std::uint64_t timestamp_us) {
    for (Track& track : tracks_) {
        if (timestamp_us > track.time_us) {
            track.filter.predict(static_cast<double>(timestamp_us - track.time_us) * 1e-6);
            track.time_us = timestamp_us;
        }
    }
}

void TrackManager::process(const ScanBatch& batch) {
    predictAll(batch.timestamp_us);

    std::vector<CartesianMeasurement> measurements;
    measurements.reserve(batch.plots.size());
    for (const PolarMeasurement& plot : batch.plots) {
        measurements.push_back(
            convertMeasurement(plot, config_.range_sigma_m, config_.azimuth_sigma_rad));
    }

    // Collect every track/measurement pair that falls inside the gate. Tracks
    // outside the current sector can still be gated: a target sitting on a
    // sector boundary may be reported in either neighbouring batch.
    struct Candidate {
        std::size_t track;
        std::size_t meas;
        double distance_sq;
    };
    std::vector<Candidate> candidates;
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
        for (std::size_t m = 0; m < measurements.size(); ++m) {
            const Innovation inn =
                tracks_[t].filter.innovation(measurements[m].z, measurements[m].R);
            if (inn.mahalanobis_sq <= config_.gate_threshold) {
                candidates.push_back({t, m, inn.mahalanobis_sq});
            }
        }
    }

    // Greedy global nearest neighbour: take the globally closest pair first.
    // Confirmed tracks win ties so a new tentative track cannot steal a plot.
    std::sort(candidates.begin(), candidates.end(), [this](const Candidate& a, const Candidate& b) {
        if (a.distance_sq != b.distance_sq) {
            return a.distance_sq < b.distance_sq;
        }
        return tracks_[a.track].status == TrackStatus::Confirmed &&
               tracks_[b.track].status != TrackStatus::Confirmed;
    });

    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<bool> meas_used(measurements.size(), false);
    for (const Candidate& c : candidates) {
        if (track_used[c.track] || meas_used[c.meas]) {
            continue;
        }
        track_used[c.track] = true;
        meas_used[c.meas] = true;

        Track& track = tracks_[c.track];
        track.filter.update(measurements[c.meas].z, measurements[c.meas].R);
        track.last_update_us = batch.timestamp_us;
        track.consecutive_misses = 0;
        ++track.hits;
        if (track.status == TrackStatus::Tentative && track.hits >= config_.confirm_hits) {
            track.status = TrackStatus::Confirmed;
        }
        const Vec2 p = track.filter.position();
        track.history.push_back({p[0], p[1]});
        while (track.history.size() > config_.history_length) {
            track.history.pop_front();
        }
    }

    // A track only misses if the antenna actually looked where it should be.
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
        if (track_used[t]) {
            continue;
        }
        const double az = cartesianToPolar(tracks_[t].filter.position()).azimuth_rad;
        if (angleInSector(az, batch.sector_start_rad, batch.sector_end_rad)) {
            ++tracks_[t].consecutive_misses;
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track& t) { return shouldDelete(t); }),
                  tracks_.end());

    for (std::size_t m = 0; m < measurements.size(); ++m) {
        if (!meas_used[m]) {
            startTrack(measurements[m], batch.timestamp_us);
        }
    }
}

void TrackManager::startTrack(const CartesianMeasurement& m, std::uint64_t timestamp_us) {
    // Single-point initialisation: position from the plot, velocity unknown
    // (zero mean with a wide prior). The second hit pins the velocity down.
    const Vec4 x0{m.z[0], m.z[1], 0.0, 0.0};
    const double v_var = config_.initial_speed_sigma_mps * config_.initial_speed_sigma_mps;
    Mat4 P0;
    P0(0, 0) = m.R(0, 0);
    P0(0, 1) = m.R(0, 1);
    P0(1, 0) = m.R(1, 0);
    P0(1, 1) = m.R(1, 1);
    P0(2, 2) = v_var;
    P0(3, 3) = v_var;

    Track track{next_id_++,
                TrackStatus::Tentative,
                KalmanFilter(x0, P0, config_.accel_noise_psd),
                timestamp_us,
                timestamp_us,
                1,
                0,
                {}};
    track.history.push_back({m.z[0], m.z[1]});
    tracks_.push_back(std::move(track));
}

bool TrackManager::shouldDelete(const Track& track) const {
    const int max_misses = track.status == TrackStatus::Confirmed ? config_.max_misses_confirmed
                                                                  : config_.max_misses_tentative;
    if (track.consecutive_misses > max_misses) {
        return true;
    }
    const Mat4& P = track.filter.covariance();
    const double sigma = std::sqrt(std::max(P(0, 0), P(1, 1)));
    return sigma > config_.max_position_sigma_m;
}

TrackState TrackManager::toState(const Track& track) {
    TrackState s;
    s.id = track.id;
    s.status = track.status;
    const Vec4& x = track.filter.state();
    s.x = x[0];
    s.y = x[1];
    s.vx = x[2];
    s.vy = x[3];
    const Mat4& P = track.filter.covariance();
    s.position_sigma_m = std::sqrt(std::max(P(0, 0), P(1, 1)));
    s.hits = track.hits;
    s.consecutive_misses = track.consecutive_misses;
    s.last_update_us = track.last_update_us;
    s.history.assign(track.history.begin(), track.history.end());
    return s;
}

std::vector<TrackState> TrackManager::tracks() const {
    std::vector<TrackState> out;
    out.reserve(tracks_.size());
    for (const Track& track : tracks_) {
        out.push_back(toState(track));
    }
    return out;
}

}  // namespace radar
