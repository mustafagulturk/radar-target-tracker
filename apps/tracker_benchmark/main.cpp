// Monte Carlo benchmark: runs the tracker against simulated ground truth and
// reports accuracy (position/velocity RMSE), track continuity (coverage, ID
// switches), false tracks and per-datagram processing time.
//
// Everything runs in-process through the real wire format (encode -> decode
// -> TrackManager), so the timing covers exactly the work the receiver and
// tracker threads do for each datagram, without network jitter.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "radar/cli.hpp"
#include "radar/coordinates.hpp"
#include "radar/protocol.hpp"
#include "radar/sim/radar_sensor.hpp"
#include "radar/sim/scenario.hpp"
#include "radar/track_manager.hpp"

namespace {

using namespace radar;
using Clock = std::chrono::steady_clock;

/// A confirmed track is credited to a truth target only within this distance.
constexpr double kAssociationRadiusM = 1000.0;
/// Truth targets younger than this are not scored (track initiation time).
constexpr double kWarmUpS = 10.0;

struct ScenarioSpec {
    std::string name;
    std::function<sim::Scenario(double max_range_m, std::uint32_t seed)> make;
};

sim::Target makeTarget(double range_km, double bearing_deg, double speed_mps, double heading_deg,
                       double turn_deg_per_s = 0.0) {
    sim::Target t;
    const Vec2 p = polarToCartesian(range_km * 1000.0, degToRad(bearing_deg));
    t.x = p[0];
    t.y = p[1];
    t.vx = speed_mps * std::sin(degToRad(heading_deg));
    t.vy = speed_mps * std::cos(degToRad(heading_deg));
    t.turn_rate_rps = degToRad(turn_deg_per_s);
    return t;
}

sim::Scenario withTargets(double max_range_m, std::uint32_t seed,
                          const std::vector<sim::Target>& targets) {
    sim::Scenario s = sim::Scenario::random(0, max_range_m, seed);
    for (const sim::Target& t : targets) {
        s.addTarget(t);
    }
    return s;
}

std::vector<ScenarioSpec> scenarios() {
    return {
        {"Straight, 250 m/s",
         [](double r, std::uint32_t seed) {
             return withTargets(r, seed, {makeTarget(30.0, 45.0, 250.0, 200.0)});
         }},
        {"Crossing pair, 90 deg",
         [](double r, std::uint32_t seed) {
             return withTargets(
                 r, seed,
                 {makeTarget(25.0, 300.0, 220.0, 120.0), makeTarget(25.0, 60.0, 220.0, 210.0)});
         }},
        {"Manoeuvring, 3 deg/s turn",
         [](double r, std::uint32_t seed) {
             return withTargets(r, seed, {makeTarget(18.0, 150.0, 280.0, 0.0, 3.0)});
         }},
        {"Standard, 11 targets",
         [](double r, std::uint32_t seed) { return sim::Scenario::standard(r, 6, seed); }},
    };
}

struct Accumulator {
    double raw_sq_sum = 0.0;  ///< Expected squared error of a single raw plot.
    double pos_sq_sum = 0.0;
    double vel_sq_sum = 0.0;
    std::uint64_t updates = 0;  ///< Scored filtered estimates.
    std::uint64_t covered = 0;  ///< Truth-target scans with a confirmed track on them.
    std::uint64_t scored = 0;   ///< Truth-target scans that should have a track.
    std::uint64_t id_switches = 0;
    std::uint64_t false_tracks = 0;
    std::uint64_t scans = 0;
    std::vector<double> datagram_us;
};

/// Scores every confirmed track updated in the batch just processed: the
/// filtered estimate against the nearest truth target.
void scoreAccuracy(const TrackManager& tracker, const sim::Scenario& scenario,
                   const sim::SensorConfig& sensor_cfg, std::uint64_t time_us, double time_s,
                   std::map<std::uint32_t, double>& first_seen_s, Accumulator& acc) {
    for (const TrackState& trk : tracker.tracks()) {
        if (trk.status != TrackStatus::Confirmed || trk.last_update_us != time_us) {
            continue;
        }
        const sim::Target* best = nullptr;
        double best_d = kAssociationRadiusM;
        for (const sim::Target& tgt : scenario.targets()) {
            const double d = std::hypot(tgt.x - trk.x, tgt.y - trk.y);
            if (d < best_d) {
                best_d = d;
                best = &tgt;
            }
        }
        if (best == nullptr || time_s - first_seen_s[best->id] < kWarmUpS) {
            continue;
        }
        acc.pos_sq_sum += best_d * best_d;
        const double dvx = best->vx - trk.vx;
        const double dvy = best->vy - trk.vy;
        acc.vel_sq_sum += dvx * dvx + dvy * dvy;
        const double cross_range = best->range() * sensor_cfg.azimuth_sigma_rad;
        acc.raw_sq_sum +=
            sensor_cfg.range_sigma_m * sensor_cfg.range_sigma_m + cross_range * cross_range;
        ++acc.updates;
    }
}

/// Once per revolution: one-to-one assignment of truth targets to confirmed
/// tracks for coverage, ID switches and false tracks.
void scoreContinuity(const TrackManager& tracker, const sim::Scenario& scenario,
                     const sim::SensorConfig& sensor_cfg, double time_s,
                     std::map<std::uint32_t, double>& first_seen_s,
                     std::map<std::uint32_t, std::uint32_t>& last_track, Accumulator& acc) {
    ++acc.scans;

    std::vector<TrackState> confirmed;
    for (TrackState& t : tracker.tracks()) {
        if (t.status == TrackStatus::Confirmed) {
            confirmed.push_back(std::move(t));
        }
    }

    struct Pair {
        std::size_t truth;
        std::size_t track;
        double dist;
    };
    const std::vector<sim::Target>& truth = scenario.targets();
    std::vector<Pair> pairs;
    for (std::size_t i = 0; i < truth.size(); ++i) {
        for (std::size_t j = 0; j < confirmed.size(); ++j) {
            const double d = std::hypot(truth[i].x - confirmed[j].x, truth[i].y - confirmed[j].y);
            if (d < kAssociationRadiusM) {
                pairs.push_back({i, j, d});
            }
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.dist < b.dist; });

    std::vector<bool> truth_used(truth.size(), false);
    std::vector<bool> track_used(confirmed.size(), false);
    for (const Pair& p : pairs) {
        if (truth_used[p.truth] || track_used[p.track]) {
            continue;
        }
        truth_used[p.truth] = true;
        track_used[p.track] = true;

        const std::uint32_t truth_id = truth[p.truth].id;
        const std::uint32_t track_id = confirmed[p.track].id;
        const auto prev = last_track.find(truth_id);
        if (prev != last_track.end() && prev->second != track_id) {
            ++acc.id_switches;
        }
        last_track[truth_id] = track_id;
    }

    for (std::size_t i = 0; i < truth.size(); ++i) {
        const bool in_play = time_s - first_seen_s[truth[i].id] >= kWarmUpS &&
                             truth[i].range() < sensor_cfg.max_range_m;
        if (in_play) {
            ++acc.scored;
            if (truth_used[i]) {
                ++acc.covered;
            }
        }
    }
    acc.false_tracks +=
        static_cast<std::uint64_t>(std::count(track_used.begin(), track_used.end(), false));
}

void runOnce(const ScenarioSpec& spec, const sim::SensorConfig& sensor_cfg, double duration_s,
             std::uint32_t seed, Accumulator& acc) {
    sim::Scenario scenario = spec.make(sensor_cfg.max_range_m, seed);
    sim::RadarSensor sensor(sensor_cfg, seed + 7919);
    TrackManager tracker;

    const int sectors = sensor_cfg.sectors_per_revolution;
    const double sector_dt = sensor_cfg.rotation_period_s / sectors;
    const double sector_width = kTwoPi / sectors;
    const auto total_steps = static_cast<long>(duration_s / sector_dt);

    std::map<std::uint32_t, double> first_seen_s;       // truth id -> time
    std::map<std::uint32_t, std::uint32_t> last_track;  // truth id -> track id

    for (long step = 0; step < total_steps; ++step) {
        scenario.advance(sector_dt);
        const double time_s = static_cast<double>(step + 1) * sector_dt;
        const auto time_us = static_cast<std::uint64_t>(time_s * 1e6);
        const double start = static_cast<double>(step % sectors) * sector_width;
        for (const sim::Target& t : scenario.targets()) {
            first_seen_s.emplace(t.id, time_s);
        }

        const ScanBatch batch =
            sensor.scanSector(scenario.targets(), time_us, start, start + sector_width);
        for (const ScanBatch& part : protocol::splitBatch(batch)) {
            const std::vector<std::uint8_t> datagram = protocol::encode(part);
            const auto t0 = Clock::now();
            if (const auto decoded = protocol::decode(datagram)) {
                tracker.process(*decoded);
            }
            const auto t1 = Clock::now();
            acc.datagram_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        }

        scoreAccuracy(tracker, scenario, sensor_cfg, time_us, time_s, first_seen_s, acc);
        if ((step + 1) % sectors == 0) {
            scoreContinuity(tracker, scenario, sensor_cfg, time_s, first_seen_s, last_track, acc);
        }
    }
}

double percentile(std::vector<double> values, double p) {
    if (values.empty()) {
        return 0.0;
    }
    const auto idx = static_cast<std::size_t>(p * static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx),
                     values.end());
    return values[idx];
}

double ratio(double num, std::uint64_t den) {
    return num / static_cast<double>(std::max<std::uint64_t>(den, 1));
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const cli::Args args(argc, argv);
        if (args.has("help")) {
            std::cout << "Usage: tracker_benchmark [--runs N] [--duration S] [--clutter N]\n";
            return 0;
        }
        const auto runs = static_cast<std::uint32_t>(args.get("runs", 50L));
        const double duration_s = args.get("duration", 120.0);

        sim::SensorConfig sensor_cfg;
        sensor_cfg.clutter_per_revolution = args.get("clutter", sensor_cfg.clutter_per_revolution);

        std::printf(
            "Monte Carlo: %u runs x %.0f s per scenario, Pd %.2f, %.0f false alarms/scan, "
            "sigma_r %.0f m, sigma_az %.2f deg\n\n",
            runs, duration_s, sensor_cfg.detection_probability, sensor_cfg.clutter_per_revolution,
            sensor_cfg.range_sigma_m, radToDeg(sensor_cfg.azimuth_sigma_rad));
        std::printf(
            "| Scenario | Raw plot error (m) | Track pos. RMSE (m) | Vel. RMSE (m/s) | "
            "Track coverage | ID switches / run | False tracks / scan | Datagram mean (us) | "
            "Datagram p99 (us) |\n");
        std::printf("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");

        for (const ScenarioSpec& spec : scenarios()) {
            Accumulator acc;
            for (std::uint32_t seed = 1; seed <= runs; ++seed) {
                runOnce(spec, sensor_cfg, duration_s, seed, acc);
            }
            double sum_us = 0.0;
            for (double v : acc.datagram_us) {
                sum_us += v;
            }
            std::printf("| %s | %.0f | %.0f | %.1f | %.1f %% | %.2f | %.2f | %.1f | %.1f |\n",
                        spec.name.c_str(), std::sqrt(ratio(acc.raw_sq_sum, acc.updates)),
                        std::sqrt(ratio(acc.pos_sq_sum, acc.updates)),
                        std::sqrt(ratio(acc.vel_sq_sum, acc.updates)),
                        100.0 * ratio(static_cast<double>(acc.covered), acc.scored),
                        ratio(static_cast<double>(acc.id_switches), runs),
                        ratio(static_cast<double>(acc.false_tracks), acc.scans),
                        ratio(sum_us, acc.datagram_us.size()), percentile(acc.datagram_us, 0.99));
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "tracker_benchmark: " << e.what() << '\n';
        return 1;
    }
}
