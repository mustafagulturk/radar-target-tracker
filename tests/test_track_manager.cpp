#include "radar/track_manager.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <optional>
#include <set>

#include "sim_harness.hpp"

namespace radar {
namespace {

using test::makeTarget;
using test::SimHarness;

sim::SensorConfig sensorConfig(double pd, double clutter) {
    sim::SensorConfig cfg;
    cfg.detection_probability = pd;
    cfg.clutter_per_revolution = clutter;
    return cfg;
}

double distance(const TrackState& t, const sim::Target& truth) {
    return std::hypot(t.x - truth.x, t.y - truth.y);
}

/// Closest confirmed track to `truth` within `radius_m`, if any.
std::optional<TrackState> nearestConfirmed(const std::vector<TrackState>& tracks,
                                           const sim::Target& truth, double radius_m) {
    std::optional<TrackState> best;
    for (const TrackState& t : tracks) {
        if (t.status != TrackStatus::Confirmed || distance(t, truth) > radius_m) {
            continue;
        }
        if (!best || distance(t, truth) < distance(*best, truth)) {
            best = t;
        }
    }
    return best;
}

std::size_t countConfirmedNear(const std::vector<TrackState>& tracks, const sim::Target& truth,
                               double radius_m) {
    std::size_t n = 0;
    for (const TrackState& t : tracks) {
        if (t.status == TrackStatus::Confirmed && distance(t, truth) <= radius_m) {
            ++n;
        }
    }
    return n;
}

void runRevolution(SimHarness& sim, TrackManager& tracker) {
    for (const ScanBatch& batch : sim.nextRevolution()) {
        tracker.process(batch);
    }
}

TEST(TrackManager, EmptyBatchesCreateNothing) {
    TrackManager tracker;
    ScanBatch batch;
    batch.timestamp_us = 1000;
    tracker.process(batch);
    EXPECT_EQ(tracker.trackCount(), 0U);
    EXPECT_TRUE(tracker.tracks().empty());
}

TEST(TrackManager, TrackLifeCycleOnSyntheticPlots) {
    // One plot per revolution at a fixed point: tentative after one hit,
    // confirmed after confirm_hits, deleted after too many misses.
    TrackerConfig cfg;
    TrackManager tracker(cfg);
    const PolarMeasurement plot{10000.0, degToRad(45.0), 10.0};

    std::uint64_t t_us = 0;
    for (int hit = 1; hit <= cfg.confirm_hits; ++hit) {
        t_us += 2'000'000;
        ScanBatch b;
        b.timestamp_us = t_us;
        b.sector_start_rad = degToRad(40.0);
        b.sector_end_rad = degToRad(50.0);
        b.plots = {plot};
        tracker.process(b);
        const auto tracks = tracker.tracks();
        ASSERT_EQ(tracks.size(), 1U);
        EXPECT_EQ(tracks[0].id, 1U);
        EXPECT_EQ(tracks[0].hits, hit);
        EXPECT_EQ(tracks[0].status,
                  hit >= cfg.confirm_hits ? TrackStatus::Confirmed : TrackStatus::Tentative);
    }
    EXPECT_EQ(tracker.confirmedCount(), 1U);

    for (int miss = 1; miss <= cfg.max_misses_confirmed + 1; ++miss) {
        t_us += 2'000'000;
        ScanBatch b;
        b.timestamp_us = t_us;
        b.sector_start_rad = degToRad(40.0);
        b.sector_end_rad = degToRad(50.0);
        tracker.process(b);
        if (miss <= cfg.max_misses_confirmed) {
            ASSERT_EQ(tracker.trackCount(), 1U) << "miss " << miss;
            EXPECT_EQ(tracker.tracks()[0].consecutive_misses, miss);
        } else {
            EXPECT_EQ(tracker.trackCount(), 0U);
        }
    }
}

TEST(TrackManager, NoMissWhenAntennaLooksElsewhere) {
    TrackManager tracker;
    ScanBatch b;
    b.timestamp_us = 1'000'000;
    b.sector_start_rad = 0.0;
    b.sector_end_rad = degToRad(5.0);
    b.plots = {{10000.0, degToRad(2.0), 10.0}};
    tracker.process(b);
    ASSERT_EQ(tracker.trackCount(), 1U);

    // Sweep the opposite side of the picture several times.
    for (int i = 0; i < 10; ++i) {
        ScanBatch other;
        other.timestamp_us = b.timestamp_us + static_cast<std::uint64_t>(i + 1) * 10'000;
        other.sector_start_rad = degToRad(180.0);
        other.sector_end_rad = degToRad(185.0);
        tracker.process(other);
    }
    ASSERT_EQ(tracker.trackCount(), 1U);
    EXPECT_EQ(tracker.tracks()[0].consecutive_misses, 0);
}

class SingleTargetTest : public ::testing::TestWithParam<double> {};

TEST_P(SingleTargetTest, OneConfirmedTrackWithStableIdAndAccurateState) {
    const double clutter = GetParam();
    SimHarness sim(sensorConfig(0.9, clutter), 11);
    sim.addTarget(makeTarget(25.0, 45.0, 220.0, 260.0));
    TrackManager tracker;

    std::optional<std::uint32_t> id;
    double pos_sq = 0.0;
    double vel_sq = 0.0;
    int samples = 0;
    double plot_sq = 0.0;  // raw plot error, the baseline the filter must beat
    int plot_samples = 0;
    for (int rev = 1; rev <= 40; ++rev) {
        for (int sector = 0; sector < sim.config().sectors_per_revolution; ++sector) {
            const ScanBatch batch = sim.nextSector();
            tracker.process(batch);
            const sim::Target& truth = sim.targets().front();
            for (const PolarMeasurement& plot : batch.plots) {
                const Vec2 p = polarToCartesian(plot.range_m, plot.azimuth_rad);
                const double e = std::hypot(p[0] - truth.x, p[1] - truth.y);
                if (e < 1000.0) {
                    plot_sq += e * e;
                    ++plot_samples;
                }
            }
        }
        if (rev < 8) {
            continue;  // initiation and convergence
        }
        const sim::Target& truth = sim.targets().front();
        const auto tracks = tracker.tracks();
        ASSERT_EQ(countConfirmedNear(tracks, truth, 1000.0), 1U) << "rev " << rev;
        const auto track = nearestConfirmed(tracks, truth, 1000.0);
        if (!id) {
            id = track->id;
        }
        EXPECT_EQ(track->id, *id) << "track id changed at rev " << rev;

        pos_sq += std::pow(distance(*track, truth), 2.0);
        vel_sq += std::pow(track->vx - truth.vx, 2.0) + std::pow(track->vy - truth.vy, 2.0);
        ++samples;
        if (clutter == 0.0) {
            EXPECT_EQ(tracker.confirmedCount(), 1U);
        }
    }
    const double pos_rmse = std::sqrt(pos_sq / samples);
    const double vel_rmse = std::sqrt(vel_sq / samples);
    const double plot_rmse = std::sqrt(plot_sq / plot_samples);
    // Raw plots are ~100 m RMS over this leg (25 km closing to ~10 km, 30 m in
    // range, 0.3 deg in bearing). The default process noise is sized so the
    // 1.5 g turn of the standard scenario stays inside the gate, which leaves
    // little position smoothing on a straight leg: the tracker must not be worse
    // than the raw plots, and must deliver an accurate velocity they lack.
    EXPECT_LT(pos_rmse, plot_rmse);
    EXPECT_LT(pos_rmse, 130.0);
    EXPECT_LT(vel_rmse, 20.0);
    RecordProperty("plot_rmse_m", std::to_string(plot_rmse));
    RecordProperty("pos_rmse_m", std::to_string(pos_rmse));
    RecordProperty("vel_rmse_mps", std::to_string(vel_rmse));
}

INSTANTIATE_TEST_SUITE_P(Clutter, SingleTargetTest, ::testing::Values(0.0, 20.0),
                         [](const ::testing::TestParamInfo<double>& param_info) {
                             return param_info.param == 0.0 ? std::string("NoClutter")
                                                            : std::string("WithClutter");
                         });

TEST(TrackManager, CrossingTargetsKeepDistinctIds) {
    SimHarness sim(sensorConfig(0.9, 20.0), 23);
    // Two airliners whose paths cross at right angles ~15 km north of the radar,
    // passing within a few hundred metres of each other.
    sim.addTarget(
        makeTarget(std::hypot(9.0, 15.0), radToDeg(std::atan2(-9.0, 15.0)) + 360.0, 200.0, 90.0));
    sim.addTarget(makeTarget(std::hypot(0.3, 6.0), radToDeg(std::atan2(0.3, 6.0)), 200.0, 0.0));
    TrackManager tracker;

    std::map<std::size_t, std::uint32_t> assigned;  // truth index -> last track id
    std::map<std::size_t, int> id_changes;
    for (int rev = 1; rev <= 45; ++rev) {
        runRevolution(sim, tracker);
        if (rev < 5) {
            continue;
        }
        const auto tracks = tracker.tracks();
        std::set<std::uint32_t> ids_this_rev;
        for (std::size_t i = 0; i < 2; ++i) {
            const auto track = nearestConfirmed(tracks, sim.targets()[i], 1500.0);
            ASSERT_TRUE(track.has_value()) << "target " << i << " lost at rev " << rev;
            ids_this_rev.insert(track->id);
            if (assigned.count(i) != 0 && assigned[i] != track->id) {
                ++id_changes[i];
            }
            assigned[i] = track->id;
        }
        // Near the crossing both truths may momentarily share the closest track,
        // but away from it they must be held by different tracks.
        const double separation = std::hypot(sim.targets()[0].x - sim.targets()[1].x,
                                             sim.targets()[0].y - sim.targets()[1].y);
        if (separation > 2000.0) {
            EXPECT_EQ(ids_this_rev.size(), 2U) << "rev " << rev;
        }
    }
    EXPECT_EQ(id_changes[0], 0);
    EXPECT_EQ(id_changes[1], 0);
    EXPECT_NE(assigned[0], assigned[1]);
}

TEST(TrackManager, DisappearedTargetIsDeleted) {
    SimHarness sim(sensorConfig(0.9, 20.0), 5);
    sim.addTarget(makeTarget(20.0, 200.0, 150.0, 30.0));
    TrackManager tracker;

    for (int rev = 0; rev < 10; ++rev) {
        runRevolution(sim, tracker);
    }
    const auto track = nearestConfirmed(tracker.tracks(), sim.targets().front(), 1000.0);
    ASSERT_TRUE(track.has_value());
    const std::uint32_t id = track->id;

    auto alive = [&] {
        for (const TrackState& t : tracker.tracks()) {
            if (t.id == id) {
                return true;
            }
        }
        return false;
    };

    sim.setTargetsVisible(false);
    // Coasts through max_misses_confirmed scans...
    for (int rev = 0; rev < tracker.config().max_misses_confirmed - 1; ++rev) {
        runRevolution(sim, tracker);
    }
    EXPECT_TRUE(alive()) << "confirmed track deleted too early";
    // ...and is gone shortly after.
    for (int rev = 0; rev < 3; ++rev) {
        runRevolution(sim, tracker);
    }
    EXPECT_FALSE(alive()) << "track of a vanished target survived";
}

TEST(TrackManager, PureClutterProducesNoLastingConfirmedTracks) {
    SimHarness sim(sensorConfig(0.9, 20.0), 99);
    TrackManager tracker;

    std::map<std::uint32_t, int> confirmed_revs;  // id -> revolutions seen confirmed
    std::size_t max_confirmed = 0;
    std::size_t max_tracks = 0;
    for (int rev = 0; rev < 150; ++rev) {
        runRevolution(sim, tracker);
        max_confirmed = std::max(max_confirmed, tracker.confirmedCount());
        max_tracks = std::max(max_tracks, tracker.trackCount());
        for (const TrackState& t : tracker.tracks()) {
            if (t.status == TrackStatus::Confirmed) {
                ++confirmed_revs[t.id];
            }
        }
    }
    int longest = 0;
    for (const auto& entry : confirmed_revs) {
        longest = std::max(longest, entry.second);
    }
    RecordProperty("false_confirmed_ids", static_cast<int>(confirmed_revs.size()));
    RecordProperty("longest_false_track_revs", longest);
    // 150 scans x 20 false alarms = 3000 clutter plots.
    // A false track can reach confirm_hits by chance, but must die again after
    // max_misses_confirmed empty scans instead of living on.
    EXPECT_LE(confirmed_revs.size(), 6U);
    EXPECT_LE(max_confirmed, 2U);
    EXPECT_LE(longest, 12);
    // Tentative tracks must not pile up either.
    EXPECT_LT(max_tracks, 60U);
}

struct TurnResult {
    bool confirmed = false;
    int id_changes = 0;
    int lost_revs = 0;
    double worst_error_m = 0.0;
    double worst_speed_error_mps = 0.0;
};

/// The manoeuvring case of Scenario::standard(): 280 m/s in a sustained
/// 3 deg/s turn (~15 m/s^2, 1.5 g) at ~18 km, 2 s revisit, Pd 0.9, clutter on.
/// Follows the target from its first confirmation through a full 360 deg turn.
TurnResult runTurningTarget(std::uint32_t seed) {
    SimHarness sim(sensorConfig(0.9, 20.0), seed);
    sim.addTarget(makeTarget(18.0, 150.0, 280.0, 0.0, 3.0));
    TrackManager tracker;
    const int full_turn_revs =
        static_cast<int>(360.0 / 3.0 / sim.config().rotation_period_s);  // 60

    TurnResult result;
    std::optional<std::uint32_t> id;
    int confirmed_at = -1;
    for (int rev = 1; rev <= 20 + full_turn_revs; ++rev) {
        runRevolution(sim, tracker);
        const sim::Target& truth = sim.targets().front();
        const auto track = nearestConfirmed(tracker.tracks(), truth, 1500.0);
        if (confirmed_at < 0) {
            if (track) {
                confirmed_at = rev;
                id = track->id;
                result.confirmed = true;
            }
            continue;
        }
        if (rev > confirmed_at + full_turn_revs) {
            break;
        }
        if (!track) {
            ++result.lost_revs;
            continue;
        }
        if (track->id != *id) {
            ++result.id_changes;
            id = track->id;
        }
        result.worst_error_m = std::max(result.worst_error_m, distance(*track, truth));
        result.worst_speed_error_mps =
            std::max(result.worst_speed_error_mps, std::abs(track->speed() - truth.speed()));
    }
    return result;
}

TEST(TrackManager, TurningTargetHoldsOneTrackThroughFullTurn) {
    const TurnResult r = runTurningTarget(31);
    ASSERT_TRUE(r.confirmed);
    EXPECT_EQ(r.id_changes, 0);
    EXPECT_EQ(r.lost_revs, 0);
    EXPECT_LT(r.worst_error_m, 1000.0);  // CV model lags in the turn
    EXPECT_LT(r.worst_speed_error_mps, 80.0);
    RecordProperty("worst_error_m", std::to_string(r.worst_error_m));
}

TEST(TrackManager, TurningTargetIsHeldAcrossSeeds) {
    // Statistical version: a constant-velocity filter with too little process
    // noise (the old q = 20) drops this target in every run; the tuned
    // defaults keep one ID through the whole turn in the large majority.
    constexpr int kRuns = 20;
    int held = 0;
    for (int seed = 1; seed <= kRuns; ++seed) {
        const TurnResult r = runTurningTarget(static_cast<std::uint32_t>(seed));
        EXPECT_TRUE(r.confirmed) << "seed " << seed;
        EXPECT_LE(r.lost_revs, 2) << "seed " << seed;
        if (r.confirmed && r.id_changes == 0 && r.lost_revs == 0) {
            ++held;
        }
    }
    RecordProperty("runs_holding_one_id", held);
    EXPECT_GE(held, 17) << held << " of " << kRuns << " runs kept a single track ID";
}

TEST(TrackManager, HistoryIsBounded) {
    TrackerConfig cfg;
    cfg.history_length = 5;
    SimHarness sim(sensorConfig(1.0, 0.0), 3);
    sim.addTarget(makeTarget(10.0, 10.0, 100.0, 90.0));
    TrackManager tracker(cfg);
    for (int rev = 0; rev < 12; ++rev) {
        runRevolution(sim, tracker);
    }
    ASSERT_EQ(tracker.trackCount(), 1U);
    EXPECT_EQ(tracker.tracks()[0].history.size(), 5U);
}

}  // namespace
}  // namespace radar
