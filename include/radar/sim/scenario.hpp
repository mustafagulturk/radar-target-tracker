#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "radar/sim/target.hpp"

namespace radar::sim {

/// A set of targets flying around the radar. Targets that leave coverage are
/// replaced by new inbound ones so the picture stays busy indefinitely.
class Scenario {
public:
    /// Hand-made scenario: crossing tracks, a manoeuvring target, a slow
    /// surface contact and a fast inbound, plus `extra_random` random ones.
    static Scenario standard(double max_range_m, int extra_random, std::uint32_t seed);

    /// `count` random targets.
    static Scenario random(int count, double max_range_m, std::uint32_t seed);

    void advance(double dt_s);

    const std::vector<Target>& targets() const { return targets_; }
    void addTarget(Target target);

private:
    Scenario(double max_range_m, std::uint32_t seed);
    Target makeRandomInbound();

    double max_range_m_;
    std::mt19937 rng_;
    std::uint32_t next_id_ = 1;
    std::vector<Target> targets_;
};

}  // namespace radar::sim
