#include "radar/sim/scenario.hpp"

#include <algorithm>
#include <cmath>

#include "radar/coordinates.hpp"

namespace radar::sim {

namespace {

Target makeTarget(double range_km, double bearing_deg, double speed_mps, double heading_deg,
                  double turn_deg_per_s = 0.0, double rcs_db = 10.0) {
    Target t;
    const Vec2 p = polarToCartesian(range_km * 1000.0, degToRad(bearing_deg));
    t.x = p[0];
    t.y = p[1];
    t.vx = speed_mps * std::sin(degToRad(heading_deg));
    t.vy = speed_mps * std::cos(degToRad(heading_deg));
    t.turn_rate_rps = degToRad(turn_deg_per_s);
    t.rcs_db = rcs_db;
    return t;
}

}  // namespace

Scenario::Scenario(double max_range_m, std::uint32_t seed)
    : max_range_m_(max_range_m), rng_(seed) {}

void Scenario::addTarget(Target target) {
    target.id = next_id_++;
    targets_.push_back(target);
}

Scenario Scenario::standard(double max_range_m, int extra_random, std::uint32_t seed) {
    Scenario s(max_range_m, seed);
    const double r = max_range_m / 1000.0;
    // Two airliners on crossing courses.
    s.addTarget(makeTarget(0.8 * r, 300.0, 230.0, 100.0));
    s.addTarget(makeTarget(0.8 * r, 60.0, 220.0, 255.0));
    // Fighter in a sustained 3 deg/s turn.
    s.addTarget(makeTarget(0.45 * r, 150.0, 280.0, 0.0, 3.0, 8.0));
    // Slow surface vessel.
    s.addTarget(makeTarget(0.3 * r, 220.0, 12.0, 45.0, 0.0, 20.0));
    // Fast inbound low-RCS contact.
    s.addTarget(makeTarget(0.95 * r, 20.0, 450.0, 200.0, 0.0, 0.0));
    for (int i = 0; i < extra_random; ++i) {
        s.addTarget(s.makeRandomInbound());
    }
    return s;
}

Scenario Scenario::random(int count, double max_range_m, std::uint32_t seed) {
    Scenario s(max_range_m, seed);
    for (int i = 0; i < count; ++i) {
        s.addTarget(s.makeRandomInbound());
    }
    return s;
}

Target Scenario::makeRandomInbound() {
    std::uniform_real_distribution<double> bearing(0.0, 360.0);
    std::uniform_real_distribution<double> speed(80.0, 320.0);
    std::uniform_real_distribution<double> offset(-50.0, 50.0);
    std::uniform_real_distribution<double> range(0.7, 0.95);
    std::uniform_real_distribution<double> turn(-2.0, 2.0);
    std::bernoulli_distribution manoeuvring(0.3);

    // Draw in a fixed order: argument evaluation order is unspecified in C++,
    // and a given seed must produce the same scenario on every compiler.
    const double b = bearing(rng_);
    // Head roughly towards the radar, offset so tracks do not all meet at the origin.
    const double heading = std::fmod(b + 180.0 + offset(rng_) + 360.0, 360.0);
    const double range_km = range(rng_) * max_range_m_ / 1000.0;
    const double speed_mps = speed(rng_);
    const double turn_rate = manoeuvring(rng_) ? turn(rng_) : 0.0;
    return makeTarget(range_km, b, speed_mps, heading, turn_rate);
}

void Scenario::advance(double dt_s) {
    for (Target& t : targets_) {
        t.advance(dt_s);
        if (t.range() > max_range_m_ * 1.05) {
            const std::uint32_t id = next_id_++;
            t = makeRandomInbound();
            t.id = id;
        }
    }
}

}  // namespace radar::sim
