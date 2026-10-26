#include "radar/sim/target.hpp"

#include <cmath>

namespace radar::sim {

void Target::advance(double dt_s) {
    const double w = turn_rate_rps;
    if (std::abs(w) < 1e-9) {
        x += vx * dt_s;
        y += vy * dt_s;
        return;
    }

    // Exact coordinated-turn integration. A positive rate turns the velocity
    // vector clockwise, matching the clockwise bearing convention.
    const double s = std::sin(w * dt_s);
    const double c = std::cos(w * dt_s);
    x += (vx * s + vy * (1.0 - c)) / w;
    y += (vy * s - vx * (1.0 - c)) / w;
    const double new_vx = vx * c + vy * s;
    const double new_vy = vy * c - vx * s;
    vx = new_vx;
    vy = new_vy;
}

double Target::range() const {
    return std::hypot(x, y);
}

double Target::speed() const {
    return std::hypot(vx, vy);
}

}  // namespace radar::sim
