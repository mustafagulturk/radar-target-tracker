#pragma once

#include <cstdint>

namespace radar::sim {

/// Ground-truth kinematics of a simulated target.
///
/// Motion is a coordinated turn with constant speed and turn rate; a turn
/// rate of zero gives straight, constant-velocity flight.
struct Target {
    std::uint32_t id = 0;
    double x = 0.0;              ///< East, metres.
    double y = 0.0;              ///< North, metres.
    double vx = 0.0;             ///< East velocity, m/s.
    double vy = 0.0;             ///< North velocity, m/s.
    double turn_rate_rps = 0.0;  ///< Positive turns clockwise (to starboard).
    double rcs_db = 10.0;        ///< Returned amplitude, used for display only.

    /// Advances the target by `dt_s` seconds along its exact trajectory.
    void advance(double dt_s);

    double range() const;
    double speed() const;
};

}  // namespace radar::sim
