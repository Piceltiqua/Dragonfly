#ifndef Waypoints_HPP
#define Waypoints_HPP

#include "Utils.hpp"
#include <vector>

class WaypointManager {
public:
    WaypointManager(PosCtrlSetpoint& position_setpoint)
        : position_setpoint_(position_setpoint) {}

    bool init();
    bool flying(float flight_time);
    bool waypointParameters(const Waypoint& target, const Waypoint& start);
private:
    PosCtrlSetpoint& position_setpoint_;

    std::vector<Waypoint> waypoints_;
    bool invalid = false;
    size_t previous_waypoint_index_ = 0;
    size_t current_waypoint_index_ = 1;

    static constexpr float MAX_VELOCITY = 0.5f; // m/s
    static constexpr float ACC_RATE = 0.2f;     // m/s²
    static constexpr float DEC_RATE = 0.2f;     // m/s²
    static constexpr float VEL_DELAY = 0.6f;    // s

    float Vc = 0.0f; // Cruise velocity
    float Ta = 0.0f; // Acceleration time
    float Td = 0.0f; // Deceleration time
    float Tc = 0.0f; // Cruise time
    float S  = 0.0f; // Total distance

    // Unit direction vector components
    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
};

#endif