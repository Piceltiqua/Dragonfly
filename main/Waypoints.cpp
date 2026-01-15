#include "Waypoints.hpp"
#include <Eigen/LU>

void WaypointManager::init() {
    // Example waypoints: time (s), posN (m), posE (m), posD (m)
    waypoints_.push_back({0.0f,  0.0f, 0.0f,  0.0f});
    waypoints_.push_back({5.0f,  0.0f, 0.0f, -1.0f});
    waypoints_.push_back({10.0f, 1.0f, 0.0f, -1.0f});
    waypoints_.push_back({15.0f, 1.0f, 0.0f,  0.0f});

    // Check all waypoint segments for feasibility
    for (size_t i = 1; i < waypoints_.size(); ++i) {
        bool valid = waypointParameters(waypoints_[i], waypoints_[i - 1]);
        if (!valid) {
            invalid = true;
            Serial.println("Waypoint segment " + String(i-1) + " to " + String(i) + " is not feasible with given constraints.");
            return;
        }
    }

    previous_waypoint_index_ = 0;
    current_waypoint_index_ = 1;
    waypointParameters(waypoints_[current_waypoint_index_], waypoints_[previous_waypoint_index_]);
}

void WaypointManager::trajectoryControl(float flight_time) {
    if (invalid) {
        position_setpoint_.posN = 0.0f;
        position_setpoint_.posE = 0.0f;
        position_setpoint_.posD = 0.0f;
        position_setpoint_.velN = 0.0f;
        position_setpoint_.velE = 0.0f;
        position_setpoint_.velD = 0.0f;
        return;
    }

    if (current_waypoint_index_ >= waypoints_.size()) {
        position_setpoint_.velN = 0.0f;
        position_setpoint_.velE = 0.0f;
        position_setpoint_.velD = 0.0f;
        return;
    }

    Waypoint& waypoint = waypoints_[current_waypoint_index_];
    Waypoint& previous_waypoint = waypoints_[previous_waypoint_index_];

    if (flight_time >= waypoint.time) {
        // Move to next waypoint
        current_waypoint_index_++;
        previous_waypoint_index_ ++;
        if (current_waypoint_index_ < waypoints_.size()) {
            waypoint = waypoints_[current_waypoint_index_];
            previous_waypoint = waypoints_[previous_waypoint_index_];
            waypointParameters(waypoint, previous_waypoint);
        } else {
            // Last waypoint reached
            position_setpoint_.velN = 0.0f;
            position_setpoint_.velE = 0.0f;
            position_setpoint_.velD = 0.0f;
            return;
        }
    }

    float t = flight_time - previous_waypoint.time;
    if (t < Ta)                 {v = ACC_RATE * t;
                                 s = 0.5f * ACC_RATE * t * t;}
    else if (t < Ta + Tc)       {v = Vc;
                                 s = 0.5f * ACC_RATE * Ta * Ta + Vc * (t - Ta);}
    else if (t < Ta + Tc + Td)  {v = Vc - DEC_RATE * (t - Ta - Tc);
                                 float td = t - Ta - Tc;
                                 s = 0.5f * ACC_RATE * Ta * Ta + Vc * Tc + Vc * td - 0.5f * DEC_RATE * td * td;}
    else                        {v = 0.0f;
                                 s = S;}

    position_setpoint_.posN = previous_waypoint.posN + s * ux;
    position_setpoint_.posE = previous_waypoint.posE + s * uy;
    position_setpoint_.posD = previous_waypoint.posD + s * uz;
    position_setpoint_.velN = v * ux;
    position_setpoint_.velE = v * uy;
    position_setpoint_.velD = v * uz;
}

bool WaypointManager::waypointParameters(const Waypoint& target, const Waypoint& start) {
    // Define acceleration time, deceleration time, and cruise time and cruise velocity
    float T = target.time - start.time;
    float C = (1.0f / ACC_RATE) + (1.0f / DEC_RATE);

    float dx = target.posN - start.posN;
    float dy = target.posE - start.posE;
    float dz = target.posD - start.posD;
    S = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (S < 1e-6f) {
        ux = uy = uz = 0.0f;
        Ta = Tc = Td = 0.0f;
        Vc = 0.0f;
        return true;
    }

    ux = dx / S;
    uy = dy / S;
    uz = dz / S;
    
    if (T*T - 2.0f*C*S < 0) {
        Serial.println("Trajectory is not feasible with given constraints.");
            return false;
    }

    float Vc = (T - std::sqrt(T*T - 2.0f*C*S)) / C;

    if (0 < Vc && Vc < MAX_VELOCITY) {
        Ta = Vc / ACC_RATE;
        Td = Vc / DEC_RATE;
        Tc = T - Ta - Td;
        return true;
    }

    else {
        // Clamp to max speed
        Vc = MAX_VELOCITY;

        Ta = Vc / ACC_RATE;
        Td = Vc / DEC_RATE;
        Tc = T - Ta - Td;

        if (Tc < 0.0f) {
            Serial.println("Trajectory is not feasible with given constraints.");
            return false;
        }
        return true;
    }
}
