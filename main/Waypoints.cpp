#include "Waypoints.hpp"

bool WaypointManager::init() {
    // Waypoints: time (s), posN (m), posE (m), posD (m)
    waypoints_.push_back(Waypoint{0.0f,  0.0f, 0.0f,  0.0f});
    waypoints_.push_back(Waypoint{1.0f,  0.0f, 0.0f,  0.0f});
    waypoints_.push_back(Waypoint{7.0f,  0.0f, 0.0f, -1.0f});
    waypoints_.push_back(Waypoint{15.0f, 0.0f, 0.0f, -1.0f});
    waypoints_.push_back(Waypoint{25.0f, 0.0f, 0.0f, -0.0f});
    waypoints_.push_back(Waypoint{30.0f, 0.0f, 0.0f, -0.0f});

    // Check all waypoint segments for feasibility
    for (size_t i = 1; i < waypoints_.size(); ++i) {
        bool valid = waypointParameters(waypoints_[i], waypoints_[i - 1]);

        if (!valid) {
            invalid = true;
            Serial.println("Waypoint segment " + String(i-1) + " to " + String(i) + " is not feasible with given constraints.");
            return false;
        }
    }

    // Check if last waypoint is on the ground
    const Waypoint& last_wp = waypoints_.back();
    if (last_wp.posD < -0.0f) {
        invalid = true;
        Serial.println("Last waypoint must be on the ground (posD >= 0).");
        return false;
    }

    previous_waypoint_index_ = 0;
    current_waypoint_index_ = 1;
    waypointParameters(waypoints_[current_waypoint_index_], waypoints_[previous_waypoint_index_]);

    return true;
}

bool WaypointManager::flying(float flight_time) {
    if (invalid) {
        position_setpoint_.posN = 0.0f;
        position_setpoint_.posE = 0.0f;
        position_setpoint_.posD = 0.0f;
        position_setpoint_.velN = 0.0f;
        position_setpoint_.velE = 0.0f;
        position_setpoint_.velD = 0.0f;
        return false;
    }

    if (current_waypoint_index_ >= waypoints_.size()) {
        position_setpoint_.velN = 0.0f;
        position_setpoint_.velE = 0.0f;
        position_setpoint_.velD = 0.0f;
        return false;
    }


    // Advance waypoint if needed
    if (flight_time >= waypoints_[current_waypoint_index_].time) {
        previous_waypoint_index_++;
        current_waypoint_index_++;

        if (current_waypoint_index_ >= waypoints_.size()) {
            position_setpoint_.velN = 0.0f;
            position_setpoint_.velE = 0.0f;
            position_setpoint_.velD = 0.0f;
            return false;
        }

        waypointParameters(
            waypoints_[current_waypoint_index_],
            waypoints_[previous_waypoint_index_]
        );
    }

    const Waypoint& waypoint          = waypoints_[current_waypoint_index_];
    const Waypoint& previous_waypoint = waypoints_[previous_waypoint_index_];

    float a = 0.0f;
    float v = 0.0f;
    float s = 0.0f;

    float t = flight_time - previous_waypoint.time;

    // Position and accelerations are not delayed
    if (t < Ta)                 {a = ACC_RATE;
                                 s = 0.5f * ACC_RATE * t * t;}
    else if (t < Ta + Tc)       {a = 0;
                                 s = 0.5f * ACC_RATE * Ta * Ta + Vc * (t - Ta);}
    else if (t < Ta + Tc + Td)  {a = -DEC_RATE;
                                 float td = t - Ta - Tc;
                                 s = 0.5f * ACC_RATE * Ta * Ta + Vc * Tc + Vc * td - 0.5f * DEC_RATE * td * td;}
    else                        {a = 0;
                                 s = S;}

    // Velocity setpoints are delayed by VEL_DELAY seconds
    if (t - VEL_DELAY < 0)                  {v = 0.0f;}
    else if (t - VEL_DELAY < Ta)            {v = ACC_RATE * (t - VEL_DELAY);}
    else if (t - VEL_DELAY < Ta + Tc)       {v = Vc;}
    else if (t - VEL_DELAY < Ta + Tc + Td)  {v = Vc - DEC_RATE * (t - VEL_DELAY - Ta - Tc);}
    else                                    {v = 0.0f;}

    position_setpoint_.posN = previous_waypoint.posN + s * ux;
    position_setpoint_.posE = previous_waypoint.posE + s * uy;
    position_setpoint_.posD = previous_waypoint.posD + s * uz;
    position_setpoint_.velN = v * ux;
    position_setpoint_.velE = v * uy;
    position_setpoint_.velD = v * uz;
    position_setpoint_.accffN = a * ux;
    position_setpoint_.accffE = a * uy;
    position_setpoint_.accffD = a * uz;

    return true;
}

bool WaypointManager::waypointParameters(const Waypoint& target, const Waypoint& start) {
    // Define acceleration time, deceleration time, and cruise time and cruise velocity
    float T = target.time - start.time - VEL_DELAY;
    float C = (1.0f / ACC_RATE) + (1.0f / DEC_RATE);

    float dx = target.posN - start.posN;
    float dy = target.posE - start.posE;
    float dz = target.posD - start.posD;
    S = sqrt(dx * dx + dy * dy + dz * dz);

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

    Vc = (T - sqrt(T*T - 2.0f*C*S)) / C;

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
