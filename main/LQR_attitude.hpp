#ifndef LQR_attitude_HPP
#define LQR_attitude_HPP

#include "Utils.hpp"
#include <Eigen/LU>


class LQR_attitude {
public:
    LQR_attitude();
    void init( Eigen::Matrix<float, 2, 4> K_init);
    void compute(const Attitude_angle& attitude,
                 const rotationspeed& rotSpeed, float thrust, float moment_arm,
                 const AttitudeSetpoint& attSetpoint,
                 ControlOutput_attitude& ctrlOutput);
    
private:
    Eigen::Matrix<float, 2, 4> K; 


};



#endif // LQR_attitude_HPP 