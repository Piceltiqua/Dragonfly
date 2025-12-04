#ifndef LQR_position_HPP
#define LQR_position_HPP

#include "Utils.hpp"
#include <Eigen/LU>


class LQR_position {
public:
    LQR_position();
    void init( Eigen::Matrix<float, 2, 4> K_init);
    void compute(const Attitude_angle& attitude,
                 const rotationspeed& rotSpeed,
                 const AttitudeSetpoint& attSetpoint,
                 ControlOutput_attitude& ctrlOutput);
    
private:
    Eigen::Matrix<float, 2, 4> K; 


};







#endif // LQR_position_HPP 