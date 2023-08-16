#ifndef __LONGITUDINAL_CONTROL_H__
#define __LONGITUDINAL_CONTROL_H__

#include "control_node/pid/pid.h"
#include "control_node/math_functions/math_functions.h"

class LongitudinalControl {

    PID control;

    double throttle;
    double brake;

public: 
    LongitudinalControl(double _dt, double _max, double _min, double _Kp, double _Kd, double _Ki);
    void SetThrottleBrake(double in_val, double meas_val);
    void SetVal(double& _throttle, double& _brake);
};


#endif // __LONGITUDINAL_CONTROL_H__