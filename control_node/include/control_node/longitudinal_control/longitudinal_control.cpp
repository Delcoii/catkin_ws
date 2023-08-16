#include "control_node/longitudinal_control/longitudinal_control.h"

LongitudinalControl::LongitudinalControl(double _dt, double _max, double _min, double _Kp, double _Kd, double _Ki) :
    control(_dt, _max, _min, _Kp, _Kd, _Ki) {
    
    throttle = 0.;
    brake = 0.;
}

void LongitudinalControl::SetThrottleBrake(double in_val, double meas_val) {
    double temp_val = control.calculate(in_val, meas_val);

    if (temp_val > 0.) {
        throttle = temp_val;
        brake = 0.;
    }
    else {
        throttle = 0.;
        brake = abs(temp_val);
    }
}

void LongitudinalControl::SetVal(double& _throttle, double& _brake) {
    _throttle = throttle;
    _brake = brake;
}