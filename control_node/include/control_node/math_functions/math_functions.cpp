#include "control_node/math_functions/math_functions.h"

double deg2rad(double deg) {
    return deg * M_PI/180.;
}

double rad2deg(double rad) {
    return rad * 180./M_PI;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double CutMinMax(double x, double min, double max) {
    if (x >= max)
        return max;
    else if (x <= min)
        return min;
    else
        return x;
}