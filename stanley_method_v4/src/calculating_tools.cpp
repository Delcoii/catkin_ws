#include "stanley_method_v4/calculating_tools.h"


double rad2deg(double rad) {
    return rad*180./M_PI;
}

double deg2rad(double deg) {
    return deg*(M_PI/180.);
}

double cut(double value, double max, double min) {
    if (value >= max)
        return max;
    else if (value <= min)
        return min;
    else
        return value;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


double avg_filter(double input) {
    static double avg = 0.;
    static int sample_count = 0;

    sample_count++;

    // calculating alpha = k-1 / k
    double alpha = (double)(sample_count-1) / (double)sample_count;

    // calculating average
    avg = alpha * avg + (1.-alpha) * input;

    return avg;
}