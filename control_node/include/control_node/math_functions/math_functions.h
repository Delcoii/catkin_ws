#ifndef __MATH_FUNCTIONS_H__
#define __MATH_FUNCTIONS_H__
#include <ros/ros.h>

double deg2rad(double deg);
double rad2deg(double rad);
double map(double x, double in_min, double in_max, double out_min, double out_max);
double CutMinMax(double x, double min, double max);

#endif // __MATH_FUNCTIONS_H__