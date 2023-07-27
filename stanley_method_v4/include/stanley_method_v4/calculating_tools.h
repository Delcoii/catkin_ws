#ifndef __CALCULATING_TOOLS_H__
#define __CALCULATING_TOOLS_H__

#include <ros/ros.h>

double rad2deg(double rad);
double deg2rad(double deg);
double cut(double value, double max, double min);
double map(double x, double in_min, double in_max, double out_min, double out_max);
double avg_filter(double input);


#endif // __CALCULATING_TOOLS_H__