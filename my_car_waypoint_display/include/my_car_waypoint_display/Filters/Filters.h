/*
 * 여러 번 쓰일 것 같아서 만듬
 * Date      : 230816
 * Author    : 박성훈       
 */ 

#ifndef __MOVING_AVERAGE_H__
#define __MOVING_AVERAGE_H__

#include <vector>

class MovingAverage {
    
    std::vector<double> window;
public:
    MovingAverage(int window_size, double ref);
    double Filter(double input);
};



double GaussianFilter(std::vector<double> vec, int window_size, int idx);

#endif // __MOVING_AVERAGE_H__