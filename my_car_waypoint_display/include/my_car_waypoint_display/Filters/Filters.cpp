/*
 * 여러 번 쓰일 것 같아서 만듬
 * Date      : 230816
 * Author    : 박성훈
 * 
 * to do : 
 * include 경로 지정.. ros library including에 대한 이해 필요 ㅠㅠ 
 */

#include "my_car_waypoint_display/Filters/Filters.h"

MovingAverage::MovingAverage(int window_size, double ref) :
    window(window_size, ref) {}


double MovingAverage::Filter(double input) {
    window.erase(window.begin());
    window.push_back(input);

    double sum = 0;

    for (int idx = 0; idx < window.size(); idx++) {
        sum += window[idx];    
    }
    return sum / window.size();
}





double GaussianFilter(std::vector<double> vec, int window_size, int idx) {
    double sum = 0.;
    if (window_size % 2 == 0) {
        for (int i = idx-(window_size/2); i < idx+(window_size/2); i++) {
            sum += vec[i];
        }
        return sum / (double)window_size;
    }

    else {
        for (int i = idx-(window_size/2); i <= idx+(window_size/2); i++) {
            sum += vec[i];
        }
        return sum / (double)window_size;
    }
}