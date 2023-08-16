/*
 * 여러 번 쓰일 것 같아서 만듬
 * Date      : 230816
 * Author    : 박성훈
 * 
 * to do : 
 * include 경로 지정.. ros library including에 대한 이해 필요 ㅠㅠ 
 */

#include "control_node/MovingAverage/MovingAverage.h"

MovingAverage::MovingAverage(int window_size) :
    window(window_size, 0) {}


double MovingAverage::Filter(double input) {
    window.erase(window.begin());
    window.push_back(input);

    double sum = 0;

    for (int idx = 0; idx < window.size(); idx++) {
        sum += window[idx];    
    }
    return sum / window.size();
}