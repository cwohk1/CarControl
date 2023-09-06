#ifndef FILTERS_H
#define FILTERS_H

#include <stdlib.h>
#define CUTOFF_FREQ 35.
#define PI 3.1415926535897932384626433832795

float lambda, dt = 1/18.;// frequency of IR sensor
// 1차 저주파통과필터
typedef struct {
    float prev_input;
    float prev_output;
} Filter;

// 헤더파일 함수들을 사용하기 전에 필터를 초기화해줘야함
void initFilter(Filter *filter, float initial_distance) {
    filter->prev_input = initial_distance;
    filter->prev_output = initial_distance;
}

// 자동완성으로 나온 필터, dt는 샘플링 주기
// 1차 칼만 필터
float lowPassFilter_1(Filter* filter, float input) {
    float RC = 1.0/(2*PI*CUTOFF_FREQ);
    float alpha = RC/(RC+dt);
    filter->prev_output = (1-alpha)*input + (alpha)*filter->prev_output;
    return filter->prev_output;
}

#endif
