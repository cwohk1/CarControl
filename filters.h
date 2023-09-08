#ifndef FILTERS_H
#define FILTERS_H

#include <stdlib.h>
#define CUTOFF_FREQ 35.
#define PI 3.1415926535897932384626433832795
#define LOOPCOUNT 5
#define DEFAULT_SPEED 5

float lambda, dt = 1/25.;// frequency of IR sensor
// 1차 저주파통과필터
typedef struct {
    float RC;
    float alpha;
    float prev_input;
    float prev_output;
} Filter;

typedef struct {
    Filter filter;
    float history[LOOPCOUNT];
    int curr_point;
    float velocity[LOOPCOUNT]; // 대략 1.5*dt 전의 속도
    float acceleration; // 대략 3*dt 전의 가속도
    float max_distance;
} Killspike;

// 헤더파일 함수들을 사용하기 전에 필터를 초기화해줘야함
void initFilter(Filter *filter, float initial_distance) {
    filter->RC = 1.0/(2*PI*CUTOFF_FREQ);
    filter->alpha = filter->RC / (filter->RC+dt);
    filter->prev_input = initial_distance;
    filter->prev_output = initial_distance;
}

void initKillspike(Killspike *filter, float initial_distance, float max_distance, bool lpf) {
    if (lpf) initFilter(&filter->filter, initial_distance);
    filter->curr_point = 0;
    for (int i = 0; i < LOOPCOUNT; ++i) {
        filter->history[i] = initial_distance;
        filter->velocity[i] = 0;
    }
    filter->acceleration = 0;
    filter->max_distance = max_distance;
}

float lowPassFilter(Filter* filter, float input) {
    filter->prev_output = (1-filter->alpha)*input + (filter->alpha)*filter->prev_output;
    return filter->prev_output;
}

float killspikefilter(Killspike *filter, float input) {
    filter->history[filter->curr_point] = input;
    filter->curr_point = (filter->curr_point + 1) % LOOPCOUNT;
    float sum = 0;
    for (int i = 0; i < LOOPCOUNT; ++i) {
        sum += filter->history[i];
    }
    filter->prev_output = sum / LOOPCOUNT;
    return filter->prev_output;
}

#endif
