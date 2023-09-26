#ifndef FILTERS_H
#define FILTERS_H

#include <stdlib.h>
#define CUTOFF_FREQ 35.
#define PI 3.1415926535897932384626433832795
#define LOOPCOUNT 10
#define DEFAULT_SPEED 5

float lambda, filter_T = 1/25.;// IR sensor
float dt = 0.0071; // code running time.
// float dt = 0.001;
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
    int lpf, max_velocity, min_velocity; // 저주파통과필터, 속도제한 설정
} Killspike;

// 헤더파일 함수들을 사용하기 전에 필터를 초기화해줘야함
void initFilter(Filter *filter, float initial_distance) {
    filter->RC = 1.0/(2*PI*CUTOFF_FREQ);
    filter->alpha = filter->RC / (filter->RC+filter_T);
    filter->prev_input = initial_distance;
    filter->prev_output = initial_distance;
}

void initKillspike(Killspike *filter, float initial_distance, float max_distance, bool lpf, float max_velocity, float min_velocity) {
    if (lpf) initFilter(&filter->filter, initial_distance);
    filter->lpf = lpf;
    filter->curr_point = 0;
    for (int i = 0; i < LOOPCOUNT; ++i) {
        filter->history[i] = initial_distance;
        filter->velocity[i] = 0;
    }
    filter->acceleration = 0;
    filter->max_distance = max_distance;
    filter->max_velocity = max_velocity;
    filter->min_velocity = min_velocity;
}

// alpha가 0.1
float lowPassFilter(Filter* filter, float input) {
    filter->prev_output = (1-filter->alpha)*input + (filter->alpha)*filter->prev_output;
    return filter->prev_output;
}

float killspikefilter(Killspike *filter, float input) {
    float ret = 0;
    int prev_idx = (filter->curr_point-1) < 0 ? LOOPCOUNT-1 : filter->curr_point-1;
    if(filter->lpf) filter->history[filter->curr_point] = lowPassFilter(&(filter->filter), input);
    else filter->history[filter->curr_point] = input;
    float sum = 0, min_val = 150;
    for (int i = 0; i < LOOPCOUNT; ++i) {
        if(filter->history[i] < min_val) min_val = filter->history[i];
        sum += filter->history[i];
    }
    sum -= min_val;
    if(filter->lpf){
        if(filter->history[filter->curr_point] > min_val) 
            ret = filter->history[filter->curr_point];
        else ret = filter->history[prev_idx] = min_val;
    }
    else ret = sum/(LOOPCOUNT-1);
    filter->velocity[filter->curr_point] = (ret + filter->history[prev_idx] \
    - filter->history[(filter->curr_point-2) < 0 ? LOOPCOUNT+filter->curr_point-2 : filter->curr_point-2] \
    - filter->history[(filter->curr_point-3) < 0 ? LOOPCOUNT+filter->curr_point-3 : filter->curr_point-3])/dt/2;
    filter->acceleration = (filter->velocity[filter->curr_point] + filter->velocity[prev_idx])/dt;
    filter->curr_point = (filter->curr_point+1) % LOOPCOUNT;
    return ret;
}
#endif
