#ifndef FILTERS_H
#define FILTERS_H

#include <stdlib.h>
#define CUTOFF_FREQ 35.
#define PI 3.1415926535897932384626433832795
#define INITIAL_DISTANCE 70.0

float lambda, Ts = 1/18.;// frequency of IR sensor
// 1차 저주파통과필터
typedef struct {
    float prev_input;
    float prev_output;
} Filter;

typedef struct {
    float x;  // State estimate
    float P;  // Estimate covariance
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
    float prev_input[3], prev_output[3];
} KalmanFilter;

// 헤더파일 함수들을 사용하기 전에 필터를 초기화해줘야함
void initFilter(Filter *filter) {
    lambda = CUTOFF_FREQ*2*PI;
    Ts = 1/18.;
    filter->prev_input = INITIAL_DISTANCE;
    filter->prev_output = INITIAL_DISTANCE;
}

// 칼만 필터 초기화
//     KalmanFilter kf;
//    initKalmanFilter(&kf, 0.0, 1.0, 0.001, 0.01);
void initKalmanFilter(KalmanFilter *kf, float initial_state, float initial_covariance,
                      float process_noise, float measurement_noise) {
    lambda = CUTOFF_FREQ*2*PI;
    Ts = 1/18.;
    kf->x = initial_state;
    kf->P = initial_covariance;
    kf->Q = process_noise;
    kf->R = measurement_noise;
    kf->prev_input[0] = INITIAL_DISTANCE;
    kf->prev_input[1] = INITIAL_DISTANCE;
    kf->prev_input[2] = INITIAL_DISTANCE;
    kf->prev_output[0] = INITIAL_DISTANCE;
    kf->prev_output[1] = INITIAL_DISTANCE;
    kf->prev_output[2] = INITIAL_DISTANCE;
}

// 자동완성으로 나온 필터, dt는 샘플링 주기
// 1차 칼만 필터
float lowPassFilter_1(Filter* filter, float input) {
    float dt = Ts;
    float RC = 1.0/(2*PI*CUTOFF_FREQ);
    float alpha = dt/(RC+dt);
    filter->prev_output = alpha*input + (1-alpha)*filter->prev_output;
    return filter->prev_output;
}

// 1차 저주파통과필터
float lowPassFilter_2(Filter* filter, float input) {
    return (filter->prev_output + input-filter->prev_input)/(1+lambda*Ts);
}

// 3rd order Kalman Filter
float updateKalmanFilter(KalmanFilter *kf, float input) {
    // Prediction step
    float predicted_x = kf->x;
    float predicted_P = kf->P + kf->Q;

    // Update step
    float innovation = input - (kf->prev_input[0] + kf->prev_input[1] + kf->prev_input[2]) / 3.0;
    float innovation_covariance = predicted_P + kf->R;

    float kalman_gain = predicted_P / innovation_covariance;

    kf->x = predicted_x + kalman_gain * innovation;
    kf->P = (1 - kalman_gain) * predicted_P;

    // Return the filtered output
    float filtered_output = kf->x;

    // Update the previous input and output
    kf->prev_input[2] = kf->prev_input[1];
    kf->prev_input[1] = kf->prev_input[0];
    kf->prev_input[0] = input;
    kf->prev_output[2] = kf->prev_output[1];
    kf->prev_output[1] = kf->prev_output[0];
    kf->prev_output[0] = filtered_output;
    return filtered_output;
}
#endif