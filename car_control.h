#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "dummy.h"
#define Thr_PWM 2
#define Ser_PWM 5
#ifndef IR_VALUES
#define IR_VALUES
#define IR_OFFSET 7.0
#define IR_THRESHOLD 60.0
#define IR_MAX 140.0
#define FRONT_IR_MAX 160.0
#define IR_MID 90.0
#define IR_MIN 20.0
#define STOP_DISTANCE 30.0
#define NUM_SENSORS 11	// number of IR sensors
#endif
#define SENSOR_LOG_LEN 3
#define MAX_SPEED 3250
#define MIN_SPEED 2750
#define DECISION_THRESHOLD 2
#ifndef max
#define max(a,b)  (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)  (((a) < (b)) ? (a) : (b))
#endif
#ifndef sign
#define sign(x) x > 0 ? 1 : (x == 0 ? 0 : -1)
#endif
#ifndef CAR_LENGHT_INFO
#define CAR_LENGTH_INFO
#define SENSOR_DISTANCE_0TO1 5.0
#endif
int sensor_group[11] = {-3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3}; // group the sensors by direction left to right -3 ~ 3
typedef struct {
    float angle, speed;
    bool emergency_stop = false;
} CarObjective;

typedef struct {
    float** irSensor;
    int len;
    int curr_point = 0;
    bool rotated = false;
    CarObjective objective[2]; // 0: current, 1: before    
    bool emergency_stop;
    bool left_open, right_open, front_open, front_left_open, front_right_open;
    // check if RC car is following a line
    bool following_left_line, following_right_line;
    // check if RC car is in a narrow passage
    bool narrow_passage;
    // check if RC car is in an open field, DECISION_THRESHOLD consecutive trues to become true
    unsigned int open_field;
    // check if RC car needs to avoid an obstacle, DECISION_THRESHOLD consecutive falses to become false
    int avoid_obstacle;
} CarControl;

int get_idx(const int &curr, const int &tick) {
    return curr-tick<0 ? curr+SENSOR_LOG_LEN - tick : curr - tick;
}
int initCarControl(CarControl* carcontrol, int len) {
    carcontrol->len = len;
    carcontrol->irSensor = static_cast<float**>(malloc(len * sizeof(float*)));
    for(int i=0; i < len; ++i)
        carcontrol->irSensor[i] = static_cast<float*>(malloc(SENSOR_LOG_LEN * sizeof(float)));
    carcontrol->emergency_stop = false;
    carcontrol->avoid_obstacle = DECISION_THRESHOLD;
    carcontrol->open_field = 0;
    carcontrol->curr_point = 0;
    carcontrol->rotated = false;
    carcontrol->objective[0].angle = 0.;
    carcontrol->objective[0].speed = 0.;
    carcontrol->objective[1].angle = 0.;
    carcontrol->objective[1].speed = 0.;
    carcontrol->objective[0].emergency_stop = false;
    carcontrol->objective[1].emergency_stop = false;
}

void updateCarControl(CarControl* carcontrol, float* values) {
    // update sensor values
    for (int i=0; i<carcontrol->len; ++i){
        carcontrol->irSensor[i][carcontrol->curr_point] = values[i];
    }
    if (carcontrol->curr_point>= SENSOR_LOG_LEN-1) {
        carcontrol->curr_point == 0;
        carcontrol->rotated = true;
    }
    else {
        carcontrol->curr_point++;
    }
    // check if RC car is following a line&&
    if(!(values[1] < 60 && values[2] < IR_MID && values[3] < IR_MID && values[4] > IR_MID && values[5] > IR_MID && values[6] > IR_MID && carcontrol->right_open))
        carcontrol->following_left_line = false;
    if(!(values[9] < 60 && values[8] < IR_MID && values[7] < IR_MID && values[4] > IR_MID && values[5] > IR_MID && values[6] > IR_MID && carcontrol->left_open))
        carcontrol->following_right_line = false;
    carcontrol->narrow_passage = false;

    // intitialize open direction
    carcontrol->left_open = true;
    carcontrol->right_open = true;
    carcontrol->front_open = true;
    carcontrol->front_left_open = true;
    carcontrol->front_right_open = true;
}

int destroyCarControl(CarControl* carcontrol) {
    for(int i=0; i < carcontrol->len; ++i)
        free(carcontrol->irSensor[i]);
    free(carcontrol->irSensor);
}

// acc -1 ~ 1, handling -30 ~ 30
void control_refined(float acc, float handling) {
    const int acc_min = 2000, acc_max = 4094, handling_min = 2240, handling_max = 3858;
    analogWrite(Thr_PWM, max(min(map(acc, -1., 1., acc_min, acc_max), MAX_SPEED), MIN_SPEED));
    handling = max(min(handling, 30), -30);
    analogWrite(Ser_PWM, map(handling, -30, 30, handling_min, handling_max));
    return;
}

void updateObjective(CarControl* carcontrol) {
    // update objective
    carcontrol->objective[1].angle = carcontrol->objective[0].angle;
    carcontrol->objective[1].speed = carcontrol->objective[0].speed;
    // update emergency stop
    carcontrol->objective[1].emergency_stop = carcontrol->objective[0].emergency_stop;
}
#endif
