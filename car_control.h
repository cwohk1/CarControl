#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define Thr_PWM 2
#define Ser_PWM 5
#define NUM_SENSORS 11	// number of IR sensors
#define SENSOR_LOG_LEN 10
#define FORWARD analogWrite(Thr_PWM, 3200)
#define REVERSE analogWrite(Thr_PWM, 2800)
#define STOP analogWrite(Thr_PWM, 3000)
#define RIGHT analogWrite(Ser_PWM, 2240)
#define LEFT analogWrite(Ser_PWM, 3858)
#define CENTER analogWrite(Ser_PWM, 3005)
#define FORWARD_LEFT analogWrite(Thr_PWM, 3200); analogWrite(Ser_PWM, 3858)
#define FORWARD_RIGHT analogWrite(Thr_PWM, 3200); analogWrite(Ser_PWM, 2240)
#define FORWARD_CENTER analogWrite(Thr_PWM, 3200); analogWrite(Ser_PWM, 3005)
#define FORWARDX(X) analogWrite(Thr_PWM, X)
#define FORWARDXY(X, Y) analogWrite(Thr_PWM, X); analogWrite(Ser_PWM, Y)
#define MAX_SPEED 3300
#define IR_THRESHOLD 40.0
#define IR_MAX 80.0
#define FRONT_IR_MAX 140.0
#define IR_MID 60.0
#define IR_MIN 20.0
#define STOP_DISTANCE 25.0
#define MIN_SPEED 3000
#define DECISION_THRESHOLD 2
#ifndef max
#define max(a,b)  (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)  (((a) < (b)) ? (a) : (b))
#endif
#ifndef sign
#define sign(x) x > 0 ? 1 : (x == 0 ? 0 : -1)
#ifndef CAR_LENGHT_INFO
#define CAR_LENGTH_INFO
#define SENSOR_DISTANCE_0TO1 10
#endif
int sensor_group[NUM_SENSORS] = {-3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3}; // group the sensors by direction left to right -3 ~ 3
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
    bool left_open, right_open, front_open, front_left_open, front_right_open;
    // check if RC car is following a line
    bool following_left_line, following_right_line;
    // check if RC car is in a narrow passage
    bool narrow_passage;
    // check if RC car is in an open field, DECISION_THRESHOLD consecutive trues to become true
    unsigned int open_field;
    // check if RC car needs to avoid an obstacle, DECISION_THRESHOLD consecutive falses to become false
    int avoid_obstacle;
    // check if RC car is in an emergency stop, 2 consecutive trues to become true
    unsigned int emergency_stop;
} CarControl;

int get_idx(const int &curr, const int &tick) {
    return curr-tick<0 ? curr+SENSOR_LOG_LEN - tick : curr - tick;
}
int initCarControl(CarControl* carcontrol, int len) {
    carcontrol->len = len;
    carcontrol->irSensor = static_cast<float**>(malloc(len * sizeof(float*)));
    for(int i=0; i < len; ++i)
        carcontrol->irSensor[i] = static_cast<float*>(malloc(SENSOR_LOG_LEN * sizeof(float)));
    carcontrol->emergency_stop = 0;
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
    // intirialize open direction
    carcontrol->left_open = true;
    carcontrol->right_open = true;
    carcontrol->front_open = true;
    carcontrol->front_left_open = true;
    carcontrol->front_right_open = true;

    // check if RC car is following a line
    carcontrol->following_left_line = false;
    carcontrol->following_right_line = false;
    carcontrol->narrow_passage = false;

}

int destroyCarControl(CarControl* carcontrol) {
    for(int i=0; i < carcontrol->len; ++i)
        free(carcontrol->irSensor[i]);
    free(carcontrol->irSensor);
}

double getDistance(CarControl* carcontrol, int idx){
    if (idx < carcontrol->len)
        return carcontrol->irSensor[idx][get_idx(carcontrol->curr_point, 1)];
    else
        return NULL;
}

double getIRValue(CarControl* carcontrol, const int &idx, const int &tick){
    // check if tick is valid
    if (tick > idx+int(carcontrol->rotated)*SENSOR_LOG_LEN || idx >= carcontrol->len)
        return NULL; 
    else
        return carcontrol->irSensor[idx][get_idx(carcontrol->curr_point, tick)];
}

// acc -1 ~ 1, handling -30 ~ 30
void control_refined(float acc, float handling) {
    const int acc_min = 2000, acc_max = 4094, handling_min = 2240, handling_max = 3858;
    analogWrite(Thr_PWM, max(min(map(acc, -1., 1., acc_min, acc_max), MAX_SPEED), MIN_SPEED));
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
