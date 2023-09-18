// #include "dummy.h"
#include "dummy.h"
#include "car_control.h"
#include "filters.h"
#include "scan_environment.h"
// #include "scan_environment.h"
#define STOP analogWrite(Thr_PWM, 3005)
#define CENTER analogWrite(Ser_PWM, 3005)
#define FAST 3250
#define MEDIUM 3240
#define SLOW 3230
#define REVERSE 2800
#ifndef IR_VALUES
#define IR_VALUES
#define IR_OFFSET 7.0
#define IR_THRESHOLD 60.0
#define IR_MAX 140.0
#define FRONT_IR_MAX 160.0
#define IR_MID 90.0
#define IR_MIN 20.0
#define STOP_DISTANCE 30.0
#define NUM_SENSORS 11  // number of IR sensors
#endif
#ifndef Thr_PWM
#define Thr_PWM 2
#endif
#ifndef Ser_PWM
#define Ser_PWM 5
#endif
#ifndef DDEBUG
#define DDEBUG 0
#endif
#define SIMPLE_MODE 0

//int pin_list[11] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};
int pin_list[NUM_SENSORS] = {A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
// int sensor_type[NUM_SENSORS] = {0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1};  // 0: long, 1: short
int sensor_type[NUM_SENSORS] = {0};
CarControl carcontrol;
Filter filter[NUM_SENSORS];
Killspike killspike[NUM_SENSORS];
float ir[NUM_SENSORS] = {IR_MAX};  // array for IR distance values
float prev_ir_avg = 0;
int system_stop_flag = 0;
// analogWrite(PinNum, desired us / 2 -> (Ex: To get 900us -> 1800))
// Analog Read 10 bit (based 5V), so result value 1023 is 5V

// THR: 2000(Reverse) ~ 3000(Stay) ~ 4094(Full)
// SER: 2240(R) ~ 3003(Center) ~ 3858(L)

void setup() {
  // RTM_TimerCalc 1.40, RuntimeMicro.com
  // Timer-3 16-bit, Mode-14 Fast, Top=ICR
  // 50 Hz Frequency, Clock is 16 MHz

  TCCR3B = 0x18;  // 0001 1000, Disable Timer
  TCCR3A = 0xA2;  // 1010 0010

  ICR3 = 40000 - 1;
  OCR3A = (int)(ICR3 * 0.25);
  OCR3B = (int)(ICR3 * 0.50);
  TCNT3 = 0x0;

  pinMode(2, OUTPUT);  // OC3b
  pinMode(5, OUTPUT);  // OC3a

  TCCR3B |= 2;  // Prescale=8, Enable Timer

  // For Serial print
  Serial.begin(9600);
  initCarControl(&carcontrol, NUM_SENSORS);
  for(int i=0; i<NUM_SENSORS; ++i) {
    initFilter(&filter[i], IR_MID);
    initKillspike(&killspike[i], IR_MID, IR_MAX, true, 10, 5);
  }
  for (int i = 0; i < 20000; ++i)
  {
    STOP;
    CENTER;
  }
}

void understand_environment(void){
  // emergency stop
  if(ir[4] < STOP_DISTANCE || ir[5] < STOP_DISTANCE || ir[6] < STOP_DISTANCE) {
    carcontrol.emergency_stop = true;
  }
  for (int i = 0; i < NUM_SENSORS; ++i)
  {
    switch (sensor_group[i])
    {
    case -3:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.left_open = false;
          }
      break;
    case -2:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.front_left_open = false;
          }
      break;
    case -1:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.front_open = false;
          }
      break;
    case 0:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.front_open = false;
          }
      break;
    case 1:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.front_open = false;
          }
      break;
    case 2:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.front_right_open = false;
          }
      break;
    case 3:
      if (ir[i] < IR_THRESHOLD) {
        carcontrol.right_open = false;
          }
      break;
    default:
      break;
    }
  }
  // follow line
  if (carcontrol.right_open && carcontrol.front_right_open && ir[5] > 120 &&ir[6] > IR_MID) {
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
      // check if RC car is close to left wall
      if (ir[i] < IR_MID && sensor_group[i] == -3) {
        carcontrol.following_left_line = true;
      }
    }
  }
  else if (carcontrol.left_open && carcontrol.front_left_open && ir[5] > 120 && ir[4] > IR_MID) {
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
      // check if RC car is close to right wall
      if (ir[i] < IR_MID && sensor_group[i] == 3) {
        carcontrol.following_right_line = true;
      }
    }
  }
  // both sides are closed
  if(carcontrol.front_open && !carcontrol.left_open && !carcontrol.right_open && ir[5] > IR_MID) {
    carcontrol.narrow_passage = true;
  }
  // check if RC car is in an open field
  if (carcontrol.front_left_open && carcontrol.front_open && carcontrol.front_right_open && ir[5] >= FRONT_IR_MAX - 5) {
    carcontrol.open_field++;
  }
  else {
    carcontrol.open_field = 0;
  }
  // check if RC car needs to avoid an obstacle
  if (!(carcontrol.front_left_open && carcontrol.front_open && carcontrol.front_right_open)) {
    carcontrol.avoid_obstacle = DECISION_THRESHOLD;
  }
  else {
    carcontrol.avoid_obstacle--; 
  }
  if(DDEBUG) {
    Serial.print(carcontrol.left_open);
    Serial.print("\t");
    Serial.print(carcontrol.front_left_open);
    Serial.print("\t");
    Serial.print(carcontrol.front_open);
    Serial.print("\t");
    Serial.print(carcontrol.front_right_open);
    Serial.print("\t");
    Serial.print(carcontrol.right_open);
    Serial.print("\t");
    Serial.print(carcontrol.following_left_line);
    Serial.print("\t");
    Serial.print(carcontrol.following_right_line);
    Serial.print("\t");
    Serial.print(carcontrol.narrow_passage);
    Serial.print("\t");
    Serial.print(carcontrol.open_field);
    Serial.print("\t");
    Serial.print(carcontrol.avoid_obstacle);
    Serial.print("\t");
    Serial.print(carcontrol.emergency_stop);
    Serial.print("\t");
  }
  if(SIMPLE_MODE) updateCarControl(&carcontrol, ir);
}
void set_objective() {
  // update objective
  updateObjective(&carcontrol);
  if (carcontrol.emergency_stop || carcontrol.objective[1].emergency_stop) // 한 번 emergency stop이 되면 계속 emergency stop
    carcontrol.objective[0].emergency_stop = true;
  else
    carcontrol.objective[0].emergency_stop = false;
  
  if (carcontrol.avoid_obstacle <= 0) {
    // avoid obstacle
    if (carcontrol.open_field >= DECISION_THRESHOLD) {
      // go straight
      carcontrol.objective[0].angle = 0;
      carcontrol.objective[0].speed = FAST;
    }
    else if (carcontrol.following_left_line) {
      float diff = ir[0] - ir[1];
      carcontrol.objective[0].angle = atan(diff / SENSOR_DISTANCE_0TO1) * 180.0 / 3.14159;
      carcontrol.objective[0].speed = MEDIUM;
    }
    else if (carcontrol.following_right_line) {
      float diff = ir[9] - ir[10];
      carcontrol.objective[0].angle = atan(diff / SENSOR_DISTANCE_0TO1) * 180.0 / 3.14159;
      carcontrol.objective[0].speed = MEDIUM;
    }
    else if (carcontrol.narrow_passage) {
      float diff[2];
      diff[0] = ir[0] - ir[10];
      diff[1] = ir[1] - ir[9];
      float avgdiff = (diff[0] - diff[1]) * 0.5;
      carcontrol.objective[0].angle = atan(avgdiff / SENSOR_DISTANCE_0TO1) * 180.0 / 3.14159;
      carcontrol.objective[0].speed = MEDIUM;
    }
    else{
      goto A;
    }
  }
    else {
      A:
      float point[NUM_SENSORS], patch_point[NUM_SENSORS-4];
      int max_point_idx = 0;
      for (int i = 0; i < NUM_SENSORS; ++i)
      {
        point[i] = ir[i];
      }
      for (int i = 0; i < NUM_SENSORS-4; ++i)
      {
        patch_point[i] = (point[i+1] + 1.2*point[i+2] + point[i+3])/3.2;
        if(patch_point[i] > patch_point[max_point_idx]) {
          max_point_idx = i;
        }
        if(point[4] == IR_MAX && point[5] == IR_MAX && point[6] == IR_MAX) { // 같은 상황이면 정면으로!
          max_point_idx = 3;  
        }
      }
      carcontrol.objective[0].angle = (max_point_idx - 3) * 22.5;
      carcontrol.objective[0].speed = SLOW;
      
//    for (int i=0; i<7; i++){
//    Serial.print(patch_point[i]);
//    Serial.print("\t");
//  }
//    Serial.println();
  }
  if (carcontrol.objective[0].emergency_stop) {
    if(!carcontrol.objective[1].emergency_stop) {
      carcontrol.objective[0].speed = 6000 - carcontrol.objective[1].speed; // 처음에는 정지신호 1번
      }
    else
      carcontrol.objective[0].speed = REVERSE; // 그다음부터는 후진
    if((ir[4]+ir[5]+ir[6])/3 - prev_ir_avg > 0) system_stop_flag = 1;
    prev_ir_avg = (ir[4]+ir[5]+ir[6])/3;
  }
  if (DDEBUG)
  {
    Serial.print(carcontrol.objective[0].angle);
    Serial.print("\t");
    Serial.print(carcontrol.objective[0].speed);
    Serial.print("\t");
  }

}

void motor_control(void) {
  // if (carcontrol.objective[0].emergency_stop) {
  //   if (carcontrol.objective[1].emergency_stop) {
  //     carcontrol.objective[0].speed = 3200;
  //   }
  //   else {
  //     carcontrol.objective[0].speed = 0;
  //     carcontrol.objective[0].angle = 0;
  //   }

  //   if(reverse_avg - (ir[4]+ir[5]+ir[6])/3 <= 5) STOP;
 
  //   reverse_ir4 = ir[4];  reverse_ir5 = ir[5]; reverse_ir6 = ir[6];
  //   reverse_avg = (reverse_ir4+reverse_ir5+reverse_ir6)/3;
  //   return;
  // }

  float speed = 0.5*carcontrol.objective[0].speed + 0.5*carcontrol.objective[1].speed;
  float angle =  carcontrol.objective[0].angle; //(sign(carcontrol.objective[0].angle) == sign(carcontrol.objective[1].angle)) ? carcontrol.objective[0].angle : carcontrol.objective[1].angle;

  // control_refined(speed, angle);
  if(system_stop_flag) {
    STOP;
    return;
  }
  analogWrite(Thr_PWM, speed);
  angle = max(min(angle, 30), -30);
  analogWrite(Ser_PWM, map(-1*angle, -30, 30, 2240, 3858));
  if(DDEBUG) {
    Serial.print(speed);
    Serial.print("\t");
    Serial.print(angle);
    Serial.print("\t");
  }
}

void loop() {
  get_result();
  //Serial.print("get result\n");
  updateCarControl(&carcontrol, ir);
  //Serial.print("updateCarControl\n");
  understand_environment();
  //Serial.print("understand environment\n");
  set_objective();
  //Serial.print("set objective\n");
  motor_control();
  if (DDEBUG) Serial.println("");
}


// 전압을 거리로 변환
float signal2distance(float signal, int type) {
  if (type == 0) {
  return float(10650.08 * pow(signal, -0.935) - 3.937);
  } else {
  return float((27.61 / (signal * 5000.0 / 1023.0 - 0.1696)) * 100.0);
  }
}

void get_result(void) {
  // read analog value
  for (int i = 0; i < NUM_SENSORS; i++) {
    float tmp = signal2distance(analogRead(pin_list[i]), sensor_type[i]);
    if(i==0 || i==10) tmp += 7;
    if(i != 5) tmp = max(min(tmp, IR_MAX), IR_MIN);
    else tmp = max(min(tmp, FRONT_IR_MAX), IR_MIN);
    //ir[i] = tmp;
    ir[i] = killspikefilter(&killspike[i], tmp);
    if (DDEBUG) {Serial.print(ir[i]); Serial.print("\t");}
  }
}