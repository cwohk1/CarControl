#include "car_control.h"
#include "filters.h"
#define SMALL_TIME 333
#define LOOPCOUNT 10
#ifndef Thr_PWM
#define Thr_PWM 2
#endif
#ifndef Ser_PWM
#define Ser_PWM 5
#endif
#ifndef DDEBUG
#define DDEBUG 0
#endif

int pin_list[11] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};
int sensor_type[11] = {0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1};  // 0: long, 1: short

CarControl carcontrol;
Filter filter[NUM_SENSORS];
int get_result_clk = 0;
int adc_history[11][LOOPCOUNT];
float ir[NUM_SENSORS] = {IR_MAX};  // array for IR distance values


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
	Serial.begin(1200);
    initCarControl(&carcontrol, NUM_SENSORS);
	for(int i=0; i<NUM_SENSORS; ++i) {
		initFilter(&filter[i], IR_MAX);
	}
	STOP;
	CENTER;
}

void understand_environment(void){
	// emergency stop
	int stop_cnt = 0;
	for(int i = 4; i < NUM_SENSORS-4; i++) {
		if(getDistance(&carcontrol, i) < STOP_DISTANCE ) {
			stop_cnt++;
		}
	}
	if(stop_cnt >= 1) {
		carcontrol.emergency_stop++;	
	}
	else {
		carcontrol.emergency_stop = 0;
	}
	for (int i = 0; i < NUM_SENSORS; ++i)
	{
		switch (sensor_group[i])
		{
		case -3:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.left_open = false;
	        }
			break;
		case -2:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.front_left_open = false;
	        }
			break;
		case -1:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.front_open = false;
	        }
			break;
		case 0:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.front_open = false;
	        }
			break;
		case 1:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.front_open = false;
	        }
			break;
		case 2:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.front_right_open = false;
	        }
			break;
		case 3:
			if (getDistance(&carcontrol, i) < IR_THRESHOLD) {
				carcontrol.right_open = false;
	        }
			break;
		default:
			break;
		}
	}
	// follow line
	if (carcontrol.front_open && carcontrol.right_open && carcontrol.front_right_open) {
		for (int i = 0; i < NUM_SENSORS; ++i)
		{
			// check if RC car is close to left wall
			if (getDistance(&carcontrol, i) < IR_MID && sensor_group[i] == -3) {
				carcontrol.following_right_line = true;
			}
		}
	}
	else if (carcontrol.front_open && carcontrol.left_open && carcontrol.front_left_open) {
		for (int i = 0; i < NUM_SENSORS; ++i)
		{
			// check if RC car is close to right wall
			if (getDistance(&carcontrol, i) < IR_MID && sensor_group[i] == 3) {
				carcontrol.following_left_line = true;
			}
		}
	}
	// both sides are closed
	if(carcontrol.front_open && !carcontrol.left_open && !carcontrol.right_open) {
		carcontrol.narrow_passage = true;
	}
	// check if RC car is in an open field
	if (carcontrol.front_left_open && carcontrol.front_open && carcontrol.front_right_open && getDistance(&carcontrol, 5) >= FRONT_IR_MAX - 5) {
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
}
void set_objective() {
	// update objective
	updateObjective(&carcontrol);
	if (carcontrol.emergency_stop >= 2)
		carcontrol.objective[0].emergency_stop = true;
	else
		carcontrol.objective[0].emergency_stop = false;
	
	if (carcontrol.avoid_obstacle > 0) {
		// avoid obstacle
		if (carcontrol.open_field >= DECISION_THRESHOLD) {
			// go straight
			carcontrol.objective[0].angle = 0;
			carcontrol.objective[0].speed = 0.1875;
		}
		else if (carcontrol.following_left_line) {
			float diff = getDistance(&carcontrol, 0) - getDistance(&carcontrol, 1);
			carcontrol.objective[0].angle = atan(diff / SENSOR_DISTANCE_0TO1) * 180.0 / 3.14159;
			carcontrol.objective[0].speed = 0.185;
		}
		else if (carcontrol.following_right_line) {
			float diff = getDistance(&carcontrol, 9) - getDistance(&carcontrol, 10);
			carcontrol.objective[0].angle = atan(diff / SENSOR_DISTANCE_0TO1) * 180.0 / 3.14159;
			carcontrol.objective[0].speed = 0.185;
		}
		else if (carcontrol.narrow_passage) {
			float diff[2];
			diff[0] = getDistance(&carcontrol, 0) - getDistance(&carcontrol, 10);
			diff[1] = getDistance(&carcontrol, 1) - getDistance(&carcontrol, 9);
			float avgdiff = (diff[0] - diff[1]) * 0.5;
			carcontrol.objective[0].angle = atan(avgdiff / SENSOR_DISTANCE_0TO1) * 180.0 / 3.14159;
			carcontrol.objective[0].speed = 0.185;
		}
		if (carcontrol.avoid_obstacle > 0) {
			float point[NUM_SENSORS], patch_point[NUM_SENSORS-4];
			int min_distance_idx = 0, max_point_idx = 0;
			for (int i = 0; i < NUM_SENSORS; ++i)
			{
				point[i] = IR_MAX - getDistance(&carcontrol, i);
				point[5] += FRONT_IR_MAX - IR_MAX;
				if(getDistance(&carcontrol, i) < getDistance(&carcontrol, min_distance_idx)) {
					min_distance_idx = i;
				}
			}
			for (int i = 0; i < NUM_SENSORS-4; ++i)
			{
				patch_point[i] = point[i+1] + point[i+2] + point[i+3];
				if(patch_point[i] > patch_point[max_point_idx]) {
					max_point_idx = i;
				}
			}
			carcontrol.objective[0].angle = (max_point_idx - 3) * 22.5;
			carcontrol.objective[0].speed = 0.185;
		}
		else {
			float point[NUM_SENSORS], patch_point[NUM_SENSORS-4];
			int min_distance_idx = 0, max_point_idx = 0;
			for (int i = 0; i < NUM_SENSORS; ++i)
			{
				point[i] = IR_MAX - getDistance(&carcontrol, i);
				point[5] += FRONT_IR_MAX - IR_MAX;
				if(getDistance(&carcontrol, i) < getDistance(&carcontrol, min_distance_idx)) {
					min_distance_idx = i;
				}
			}
			for (int i = 0; i < NUM_SENSORS-4; ++i)
			{
				patch_point[i] = point[i+1] + point[i+2] + point[i+3];
				if(patch_point[i] > patch_point[max_point_idx]) {
					max_point_idx = i;
				}
			}
			carcontrol.objective[0].angle = (max_point_idx - 3) * 22.5;
			carcontrol.objective[0].speed = 0.185;
		}
	}
}

void motor_control(void) {
	if (carcontrol.emergency_stop >= DECISION_THRESHOLD) {
		REVERSE;
		return;
	}
	float speed = 0.5*carcontrol.objective[0].speed + 0.5*carcontrol.objective[1].speed;
	float angle = sign(carcontrol.objective[0].angle) == sign(carcontrol.objective[1].angle) ? carcontrol.objective[0].angle : carcontrol.objective[1].angle;
	control_refined(speed, angle);
}

void loop() {
	get_result();
	updateCarControl(&carcontrol, ir);
	understand_environment();
	set_objective();
	motor_control();
}

// killspike method
void get_result() {
  // read analog value
  	for (int i = 0; i < 11; i++) {
    	adc_history[i][get_result_clk] = analogRead(pin_list[i]);
  	}

  for (int i = 0; i < 11; i++) {
    // kill spike and average
    int sum = 0, x = 0, high = 0, low = 0, avg = 0;
    high = low = adc_history[i][0];
    for (int j = 0; j < LOOPCOUNT; j++) {
      x = adc_history[i][j];
      sum += x;
      if (x > high) high = x;
      if (x < low) low = x;
    }
    avg = (sum - high - low) / (LOOPCOUNT - 2);

    // convert voltage to centimeter
    if (sensor_type[i] == 0) {
      ir[i] = float(10650.08 * pow(avg, -0.935) - 3.937);
    } else {
      ir[i] = float((27.61 / (avg * 5000.0 / 1023.0 - 0.1696)) * 100.0);
    }
	ir[0] += 3;
	ir[10] += 3;
    // limit the range of IR distance
    ir[i] = max(min(ir[i], IR_MAX), IR_MIN);
  }

  get_result_clk++;
  if (get_result_clk == LOOPCOUNT) 
      get_result_clk = 0;
}
// 전압을 거리로 변환
// float signal2distance(float signal, int type) {
//   if (type == 0) {
// 	return float(10650.08 * pow(signal, -0.935) - 3.937);
//   } else {
// 	return float((27.61 / (signal * 5000.0 / 1023.0 - 0.1696)) * 100.0);
//   }
// }

// void get_result(void) {
// 	// read analog value
// 	for (int i = 0; i < NUM_SENSORS; i++) {
// 		float tmp = signal2distance(analogRead(pin_list[i]), sensor_type[i]);
// 		if(i==0 || i==10) tmp += 3;
// 		if(tmp != 5) tmp = max(min(tmp, IR_MAX), IR_MIN);
// 		else tmp = max(min(tmp, FRONT_IR_MAX), IR_MIN);
// 		if (DDEBUG) Serial.print(tmp); Serial.print("\t");
// 		ir[i] = tmp;
// 		//ir[i] = updateKalmanFilter(&kf[i], tmp);
// 		//ir[i] = lowPassFilter_2(&filter[i], tmp);
// 	}
// 	if (DDEBUG) Serial.println();
// }
