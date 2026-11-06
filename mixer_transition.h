#ifndef MIXER_TRANSITION_H
#define MIXER_TRANSITION_H

#include "flight_ctrl.h"

#define X_F 0.23f
#define X_T 0.25f
#define Z_F 0.05f
#define Z_T 0.0f
#define Y_F 0.19f

typedef enum { MODE_MULTIROTOR, MODE_TRANSITION, MODE_FIXED_WING } FlightMode;

typedef struct {
    float T_L, T_R, T_tail;   
    float servo_L, servo_R;   
    float servo_tilt;         
} ActuatorOutput;

float Servo_Rate_Limit(float target_angle, float current_angle, float max_step);
void Mixer_Transition_Solve(FlightMode mode, float alpha_actual, float T_req, WrenchCmd w,
                            float current_v, float current_roll, ActuatorOutput *out);
#endif