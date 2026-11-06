#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

#include <math.h>
#include <stdbool.h>

#define PI_F 3.1415926535f
#define DEG2RAD(deg) ((deg) * PI_F / 180.0f)
#define RAD2DEG(rad) ((rad) * 180.0f / PI_F)
#define SAFE_DIV(n, d) ((n) / fmaxf((d), 0.1f))

typedef struct {
    float kp, ki, kd;
    float integral;
    float i_max;
    float d_filtered;
} PID;

typedef struct {
    float h, v_z, v_forward; 
    float x, y;              
    float pitch, roll, yaw;
    float p, q, r;
    float alpha_actual;      
} FlightState;

typedef struct {
    float target_h, target_vz;
    float target_x, target_y;
    float target_pitch, target_roll, target_yaw;
    float target_p, target_q, target_r;
    float target_alpha;      
} FlightTarget;

typedef struct {
    float T_req;     
    float M_theta;   
    float M_phi;     
    float M_psi;     
} WrenchCmd;

void PID_Init(PID *pid, float kp, float ki, float kd, float i_max);
float PID_Calc(PID *pid, float error, float current_rate, bool allow_integral, float dt);
float Altitude_Control(PID *pid_vz, FlightTarget *tgt, FlightState *st, float mass);
void Attitude_Control(FlightState *st, FlightTarget *tgt, WrenchCmd *out,
                      PID *pid_att_pitch, PID *pid_rate_pitch,
                      PID *pid_att_roll, PID *pid_rate_roll,
                      PID *pid_att_yaw, PID *pid_rate_yaw);

#endif