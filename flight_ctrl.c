#include "flight_ctrl.h"
#include "utils.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float i_max) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integral = 0; pid->i_max = i_max; pid->d_filtered = 0;
}

float PID_Calc(PID *pid, float error, float current_rate, bool allow_integral, float dt) {
    if (allow_integral) {
        pid->integral += error * dt;
        pid->integral = sat(pid->integral, -pid->i_max, pid->i_max);
    }
    float d_term = pid->kd * (0.0f - current_rate);
    pid->d_filtered = 0.9f * pid->d_filtered + 0.1f * d_term;
    return (pid->kp * error) + (pid->ki * pid->integral) + pid->d_filtered;
}

float Altitude_Control(PID *pid_vz, FlightTarget *tgt, FlightState *st, float mass) {
    float err_h = tgt->target_h - st->h;
    err_h = sat(err_h, -2.0f, 2.0f); 
    tgt->target_vz = sat(err_h * 1.5f, -3.0f, 3.0f); 
    float err_vz = tgt->target_vz - st->v_z;
    bool allow_integral = (fabsf(err_vz) < 0.05f);
    float pid_out = PID_Calc(pid_vz, err_vz, 0, allow_integral, 0.01f);
    float T_req = (mass * 9.81f) + pid_out;
    float cos_comp = fmaxf(cosf(st->pitch) * cosf(st->roll), 0.1f);
    return T_req / cos_comp;
}

void Attitude_Control(FlightState *st, FlightTarget *tgt, WrenchCmd *out,
                      PID *pid_att_pitch, PID *pid_rate_pitch,
                      PID *pid_att_roll, PID *pid_rate_roll,
                      PID *pid_att_yaw, PID *pid_rate_yaw) {
    float err_pitch = tgt->target_pitch - st->pitch;
    tgt->target_q = PID_Calc(pid_att_pitch, err_pitch, 0, false, 0.01f);
    tgt->target_q = sat(tgt->target_q, -DEG2RAD(180), DEG2RAD(180));
    out->M_theta = PID_Calc(pid_rate_pitch, tgt->target_q - st->q, st->q, true, 0.01f);

    float err_roll = tgt->target_roll - st->roll;
    tgt->target_p = PID_Calc(pid_att_roll, err_roll, 0, false, 0.01f);
    tgt->target_p = sat(tgt->target_p, -DEG2RAD(180), DEG2RAD(180));
    out->M_phi = PID_Calc(pid_rate_roll, tgt->target_p - st->p, st->p, true, 0.01f);

    float err_yaw = wrap_pi(tgt->target_yaw - st->yaw);
    tgt->target_r = PID_Calc(pid_att_yaw, err_yaw, 0, false, 0.01f);
    out->M_psi = PID_Calc(pid_rate_yaw, tgt->target_r - st->r, st->r, true, 0.01f);
}