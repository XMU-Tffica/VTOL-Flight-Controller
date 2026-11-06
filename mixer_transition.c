#include "mixer_transition.h"
#include "utils.h"

float Servo_Rate_Limit(float target_angle, float current_angle, float max_step) {
    float err = target_angle - current_angle;
    if (err > max_step) return current_angle + max_step;
    if (err < -max_step) return current_angle - max_step;
    return target_angle;
}

void Mixer_Transition_Solve(FlightMode mode, float alpha_actual, float T_req, WrenchCmd w,
                            float current_v, float current_roll, ActuatorOutput *out) {
    float alpha = sat(alpha_actual, 0.0f, DEG2RAD(90.0f));
    float sin_a = sinf(alpha);
    float cos_a = cosf(alpha);
    float W_aero = 1.0f - sin_a;

    float T_v_comp = T_req;
    if (mode == MODE_TRANSITION) T_v_comp = SAFE_DIV(T_req, fmaxf(sin_a, 0.2f)); 

    if (mode == MODE_MULTIROTOR || mode == MODE_TRANSITION) {
        float den = (X_F + X_T) * sin_a + (Z_F + Z_T) * cos_a;
        float T_front_sum;
        if (mode == MODE_MULTIROTOR) T_front_sum = (T_req * X_T + w.M_theta) / (X_F + X_T);
        else T_front_sum = (T_v_comp * (X_T * sin_a + Z_T * cos_a) - w.M_theta) / fmaxf(den, 0.1f);
        
        float T_tail = SAFE_DIV(T_v_comp, fmaxf(sin_a, 0.2f)) - T_front_sum;
        
        out->T_L = (T_front_sum / 2.0f) + (w.M_phi / (2.0f * Y_F));
        out->T_R = (T_front_sum / 2.0f) - (w.M_phi / (2.0f * Y_F));
        out->T_tail = T_tail;
    } else {
        out->T_L = 2.0f; out->T_R = 2.0f; out->T_tail = 2.0f;
    }

    out->T_L = sat(out->T_L, 0.0f, 7.35f); 
    out->T_R = sat(out->T_R, 0.0f, 7.35f);
    out->T_tail = sat(out->T_tail, 0.0f, 14.7f); 

    if (mode == MODE_FIXED_WING || mode == MODE_TRANSITION) {
        float v2 = fmaxf(current_v * current_v, 25.0f); 
        float delta_e = (w.M_theta * W_aero) / (1.5f * v2); 
        float delta_a = (w.M_phi * W_aero) / (1.2f * v2);  
        float delta_turn = 0.5f * (current_roll * current_roll); 
        
        float target_servo_L = delta_e + delta_turn + delta_a;
        float target_servo_R = delta_e + delta_turn - delta_a;
        
        target_servo_L = sat(target_servo_L, -DEG2RAD(45), DEG2RAD(45));
        target_servo_R = sat(target_servo_R, -DEG2RAD(45), DEG2RAD(45));
        
        out->servo_L = Servo_Rate_Limit(target_servo_L, out->servo_L, 0.052f);
        out->servo_R = Servo_Rate_Limit(target_servo_R, out->servo_R, 0.052f);
    }

    out->servo_tilt = SAFE_DIV(w.M_psi, 0.5f * fmaxf(out->T_tail, 1.0f));
    out->servo_tilt = sat(out->servo_tilt, -DEG2RAD(35), DEG2RAD(27)); 
}