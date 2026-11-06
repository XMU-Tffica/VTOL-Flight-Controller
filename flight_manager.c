#include "flight_manager.h"
#include "utils.h"

extern void PID_Clear_Integrals(void);

FlightMode Mode_Switch_FSM(float stick_val, FlightMode current_mode, 
                           float *last_stick_dir, FlightTarget *tgt, FlightState *st) {
    float current_dir = 0.0f;
    if (stick_val > 0.6f) current_dir = 1.0f;       
    else if (stick_val < -0.6f) current_dir = -1.0f; 
    else current_dir = *last_stick_dir; 

    if (current_dir != *last_stick_dir && current_dir != 0.0f) {
        *last_stick_dir = current_dir;
        PID_Clear_Integrals();

        if (current_dir == 1.0f) {
            tgt->target_pitch = st->pitch;
            tgt->target_roll = st->roll;
            tgt->target_h = st->h;
            return MODE_MULTIROTOR;
        } else {
            tgt->target_x = st->x;
            tgt->target_y = st->y;
            tgt->target_h = st->h;
            return MODE_TRANSITION;
        }
    }
    return current_mode;
}

void Target_Generator(FlightMode mode, float stick_thr, float stick_pitch, float stick_roll, 
                      float stick_yaw_rate, float wp_x, float wp_y, float wp_z,
                      FlightState *st, FlightTarget *tgt) {
    if (mode == MODE_MULTIROTOR) {
        // 死区处理，防止摇杆中位噪声
        float clean_pitch = apply_deadband(stick_pitch, 0.05f);
        float clean_roll  = apply_deadband(stick_roll, 0.05f);
        float clean_thr   = apply_deadband(stick_thr, 0.05f);

        tgt->target_pitch = clean_pitch * DEG2RAD(30.0f);
        tgt->target_roll  = clean_roll * DEG2RAD(30.0f);
        tgt->target_yaw   = 0.0f;
        
        if (fabsf(clean_thr) < 0.01f) tgt->target_vz = 0.0f;
        else tgt->target_vz = clean_thr * 3.0f;
        
    } else if (mode == MODE_TRANSITION || mode == MODE_FIXED_WING) {
        tgt->target_h = wp_z;
        tgt->target_pitch = 0.0f; 
        tgt->target_roll = 0.0f;
    }
}