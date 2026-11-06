#include <stdio.h>
#include <math.h>
#include "flight_manager.h"

void PID_Clear_Integrals(void) {
    printf(">>> Mode Switch: PID Integrals Cleared! <<<\n");
}

int main() {
    FlightState st = {0};
    st.alpha_actual = DEG2RAD(90.0f);
    FlightTarget tgt = {0};
    WrenchCmd cmd = {0};
    ActuatorOutput act = {0};

    PID pid_vz, pid_att_pitch, pid_rate_pitch, pid_att_roll, pid_rate_roll, pid_att_yaw, pid_rate_yaw;
    PID_Init(&pid_vz, 2.0f, 0.5f, 0.0f, 5.0f);
    PID_Init(&pid_att_pitch, 4.0f, 0, 0, 0);
    PID_Init(&pid_rate_pitch, 0.8f, 0.05f, 0.02f, 5.0f);
    PID_Init(&pid_att_roll, 4.0f, 0, 0, 0);
    PID_Init(&pid_rate_roll, 0.8f, 0.05f, 0.02f, 5.0f);
    PID_Init(&pid_att_yaw, 2.0f, 0, 0, 0);
    PID_Init(&pid_rate_yaw, 0.5f, 0.01f, 0, 5.0f);

    FlightMode current_mode = MODE_MULTIROTOR;
    float last_stick_dir = 1.0f;
    float dt = 0.01f;

    FILE *fp = fopen("ultimate_flight.csv", "w");
    fprintf(fp, "Step,Mode,Height,Speed,Alpha_Deg,RTH_Triggered\n");
    printf("--- Ultimate FC Simulation Started ---\n");

    for (int step = 0; step < 2000; step++) {
        float stick_val = (step > 300) ? -1.0f : 1.0f; 
        float stick_thr = 0.5f; 
        float stick_pitch = 0.0f, stick_roll = 0.0f;
        
        st.x = step * 0.5f; 
        float dist_to_home = sqrtf(st.x*st.x + st.y*st.y);
        int rth_triggered = 0;
        if (dist_to_home > 500.0f) {
            rth_triggered = 1;
            stick_val = -1.0f;
        }

        current_mode = Mode_Switch_FSM(stick_val, current_mode, &last_stick_dir, &tgt, &st);
        Target_Generator(current_mode, stick_thr, stick_pitch, stick_roll, 0.0f, 0.0f, 0.0f, 5.0f, &st, &tgt);

        if (current_mode != MODE_MULTIROTOR) {
            if (st.v_forward < 8.0f) tgt.target_alpha = DEG2RAD(70.0f);
            else if (st.v_forward < 15.0f) tgt.target_alpha = DEG2RAD(40.0f);
            else tgt.target_alpha = DEG2RAD(5.0f);
        } else {
            tgt.target_alpha = DEG2RAD(90.0f);
        }
        
        float max_step = DEG2RAD(3.0f);
        float diff = tgt.target_alpha - st.alpha_actual;
        if (diff > max_step) st.alpha_actual += max_step;
        else if (diff < -max_step) st.alpha_actual -= max_step;
        else st.alpha_actual = tgt.target_alpha;

        if (st.alpha_actual <= DEG2RAD(6.0f)) current_mode = MODE_FIXED_WING;

        cmd.T_req = Altitude_Control(&pid_vz, &tgt, &st, 1.5f);
        Attitude_Control(&st, &tgt, &cmd, &pid_att_pitch, &pid_rate_pitch, &pid_att_roll, &pid_rate_roll, &pid_att_yaw, &pid_rate_yaw);

        Mixer_Transition_Solve(current_mode, st.alpha_actual, cmd.T_req, cmd, st.v_forward, st.roll, &act);

        float T_total = act.T_L + act.T_R + act.T_tail;
        float aero_lift = 0.05f * st.v_forward * st.v_forward;
        if (aero_lift > 14.7f) aero_lift = 14.7f;
        
        float total_lift = T_total * sinf(st.alpha_actual) * cosf(st.pitch) + aero_lift;
        float acc_z = (total_lift - 1.5f * 9.81f) / 1.5f;
        st.v_z += acc_z * dt;
        st.h += st.v_z * dt;
        if (st.h < 0) st.h = 0;

        float total_forward = T_total * sinf(st.alpha_actual) * sinf(-st.pitch) + T_total * cosf(st.alpha_actual);
        float acc_x = (total_forward - 0.01f * st.v_forward * st.v_forward) / 1.5f;
        st.v_forward += acc_x * dt;

        st.q += (cmd.M_theta / 0.05f) * dt;
        st.pitch += st.q * dt;
        st.p += (cmd.M_phi / 0.02f) * dt;
        st.roll += st.p * dt;

        fprintf(fp, "%d,%d,%.2f,%.2f,%.2f,%d\n", step, current_mode, st.h, st.v_forward, RAD2DEG(st.alpha_actual), rth_triggered);
    }
    fclose(fp);
    printf("--- Simulation Finished ---\n");
    return 0;
}