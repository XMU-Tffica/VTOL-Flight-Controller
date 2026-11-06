#ifndef FLIGHT_MANAGER_H
#define FLIGHT_MANAGER_H

#include "flight_ctrl.h"
#include "mixer_transition.h"

FlightMode Mode_Switch_FSM(float stick_val, FlightMode current_mode, 
                           float *last_stick_dir, FlightTarget *tgt, FlightState *st);

void Target_Generator(FlightMode mode, float stick_thr, float stick_pitch, float stick_roll, 
                      float stick_yaw_rate, float wp_x, float wp_y, float wp_z,
                      FlightState *st, FlightTarget *tgt);

#endif