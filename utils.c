#include "utils.h"

float sat(float val, float min_val, float max_val) {
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

float apply_deadband(float input, float threshold) {
    if (fabsf(input) < threshold) return 0.0f;
    return input;
}

float wrap_pi(float angle) {
    const float PI = 3.14159265f;
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}