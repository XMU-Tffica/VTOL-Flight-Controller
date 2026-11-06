#ifndef UTILS_H
#define UTILS_H
#include <math.h>

float sat(float val, float min_val, float max_val);
float apply_deadband(float input, float threshold);
float wrap_pi(float angle);

#endif