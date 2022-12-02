#pragma once
#ifndef QUATERNIONFILTER_H
#define QUATERNIONFILTER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "timer_driver.h"

#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define EULER       2.718281828459045235360287471352

void quat_filter_setup(void);
void quat_filter_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void quat_filter_no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void quat_filter_madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void quat_filter_mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);


#endif  // QUATERNIONFILTER_H
