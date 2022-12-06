#ifndef  QUAT_FIL_TIM
#define QUAT_FIL_TIM

#include "nrfx_timer.h"
#include "quaternion_filter_timer.h"

void quaternion_filter_timer_init(void);
uint32_t quaternion_filter_get_us_count(void);

#endif