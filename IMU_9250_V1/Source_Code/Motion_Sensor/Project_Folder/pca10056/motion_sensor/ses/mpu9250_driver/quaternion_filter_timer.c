#include "quaternion_filter_timer.h"

const nrfx_timer_t QUATERNION_TIMER = NRFX_TIMER_INSTANCE(0);
uint32_t quaternion_us_count=0;

/**
 * @brief Handler for timer events.
 */
void quaternion_filter_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            quaternion_us_count+=interrupt_time_us;
            break;

        default:
            //Do nothing.
            break;
    }
}

void quaternion_filter_timer_init(void) {

    uint32_t time_us = interrupt_time_us; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;


    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    err_code = nrfx_timer_init(&QUATERNION_TIMER, &timer_cfg, quaternion_filter_timer_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrfx_timer_us_to_ticks(&QUATERNION_TIMER, time_us);

    nrfx_timer_extended_compare(
         &QUATERNION_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrfx_timer_enable(&QUATERNION_TIMER);

}

uint32_t quaternion_filter_get_us_count(void)
{
  return quaternion_us_count;
}