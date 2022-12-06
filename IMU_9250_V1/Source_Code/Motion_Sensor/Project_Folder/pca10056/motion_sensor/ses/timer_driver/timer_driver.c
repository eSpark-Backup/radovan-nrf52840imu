/********************************************************************************************************
 * $Id$        timer_driver.c
 * @file       timer_driver.c
 * @brief      Timer Driver Source file
 * @date       16 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"

#include "timer_driver.h"

/**************************************************** variables ********************************************/
uint16_t cnt=0;                                               //counts value to send with Hello World      
bool f_Send = false;                                       // timer overflow flag,
                                                              // f_Send=true on timer overflow
                                                              // f_Send = false, set in while(true)  
uint64_t usCount=0;
uint16_t flag_count=0;

APP_TIMER_DEF(m_millis_timer_id);                             // Timer definition  

/******************************************Function definitions ********************************************/

/**@brief Function to handle timer event
 *
 * @details Time event handler function
 *
 * @param[in]	 none
 * @param[out]   none
 */
static void millis_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    if(cnt++ >=50000) cnt=0;
    if(flag_count++ >=12) { f_Send = true; flag_count=0;}
    usCount++;
}

/**@brief Timer Init
 *
 * @details Function to initialize timer
 *
 * @param[in]	 none
 * @param[out]   none
 */
void timer_init(void)
{
    NRF_LOG_INFO("Timer Init Start");
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_millis_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                millis_timeout_handler);

    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_millis_timer_id, MILLIS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Timer Init Done");
}

/**@brief Timer Start
 *
 * @details Function to start timer
 *
 * @param[in]	 none
 * @param[out]   none
 */
void timer_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_millis_timer_id, MILLIS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Timer Started");
}


/**@brief Timer Stop
 *
 * @details Function to stop timer
 *
 * @param[in]	 none
 * @param[out]   none
 */
void timer_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_millis_timer_id);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Timer Stopped");
}

uint64_t getusCount(void)
{
  return usCount*1000;
}
/**/

