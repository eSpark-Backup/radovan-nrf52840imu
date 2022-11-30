/********************************************************************************************************
 * $Id$        timer_driver.h
 * @file       timer_driver.h
 * @brief      Timer Driver Header file
 * @date       16 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H


/***************************************** Macros *************************************************/
#define MILLIS_INTERVAL                 APP_TIMER_TICKS(500) //timer value, 500ms


/***************************************** variable declaration**************************************/
extern uint16_t cnt;
extern bool f_Send;

/***********************************Function Declaration ******************************************/
void timer_init(void);
void timer_start(void);
void timer_stop(void);

#endif /* TIMER_DRIVER_H */