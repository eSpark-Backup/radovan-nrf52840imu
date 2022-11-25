/********************************************************************************************************
 * $Id$        sensor_data.h
 * @file       sensor_data.h
 * @brief      Sensor Data Header file
 * @date       16 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H


/***************************************** Macros *************************************************/
#define NUMBER_OF_CHANNELS  9

/***************************************** variable declaration**************************************/

/***********************************Function Declaration ******************************************/
void send_dummy_data(void);
void send(int8_t *channel_no);

#endif /* SENSOR_DATA_H */