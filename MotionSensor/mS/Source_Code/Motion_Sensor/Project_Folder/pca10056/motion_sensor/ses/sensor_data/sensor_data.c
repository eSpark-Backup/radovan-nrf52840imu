/********************************************************************************************************
 * $Id$        sensor_data.c
 * @file       sensor_data.c
 * @brief      Sensor Data Source file
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

#include "sensor_data.h"
#include "usbd_drv.h"

/**************************************************** variables ********************************************/

/******************************************Function definitions ********************************************/

/**@brief Function to send single data on USB Serial Port
 *
 * @details Send single data on USB Serial Port function
 *
 * @param[in]	 none
 * @param[out]   none
 */
void send(int8_t *channel_no)
{
  int8_t data[5];
  int len=0,cnt=0;
  memset(data,0,sizeof(data));

  for(cnt=0;cnt<NUMBER_OF_CHANNELS;cnt++){
    len=sprintf(data,"%d ",channel_no[cnt]);
    send_data_str(data,len);
  }
  send_data_str("\r\n",2);
}


/**@brief Function to send dummy data
 *
 * @details Send Dummy data on USB Serial Port function
 *
 * @param[in]	 none
 * @param[out]   none
 */
void send_dummy_data(void)
{
  int8_t channel[NUMBER_OF_CHANNELS];
  uint8_t i,j;
  int8_t val;

  // every column cycle
  for(j=0; j<NUMBER_OF_CHANNELS; j++) channel[j]=0;  // clear channels
  for(i=0; i<NUMBER_OF_CHANNELS; i++)
  { 
    for(val=  0; val<= 30; val+=5) { channel[i]=val; send(channel);}   // up to max value from 0
    for(val= 30; val>=-30; val-=5) { channel[i]=val; send(channel);}   // down to min value
    for(val=-30; val<=  0; val+=5) { channel[i]=val; send(channel);}   // up to 0
    channel[i]=0;                                                      // clean channel
  }
}

/**/

