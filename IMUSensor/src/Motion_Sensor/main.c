/********************************************************************************************************
 * $Id$        main.c
 * @file       main.c
 * @brief      Motion sensor main file
 * @date       09 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"


#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "usbd_drv.h"
#include "timer_driver.h"
#include "sensor_data.h"
#include "app_mpu9250.h"

/**************************************** Variables *********************************************/    
sensor_data_t sensor_data;
/****************************************************************************************************/
/**@brief Function to send "Hello World" on USB
 *
 * @details This funtion sends test data on COM Port Generatd by USB
 *
 * @param[in]	 none
 * @param[out]   none
 */
void send_test_data(uint16_t cnt)
{
    char data[25];
    int len=0;
    memset(data,0,sizeof(data));
    len=sprintf(data,"Hello World: %d\r\n",cnt);
    send_data_str((char *)data,len);
}

/**@brief Function to initialize MPU9250
 *
 * @details This funtion initialize MPU9250 sensor and configure it
 *
 * @param[in]	 none
 * @param[out]   none
 */
void mpu_setup(void)
{
    NRF_LOG_INFO("MPU9250 Setup"); NRF_LOG_FLUSH();
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = app_mpu9250_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    /*app_mpu_config_t p_mpu_config;// = MPU9250_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
    ret_code = app_mpu9250_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value*/

    NRF_LOG_INFO("Accel Gyro calibration Start...Wait for 3 Seconds..."); NRF_LOG_FLUSH(); 
    NRF_LOG_INFO("Please leave the device still on the flat plane..."); NRF_LOG_FLUSH(); 
    app_mpu9250_calibrate_acc_gyro();
    nrf_delay_ms(3000);
    NRF_LOG_INFO("Accel Gyro calibration Done.."); NRF_LOG_FLUSH();

    NRF_LOG_INFO("Magnetometer calibration Start...Wait for 3 Seconds..."); NRF_LOG_FLUSH();   
    app_mpu9250_calibrate_mag();
    nrf_delay_ms(3000);
    NRF_LOG_INFO("Magnetometer calibration Done.."); NRF_LOG_FLUSH();

    NRF_LOG_INFO("MPU9250 Setup Done"); NRF_LOG_FLUSH();
}

void GetSensorData(void)
{
    accel_values_t acc_values;
    gyro_values_t gyro_values;
    magn_values_t magn_values;

    int8_t Data[100];
    int len=0;   

    double   Declination = +10.83;   //  Degrees ... replace this declination with yours
    double heading;
   
    uint32_t sample_number = 0;
    ret_code_t err_code;

    NRF_LOG_INFO("Fetching Sensor Data ----> "); NRF_LOG_FLUSH();
    // Read accelerometer sensor values
    //NRF_LOG_INFO("Read accelerometer sensor values"); NRF_LOG_FLUSH();
    err_code = app_mpu9250_read_accel(&acc_values);
    APP_ERROR_CHECK(err_code);
    sensor_data.x = acc_values.x;
    sensor_data.y = acc_values.y;
    sensor_data.z = acc_values.z;
    //gyro
    //NRF_LOG_INFO("Read gyro sensor values"); NRF_LOG_FLUSH();
    err_code = app_mpu9250_read_gyro(&gyro_values);
    APP_ERROR_CHECK(err_code);

    err_code = app_mpu9250_read_magnetometer(&magn_values, 0);
    APP_ERROR_CHECK(err_code);
  
    //NRF_LOG_INFO("Read Done "); NRF_LOG_FLUSH();

    NRF_LOG_INFO("Accel Sample # %d\r\nACC_X: %06d\r\nACC_Y: %06d\r\nACC_Z: %06d", ++sample_number, acc_values.x, acc_values.y, acc_values.z);NRF_LOG_FLUSH();
    NRF_LOG_INFO("Gyro Sample # %d\r\nGYRO_X: %06d\r\nGYRO_Y: %06d\r\nGYRO_Z: %06d", sample_number, gyro_values.x, gyro_values.y, gyro_values.z);NRF_LOG_FLUSH();
    NRF_LOG_INFO("Magno Sample # %d\r\nMGN_X: %06d\r\nMGN_Y: %06d\r\nMGN_Z: %06d", sample_number, magn_values.x, magn_values.y, magn_values.z);NRF_LOG_FLUSH();

    /* getting acc pitch and roll */
    sensor_data.pitch = atan2 (acc_values.y ,( sqrt ((acc_values.x * acc_values.x) + (acc_values.z * acc_values.z))));
    sensor_data.roll = atan2(-acc_values.x ,( sqrt((acc_values.y * acc_values.y) + (acc_values.z * acc_values.z))));

    sensor_data.roll = sensor_data.roll * 57.3;
    sensor_data.pitch = sensor_data.pitch * 57.3;


    /* getting yaw data */
    double Yh = (magn_values.y * cos(sensor_data.roll)) - (magn_values.z * sin(sensor_data.roll));
    double Xh = (magn_values.x * cos(sensor_data.pitch))+(magn_values.y * sin(sensor_data.roll)*sin(sensor_data.pitch)) + (magn_values.z * cos(sensor_data.roll) * sin(sensor_data.pitch));

    sensor_data.yaw =  atan2(Yh, Xh);

    /* Getting tilt data */
    heading = atan2( magn_values.y,  magn_values.x) * 57.3;
    
    sensor_data.tilt = heading + Declination;               // Geographic North
    if (sensor_data.tilt > 360.0) sensor_data.tilt -= 360.0;
    if (sensor_data.tilt < 0.0) sensor_data.tilt += 360.0;

    // ----- Allow for under/overflow
    if (sensor_data.tilt < 0) sensor_data.tilt += 360;
    if (sensor_data.tilt >= 360) sensor_data.tilt -= 360;

    /* Formatting data string */
    memset(Data,0,sizeof(Data));
    len+=sprintf(&Data[len],"%d.%02i,", (int) sensor_data.yaw, (int) fabs(((sensor_data.yaw - (int) sensor_data.yaw ) * 100))); 
    len+=sprintf(&Data[len],"%d.%02i,", (int) sensor_data.pitch, (int) fabs(((sensor_data.pitch - (int) sensor_data.pitch ) * 100))); 
    len+=sprintf(&Data[len],"%i,%i,%i,", sensor_data.x,sensor_data.y,sensor_data.z);
    len+=sprintf(&Data[len],"%d.%02i\r\n", (int) sensor_data.tilt, (int) fabs(((sensor_data.tilt - (int) sensor_data.tilt ) * 100))); 
    
    NRF_LOG_INFO("%s",Data);NRF_LOG_FLUSH();
    send_data_str((char *)Data,len);
}

/**@brief Main function
 *
 * @details This funtion includes all initialization of clock, usb drivers etc.
 *
 * @param[in]	 none
 * @param[out]   none
 */
int main(void)
{
    /* Debug Log Initalization */
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code); 
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    NRF_LOG_INFO("Device Start ----> "); NRF_LOG_FLUSH();

    /* USB Driver initalization */
    usbd_drv_init(); 

    /* Timer initialization*/
    timer_init();
    /* Start Timer */
    timer_start();

    /* Setup MPU9250 */
    mpu_setup();
 
    while (true)
    { 
        /* Fucntion to keep USB watch on USB Events*/
        usbd_drv_keep_powered();

        /* Every 500msec, f_Send flag set to true */
        if(f_Send){
          //send_dummy_data(); //send dummy data on USB COM Port
          GetSensorData();     //send sensor data on USB Port 
          f_Send=false;        // f_Send alg set to false 
        }
    }
}

/** @} */
