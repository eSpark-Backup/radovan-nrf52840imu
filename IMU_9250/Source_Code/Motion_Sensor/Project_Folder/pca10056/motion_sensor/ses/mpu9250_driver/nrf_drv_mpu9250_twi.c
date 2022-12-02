/********************************************************************************************************
 * $Id$        nrf_drv_mpu9250.c
 * @file       nrf_drv_mpu9250.c
 * @brief      MPU9250 I2C Driver Source file
 * @date       18 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_mpu9250_twi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Pins to connect MPU. Pinout is different for nRF52 DK
 * and therefore I have added a conditional statement defining different pins
 * for each board. This is only for my own convenience. 
 */
#define MPU_TWI_SCL_PIN 3
#define MPU_TWI_SDA_PIN 4


#define MPU_TWI_BUFFER_SIZE     	14 // 14 byte buffers will suffice to read acceleromter and gyroscope data in one transmission.
#define MPU_TWI_TIMEOUT 		10000 
#define MPU_ADDRESS     		0x68 
#define MPU_AK89XX_MAGN_ADDRESS         0x0C

#define TWI_INSTANCE_ID                 0

const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
volatile static bool xfer_done = false;

uint8_t twi_tx_buffer[MPU_TWI_BUFFER_SIZE];

static void nrf_drv_mpu9250_twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}



/**
 * @brief TWI initialization.
 */
uint32_t nrf_drv_mpu9250_twi_init(void)
{
    uint32_t err_code;
    
    NRF_LOG_INFO("MPU9250 TWI Init"); NRF_LOG_FLUSH();

    const nrf_drv_twi_config_t twi_mpu_config = {
       .scl                = MPU_TWI_SCL_PIN,
       .sda                = MPU_TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
       .clear_bus_init     = false
    };
    
    err_code = nrf_drv_twi_init(&m_twi_instance, &twi_mpu_config, nrf_drv_mpu9250_twi_event_handler, NULL);
    if(err_code != NRF_SUCCESS)
    {
            return err_code;
            NRF_LOG_INFO("MPU9250 TWI Init Failed!"); NRF_LOG_FLUSH();
    }
    
    nrf_drv_twi_enable(&m_twi_instance);

    NRF_LOG_INFO("MPU9250 TWI Init Done"); NRF_LOG_FLUSH();
	
    return NRF_SUCCESS;
}


// The TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
// Hence we need to merge the MPU register address with the buffer and then transmit all as one transmission
static void merge_register_and_data(uint8_t * new_buffer, uint8_t reg, uint8_t * p_data, uint32_t length)
{
    new_buffer[0] = reg;
    memcpy((new_buffer + 1), p_data, length);
}


uint32_t nrf_drv_mpu9250_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    // This burst write function is not optimal and needs improvement.
    // The new SDK 11 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    // Merging MPU register address and p_data into one buffer.
    merge_register_and_data(twi_tx_buffer, reg, p_data, length);

    // Setting up transfer
    nrf_drv_twi_xfer_desc_t xfer_desc;
    xfer_desc.address = MPU_ADDRESS;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = twi_tx_buffer;

     xfer_done = false; 
      
    // Transferring
    err_code = nrf_drv_twi_xfer(&m_twi_instance, &xfer_desc, 0);
    while(!xfer_done); 

    xfer_done = false; 
    
    return err_code;
}

uint32_t nrf_drv_mpu9250_write_single_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    uint8_t packet[2] = {reg, data};
    
    xfer_done = false; 

    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_ADDRESS, packet, 2, false);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done); 

    xfer_done = false;  
    return err_code;
}

uint8_t nrf_drv_mpu9250_read_register(uint8_t reg)
{
    uint8_t p_data;
    uint32_t err_code;
    xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_ADDRESS, &reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);

    xfer_done = false; 
    err_code = nrf_drv_twi_rx(&m_twi_instance, MPU_ADDRESS, &p_data, 1);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);

    xfer_done = false;
    return p_data;
}


uint32_t nrf_drv_mpu9250_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    xfer_done = false;  
    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_ADDRESS, &reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);
    
    xfer_done = false;    
    err_code = nrf_drv_twi_rx(&m_twi_instance, MPU_ADDRESS, p_data, length);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);
   
    xfer_done = false;  
    
    return err_code;
}


#if (TWI_COUNT >= 1) // Magnetometer only works with TWI so check if TWI is enabled

uint8_t nrf_drv_mpu9250_read_magnetometer_register(uint8_t reg)
{
    uint8_t p_data;
    uint32_t err_code;
    xfer_done = false;

    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_AK89XX_MAGN_ADDRESS, &reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);

    xfer_done = false; 
    err_code = nrf_drv_twi_rx(&m_twi_instance, MPU_AK89XX_MAGN_ADDRESS, &p_data, 1);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);

    xfer_done = false;
    return p_data;
}

uint32_t nrf_drv_mpu9250_read_magnetometer_registers(uint8_t reg, uint8_t * p_data, uint32_t length)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    xfer_done = false;    
    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_AK89XX_MAGN_ADDRESS, &reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);

    xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi_instance, MPU_AK89XX_MAGN_ADDRESS, p_data, length);
    APP_ERROR_CHECK(err_code);
    while(!xfer_done);
    
    xfer_done = false;
    
    return err_code;
}


uint32_t nrf_drv_mpu9250_write_magnetometer_register(uint8_t reg, uint8_t data)
{
    uint32_t err_code;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    uint8_t packet[2] = {reg, data};
    xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_instance, MPU_AK89XX_MAGN_ADDRESS, packet, 2, false);
    if(err_code != NRF_SUCCESS) return err_code;
    while(!xfer_done);

    xfer_done = false;

    return err_code;
}

#endif // (TWI_COUNT >= 1) // Magnetometer only works with TWI so check if TWI is enabled

/**/