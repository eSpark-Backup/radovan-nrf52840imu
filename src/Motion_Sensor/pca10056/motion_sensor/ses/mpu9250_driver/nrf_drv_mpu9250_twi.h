/********************************************************************************************************
 * $Id$        nrf_drv_mpu9250.h
 * @file       nrf_drv_mpu9250.h
 * @brief      MPU9250 I2C Driver Header file
 * @date       18 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#ifndef NRF_DRV_MPU9250_H
#define NRF_DRV_MPU9250_H


#include <stdbool.h>
#include <stdint.h>

/**@brief Function to initiate TWI drivers
 *
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu9250_twi_init(void);
	


/**@brief Function for reading an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @param[in]   length          Number of bytes to write
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu9250_write_registers(uint8_t reg, uint8_t * p_data, uint32_t length);



/**@brief Function for reading an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu9250_write_single_register(uint8_t reg, uint8_t data);

/**@brief Function for reading arbitrary register(s)
 *
 * @param[in]   reg             Register to read
 * @param[in]   p_data          Pointer to place to store value(s)
 * @param[in]   length          Number of registers to read
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu9250_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length);


/**@brief Function for reading single register
 *
 * @param[in]   reg             Register to read
 * @retval      uint8_t         Register value
 */
uint8_t nrf_drv_mpu9250_read_register(uint8_t reg);  


/**@brief Function for reading magnetometer registers(s)
 *
 * @param[in]   reg             Register to read
 * @param[in]   p_data          Pointer to place to store value(s)
 * @param[in]   length          Number of registers to read
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu9250_read_magnetometer_registers(uint8_t reg, uint8_t * p_data, uint32_t length);


/**@brief Function for writing magnetometer register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Data to be write in register
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_mpu9250_write_magnetometer_register(uint8_t reg, uint8_t data);

/**@brief Function for reading single register of magnetometer
 *
 * @param[in]   reg             Register to read
 * @retval      uint8_t         Register value
 */
uint8_t nrf_drv_mpu9250_read_magnetometer_register(uint8_t reg);

#endif /* NRF_DRV_MPU9250_H */

