/********************************************************************************************************
 * $Id$        app_mpu9250.c
 * @file       app_mpu9250.c
 * @brief      MPU9250 Driver Source file
 * @date       21 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "app_mpu9250.h"
#include "nrf_gpio.h"
#include "nrf_drv_mpu9250_twi.h"
#include "nrf_error.h"
#include "nrf_peripherals.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/******************************************** Variables ************************************************/
// Calibration Parameters
float acc_bias[3]  = {0, 0, 0};   // acc calibration value in ACCEL_FS_SEL: 2g
float gyro_bias[3] = {0, 0, 0};  // gyro calibration value in GYRO_FS_SEL: 250dps

// IMU Data
float a[3] = {0, 0, 0};
float g[3] = {0, 0, 0};
float m[3] = {0, 0, 0};
float q[4] = {1, 0, 0, 0};  // vector to hold quaternion
float rpy[3] = {0, 0, 0};
float lin_acc[3] = {0, 0, 0};  // linear acceleration (acceleration with gravity component subtracted)

float acc_resolution  = 0;       // scale resolutions per LSB for the sensors
float gyro_resolution = 0;       // scale resolutions per LSB for the sensors
float mag_resolution  = 0;       // scale resolutions per LSB for the sensors

float mag_bias_factory[3] = {0, 0, 0};
float mag_bias[3] = {0, 0, 0};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
float mag_scale[3] = {1, 1, 1};
float magnetic_declination = 0;  

uint8_t MAG_MODE = 0x06;

/*********************************************************************************************************************
 * FUNCTIONS FOR GYRO and ACCELEROMETER.
 */

bool app_mpu9250_whoami(void)
{
    uint8_t device_id;

    device_id = nrf_drv_mpu9250_read_register(MPU9250_REG_WHO_AM_I);
    NRF_LOG_INFO("Device ID: %X ", device_id); NRF_LOG_FLUSH();
    if((device_id == 0x68) || (device_id == 0x71)) return true;
    else return false;
}

uint32_t app_mpu9250_config(app_mpu_config_t * configuration)
{
    uint8_t *config_data;
    config_data = (uint8_t*)configuration;
    return nrf_drv_mpu9250_write_registers(MPU9250_REG_SMPLRT_DIV, config_data, 4);
}



uint32_t app_mpu9250_int_cfg_pin(app_mpu_int_pin_cfg_t *configuration)
{
    uint8_t *config_data;
    config_data = (uint8_t*)configuration;
    return nrf_drv_mpu9250_write_single_register(MPU9250_REG_INT_PIN_CFG, *config_data);
    
}



uint32_t app_mpu9250_int_enable(app_mpu_int_enable_t *configuration)
{
    uint8_t *config_data;
    config_data = (uint8_t*)configuration;
    return nrf_drv_mpu9250_write_single_register(MPU9250_REG_INT_ENABLE, *config_data);
}



uint32_t app_mpu9250_init(void)
{
    uint32_t ret_code;
	
    // Initate TWI or SPI driver dependent on what is defined from the project
    ret_code = nrf_drv_mpu9250_twi_init();
    if(ret_code != NRF_SUCCESS) return ret_code;

    if(app_mpu9250_whoami()){
        NRF_LOG_INFO("MPU9250 Found "); NRF_LOG_FLUSH();
    }
    else{
        NRF_LOG_INFO("MPU9250 Not Found "); NRF_LOG_FLUSH();
    }

    return NRF_SUCCESS;
}



uint32_t app_mpu9250_read_accel(accel_values_t * accel_values)
{
    uint32_t ret_code;
    uint8_t raw_values[6];
    ret_code = nrf_drv_mpu9250_read_registers(MPU9250_REG_ACCEL_XOUT_H, raw_values, 6);
    if(ret_code != NRF_SUCCESS) return ret_code;

    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}



uint32_t app_mpu9250_read_gyro(gyro_values_t * gyro_values)
{
    uint32_t ret_code;
    uint8_t raw_values[6];
    ret_code = nrf_drv_mpu9250_read_registers(MPU9250_REG_GYRO_XOUT_H, raw_values, 6);
    if(ret_code != NRF_SUCCESS) return ret_code;

    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)gyro_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}


uint32_t app_mpu9250_read_int_source(uint8_t * int_source)
{
    return nrf_drv_mpu9250_read_registers(MPU9250_REG_INT_STATUS, int_source, 1);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// ACCEL_FS_SEL: 2g (maximum sensitivity)
// GYRO_FS_SEL: 250dps (maximum sensitivity)
void app_mpu9250_calibrate_acc_gyro(void) {
    app_mpu9250_set_acc_gyro_to_calibration();
    app_mpu9250_collect_acc_gyro_data_to(acc_bias, gyro_bias);
    app_mpu9250_write_accel_offset();
    app_mpu9250_write_gyro_offset();
    nrf_delay_ms(100);
    app_mpu9250_cal_config(); 
    nrf_delay_ms(1000);
}




void app_mpu9250_set_acc_gyro_to_calibration(void) {
    // reset device
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    nrf_delay_ms(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_PWR_MGMT_1, 0x01);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_PWR_MGMT_2, 0x00);
    nrf_delay_ms(200);

    // Configure device for bias calculation
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_INT_ENABLE, 0x00);    // Disable all interrupts
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_FIFO_EN, 0x00);       // Disable FIFO
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_PWR_MGMT_1, 0x00);    // Turn on internal clock source
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_I2C_MST_CTRL, 0x00);  // Disable I2C master
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_USER_CTRL, 0x0C);     // Reset FIFO and DMP
    nrf_delay_ms(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_CONFIG, 0x01);         // Set low-pass filter to 188 Hz
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_SMPLRT_DIV, 0x00);     // Set sample rate to 1 kHz
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_GYRO_CONFIG, 0x00);    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_ACCEL_CONFIG, 0x00);   // Set accelerometer full-scale to 2 g, maximum sensitivity

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_USER_CTRL, 0x40);  // Enable FIFO
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    nrf_delay_ms(40);                                  // accumulate 40 samples in 40 milliseconds = 480 bytes
}

void app_mpu9250_collect_acc_gyro_data_to(float* a_bias, float* g_bias) {
    // At end of sample accumulation, turn off FIFO sensor read
    uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
    nrf_drv_mpu9250_read_registers(MPU9250_REG_FIFO_COUNTH, &data[0], sizeof(data));  // read FIFO sample count
    uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
    uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

    for (uint16_t ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        nrf_drv_mpu9250_read_registers(MPU9250_REG_FIFO_R_W, &data[0], sizeof(data));              // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        a_bias[1] += (float)accel_temp[1];
        a_bias[2] += (float)accel_temp[2];
        g_bias[0] += (float)gyro_temp[0];
        g_bias[1] += (float)gyro_temp[1];
        g_bias[2] += (float)gyro_temp[2];
    }
    a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
    a_bias[1] /= (float)packet_count;
    a_bias[2] /= (float)packet_count;
    g_bias[0] /= (float)packet_count;
    g_bias[1] /= (float)packet_count;
    g_bias[2] /= (float)packet_count;

    if (a_bias[2] > 0L) {
        a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
    }
}

void app_mpu9250_write_accel_offset(void) {
    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};                      // A place to hold the factory accelerometer trim biases
    nrf_drv_mpu9250_read_registers(MPU9250_REG_XA_OFFSET_H, &read_data[0], sizeof(read_data));  // Read factory accelerometer trim values
    acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
    nrf_drv_mpu9250_read_registers(MPU9250_REG_YA_OFFSET_H, &read_data[0], sizeof(read_data));
    acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
    nrf_drv_mpu9250_read_registers(MPU9250_REG_ZA_OFFSET_H, &read_data[0], sizeof(read_data));
    acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

    int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
    for (int i = 0; i < 3; i++) {
        if (acc_bias_reg[i] % 2) {
            mask_bit[i] = 0;
        }
        acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
        if (mask_bit[i]) {
            acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
        } else {
            acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
        }
    }

    uint8_t write_data[6] = {0};
    write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    write_data[1] = (acc_bias_reg[0]) & 0xFF;
    write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    write_data[3] = (acc_bias_reg[1]) & 0xFF;
    write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    write_data[5] = (acc_bias_reg[2]) & 0xFF;

    // Push accelerometer biases to hardware registers
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_XA_OFFSET_H, write_data[0]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_XA_OFFSET_L, write_data[1]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_YA_OFFSET_H, write_data[2]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_YA_OFFSET_L, write_data[3]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_ZA_OFFSET_H, write_data[4]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_ZA_OFFSET_L, write_data[5]);
}

void app_mpu9250_write_gyro_offset(void) {
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    uint8_t gyro_offset_data[6] = {0};
    gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
    gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_XG_OFFSET_H, gyro_offset_data[0]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_XG_OFFSET_L, gyro_offset_data[1]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_YG_OFFSET_H, gyro_offset_data[2]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_YG_OFFSET_L, gyro_offset_data[3]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_ZG_OFFSET_H, gyro_offset_data[4]);
    nrf_drv_mpu9250_write_single_register( MPU9250_REG_ZG_OFFSET_L, gyro_offset_data[5]);
}


float app_mpu9250_get_acc_resolution(ACCEL_RANGE_t accel_af_sel) {
    switch (accel_af_sel) {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            return 2.0 / 32768.0;
        case AFS_4G:
            return 4.0 / 32768.0;
        case AFS_8G:
            return 8.0 / 32768.0;
        case AFS_16G:
            return 16.0 / 32768.0;
        default:
            return 0.0;
    }
}

float app_mpu9250_get_gyro_resolution(GYRO_RANGE_t gyro_fs_sel) {
    switch (gyro_fs_sel) {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            return 250.0 / 32768.0;
        case GFS_500DPS:
            return 500.0 / 32768.0;
        case GFS_1000DPS:
            return 1000.0 / 32768.0;
        case GFS_2000DPS:
            return 2000.0 / 32768.0;
        default:
            return 0.0;
    }
}

void app_mpu9250_cal_config(void) {
    acc_resolution = app_mpu9250_get_acc_resolution(AFS_16G);
    gyro_resolution = app_mpu9250_get_gyro_resolution(GFS_2000DPS);

    // reset device
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    nrf_delay_ms(100);

    // wake up device
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
    nrf_delay_ms(100);                                  // Wait for all registers to reset

    // get stable time source
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    nrf_delay_ms(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    uint8_t mpu_config = GYRO_DLPF_41HZ;
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_CONFIG, mpu_config);

    // Set sample rate = gyroscope output rate/(1 + MPU9250_REG_SMPLRT_DIV)
    uint8_t sample_rate = SMPL_200HZ;
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_SMPLRT_DIV, sample_rate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                        // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = nrf_drv_mpu9250_read_register(MPU9250_REG_GYRO_CONFIG);  // get current GYRO_CONFIG register value
    c = c & ~0xE0;                                     // Clear self-test bits [7:5]
    c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
    c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
    c = c | (GFS_2000DPS << 3);                        // Set full scale range for the gyro
    c = c | (0x03);   // Set Fchoice for the gyro
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_GYRO_CONFIG, c);          // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = nrf_drv_mpu9250_read_register(MPU9250_REG_ACCEL_CONFIG);     // get current ACCEL_CONFIG register value
    c = c & ~0xE0;                                 // Clear self-test bits [7:5]
    c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
    c = c | (AFS_16G << 3);  // Set full scale range for the accelerometer
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_ACCEL_CONFIG, c);     // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = nrf_drv_mpu9250_read_register(MPU9250_REG_ACCEL_CONFIG_2);        // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | (~(0x01) & 0x08);    // Set accel_fchoice_b to 1
    c = c | (ACCEL_DLPF_45HZ & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_ACCEL_CONFIG_2, c);        // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_INT_PIN_CFG, 0x22);
    nrf_drv_mpu9250_write_single_register(MPU9250_REG_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    nrf_delay_ms(100);
}

/*********************************************************************************************************************
 * FUNCTIONS FOR MAGNETOMETER.
 * MPU9150 has an AK8975C and MPU9255 an AK8963 internal magnetometer. Their register maps
 * are similar, but AK8963 has adjustable resoultion (14 and 16 bits) while AK8975C has 13 bit resolution fixed. 
 */

uint32_t app_mpu9250_magnetometer_init(void)
{	
	uint32_t ret_code;
	
        // First extract the factory calibration for each magnetometer axis
        uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here

        nrf_drv_mpu9250_write_magnetometer_register(MPU9250_AK89XX_REG_CNTL, 0x00);  // Power down magnetometer
        nrf_delay_ms(10);
        nrf_drv_mpu9250_write_magnetometer_register(MPU9250_AK89XX_REG_CNTL, 0x0F);  // Enter Fuse ROM access mode
        nrf_delay_ms(10);
        nrf_drv_mpu9250_read_magnetometer_registers(MPU9250_AK89XX_REG_ASAX, &raw_data[0], sizeof(raw_data));      // Read the x-, y-, and z-axis calibration values
        mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
        mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
        mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
        nrf_drv_mpu9250_write_magnetometer_register(MPU9250_AK89XX_REG_CNTL, 0x00);  // Power down magnetometer
        nrf_delay_ms(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        ret_code = nrf_drv_mpu9250_write_magnetometer_register(MPU9250_AK89XX_REG_CNTL, (OUTPUT_RESOLUTION_16bit << 4) | MAG_MODE);  // Set magnetometer data resolution and sample ODR
        nrf_delay_ms(10);

        return ret_code;
}

uint32_t app_mpu9250_read_magnetometer(magn_values_t * p_magnetometer_values, app_mpu_magn_read_status_t * p_read_status)
{
	uint32_t ret_code;
	ret_code = nrf_drv_mpu9250_read_magnetometer_registers(MPU9250_AK89XX_REG_HXL, (uint8_t *)p_magnetometer_values, 6);
	if(ret_code != NRF_SUCCESS) return ret_code;
        
	/* Quote from datasheet: MPU9250_AK89XX_REG_ST2 register has a role as data reading end register, also. When any of measurement data register is read
	in continuous measurement mode or external trigger measurement mode, it means data reading start and
	taken as data reading until ST2 register is read. Therefore, when any of measurement data is read, be
	sure to read ST2 register at the end. */
	if(p_read_status == NULL)
	{
		// If p_read_status equals NULL perform dummy read
		uint8_t status_2_reg;
		ret_code = nrf_drv_mpu9250_read_magnetometer_registers(MPU9250_AK89XX_REG_ST2, &status_2_reg, 1);
	}
	else
	{
		// If p_read_status NOT equals NULL read and return value of MPU9250_AK89XX_REG_ST2
		ret_code = nrf_drv_mpu9250_read_magnetometer_registers(MPU9250_AK89XX_REG_ST2, (uint8_t *)p_read_status, 1);
	}
	return ret_code;
}

// Test function for development purposes
uint32_t app_mpu9250_read_magnetometer_test(uint8_t reg, uint8_t * registers, uint8_t len)
{
    return nrf_drv_mpu9250_read_magnetometer_registers(reg, registers, len);
}

float app_mpu9250_get_mag_resolution(MAG_RESOLUTION_t mag_output_bits){
    switch (mag_output_bits) {
        // Possible magnetometer scales (and their register bit settings) are:
        // 14 bit resolution (0) and 16 bit resolution (1)
        // Proper scale to return milliGauss
        case OUTPUT_RESOLUTION_14bit:
            return 10. * 4912. / 8190.0;
        case OUTPUT_RESOLUTION_16bit:
            return 10. * 4912. / 32760.0;
        default:
            return 0.;
    }
}


bool app_mpu9250_read_mag(int16_t* destination) {
    const uint8_t st1 = nrf_drv_mpu9250_read_magnetometer_register(MPU9250_AK89XX_REG_ST1);
    if (st1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
        uint8_t raw_data[7];                                             // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        nrf_drv_mpu9250_read_magnetometer_registers(MPU9250_AK89XX_REG_HXL,&raw_data[0],sizeof(raw_data));      // Read the six raw data and ST2 registers sequentially into data array
        if (MAG_MODE == 0x02 || MAG_MODE == 0x04 || MAG_MODE == 0x06) {  // continuous or external trigger read mode
            if ((st1 & 0x02) != 0)                                       // check if data is not skipped
                return false;                                            // this should be after data reading to clear DRDY register
        }

        uint8_t c = raw_data[6];                                         // End data read by reading ST2 register
        if (!(c & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
            destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
            return true;
        }
    }
    return false;
}

void app_mpu9250_collect_mag_data_to(float* m_bias, float* m_scale){
    // shoot for ~fifteen seconds of mag data
    uint16_t sample_count = 0;
    if (MAG_MODE == 0x02)
        sample_count = 128;     // at 8 Hz ODR, new mag data is available every 125 ms
    else if (MAG_MODE == 0x06)  // in this library, fixed to 100Hz
        sample_count = 1500;    // at 100 Hz ODR, new mag data is available every 10 ms

    int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767, 32767};
    int16_t mag_temp[3] = {0, 0, 0};
    for (uint16_t ii = 0; ii < sample_count; ii++) {
        app_mpu9250_read_mag(mag_temp);  // Read the mag data
        for (int jj = 0; jj < 3; jj++) {
            if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if (MAG_MODE == 0x02) nrf_delay_ms(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        if (MAG_MODE == 0x06) nrf_delay_ms(12);   // at 100 Hz ODR, new mag data is available every 10 ms
    }


    // Get hard iron correction
    bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
    bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
    bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

    float bias_resolution = app_mpu9250_get_mag_resolution(OUTPUT_RESOLUTION_16bit);
    m_bias[0] = (float)bias[0] * bias_resolution * mag_bias_factory[0];  // save mag biases in G for main program
    m_bias[1] = (float)bias[1] * bias_resolution * mag_bias_factory[1];
    m_bias[2] = (float)bias[2] * bias_resolution * mag_bias_factory[2];

    // Get soft iron correction estimate
    //*** multiplication by mag_bias_factory added in accordance with the following comment
    //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
    scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_bias_factory[0] / 2;  // get average x axis max chord length in counts
    scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_bias_factory[1] / 2;  // get average y axis max chord length in counts
    scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_bias_factory[2] / 2;  // get average z axis max chord length in counts

    float avg_rad = scale[0] + scale[1] + scale[2];
    avg_rad /= 3.0;

    m_scale[0] = avg_rad / ((float)scale[0]);
    m_scale[1] = avg_rad / ((float)scale[1]);
    m_scale[2] = avg_rad / ((float)scale[2]);
}

void app_mpu9250_calibrate_mag(void){
// set MAG_OUTPUT_BITS to maximum to calibrate
    MAG_RESOLUTION_t mag_output_bits_cache = app_mpu9250_get_mag_resolution(OUTPUT_RESOLUTION_16bit);
    app_mpu9250_magnetometer_init();
    app_mpu9250_collect_mag_data_to(mag_bias, mag_scale);
    nrf_delay_ms(2000); 
    app_mpu9250_magnetometer_init();
}

/**/
