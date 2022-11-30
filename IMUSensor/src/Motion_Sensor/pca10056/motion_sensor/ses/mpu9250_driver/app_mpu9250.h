/********************************************************************************************************
 * $Id$        app_mpu9250.h
 * @file       app_mpu9250.h
 * @brief      MPU9250 Driver Header file
 * @date       21 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/

#ifndef APP_MPU9250_H__
#define APP_MPU9250_H__


#include <stdbool.h>
#include <stdint.h>

#include "nrf_peripherals.h"

#include "mpu9250_register_map.h"

#define MPU_MG_PR_LSB_FF_THR    4


#define MPU_MPU_BASE_NUM    		0x4000
#define MPU_BAD_PARAMETER       	(MPU_MPU_BASE_NUM + 0) // An invalid paramameter has been passed to function.

#define CALIB_GYRO_SENSITIVITY  {131};     // LSB/degrees/sec
#define CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

/**@brief Enum defining Accelerometer's Full Scale range posibillities in Gs. */
typedef enum ACCEL_RANGE {
  AFS_2G = 0,       // 2 G
  AFS_4G,           // 4 G
  AFS_8G,           // 8 G
  AFS_16G           // 16 G
}ACCEL_RANGE_t;

/**@brief Enum defining Gyroscopes Full Scale range posibillities in Degrees Pr Second. */
typedef enum GYRO_RANGE {
  GFS_250DPS = 0,   // 250 deg/s
  GFS_500DPS,       // 500 deg/s
  GFS_1000DPS,      // 1000 deg/s
  GFS_2000DPS       // 2000 deg/s
}GYRO_RANGE_t;


enum GYRO_DLPF_CFG {
    GYRO_DLPF_250HZ,
    GYRO_DLPF_184HZ,
    GYRO_DLPF_92HZ,
    GYRO_DLPF_41HZ,
    GYRO_DLPF_20HZ,
    GYRO_DLPF_10HZ,
    GYRO_DLPF_5HZ,
    GYRO_DLPF_3600HZ,
};

enum ACCEL_DLPF_CFG  {
    ACCEL_DLPF_218HZ_0,
    ACCEL_DLPF_218HZ_1,
    ACCEL_DLPF_99HZ,
    ACCEL_DLPF_45HZ,
    ACCEL_DLPF_21HZ,
    ACCEL_DLPF_10HZ,
    ACCEL_DLPF_5HZ,
    ACCEL_DLPF_420HZ,
};

enum FIFO_SAMPLE_RATE {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

/*Sensor Data*/
typedef struct{
  int16_t z;
  int16_t y;
  int16_t x;
  double yaw;
  double pitch;
  double roll;
  double tilt;
}sensor_data_t;

/**@brief Structure to hold acceleromter values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}accel_values_t;


/**@brief Structure to hold gyroscope values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}gyro_values_t;


/**@brief MPU driver digital low pass fileter and external Frame Synchronization (FSYNC) pin sampling configuration structure */
typedef struct
{
    uint8_t dlpf_cfg     :3; // 3-bit unsigned value. Configures the Digital Low Pass Filter setting.
    uint8_t ext_sync_set :3; // 3-bit unsigned value. Configures the external Frame Synchronization (FSYNC) pin sampling.
    uint8_t fifo_mode    :1; // When set to 0, when the fifo is full, additional writes will not be written to fifo. When set to 1, when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
    uint8_t              :1;
}sync_dlpf_config_t;

/**@brief MPU driver gyro configuration structure. */
typedef struct
{
    uint8_t f_choice        :2;
    uint8_t                 :1;
    uint8_t fs_sel          :2; // FS_SEL 2-bit unsigned value. Selects the full scale range of gyroscopes.
    uint8_t gz_st           :1;
    uint8_t gy_st           :1;
    uint8_t gx_st           :1;

}gyro_config_t;

/**@brief MPU driver accelerometer configuration structure. */
typedef struct
{
    uint8_t                 :3;
    uint8_t afs_sel         :2; // 2-bit unsigned value. Selects the full scale range of accelerometers.
    uint8_t za_st           :1; // When set to 1, the Z- Axis accelerometer performs self test.
    uint8_t ya_st           :1; // When set to 1, the Y- Axis accelerometer performs self test.
    uint8_t xa_st           :1; // When set to 1, the X- Axis accelerometer performs self test.
}accel_config_t;

/**@brief MPU9250 driver accelerometer second configuration structure. */
typedef struct
{
    uint8_t a_dlpf_cfg      :2; // 3-bit unsigned value. Selects the Digital High Pass Filter configuration.
    uint8_t accel_f_choice_b :2; // 2-bit unsigned value. Selects the full scale range of accelerometers.
    uint8_t                 :4;
}accel_config_2_t;

#define MPU_DEFAULT_2_CONFIG()                          \
    {                                                     \
        .a_dlpf_cfg             = 0,              \
        .accel_f_choice_b       = 0,              \
    }

/**@brief MPU driver general configuration structure. */
typedef struct
{
    uint8_t             smplrt_div;         // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    sync_dlpf_config_t  sync_dlpf_gonfig;   // Digital low pass fileter and external Frame Synchronization (FSYNC) configuration structure
    gyro_config_t       gyro_config;        // Gyro configuration structure
    accel_config_t      accel_config;       // Accelerometer configuration structure
}app_mpu_config_t;

#define MPU_DEFAULT_CONFIG()                          \
    {                                                     \
        .smplrt_div                     = 7,              \
        .sync_dlpf_gonfig.dlpf_cfg      = 1,              \
        .sync_dlpf_gonfig.ext_sync_set  = 0,              \
        .gyro_config.fs_sel             = GFS_2000DPS,    \
        .gyro_config.f_choice           = 0,              \
        .gyro_config.gz_st              = 0,              \
        .gyro_config.gy_st              = 0,              \
        .gyro_config.gx_st              = 0,              \
        .accel_config.afs_sel           = AFS_16G,        \
        .accel_config.za_st             = 0,              \
        .accel_config.ya_st             = 0,              \
        .accel_config.xa_st             = 0,              \
    }
 
/**@brief MPU driver interrupt pin configuration structure. */    
typedef struct
{
    uint8_t clkout_en       :1;  // When this bit is equal to 1, a reference clock output is provided at the CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For further information regarding CLKOUT, please refer to the MPU-9150 Product Specification document.
    uint8_t i2c_bypass_en   :1;  // When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-9150. When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-9150 regardless of the state of I2C_MST_EN (Register 106 bit[5]).
    uint8_t fsync_int_en    :1;  // When equal to 0, this bit disables the FSYNC pin from causing an interrupt to the host processor. When equal to 1, this bit enables the FSYNC pin to be used as an interrupt to the host processor.
    uint8_t fsync_int_level :1;  // When this bit is equal to 0, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active high. When this bit is equal to 1, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active low.
    uint8_t int_rd_clear    :1;  // When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58). When this bit is equal to 1, interrupt status bits are cleared on any read operation.
    uint8_t latch_int_en    :1;  // When this bit is equal to 0, the INT pin emits a 50us long pulse. When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.
    uint8_t int_open        :1;  // When this bit is equal to 0, the INT pin is configured as push-pull. When this bit is equal to 1, the INT pin is configured as open drain.
    uint8_t int_level       :1; // When this bit is equal to 0, the logic level for the INT pin is active high. When this bit is equal to 1, the logic level for the INT pin is active low.
}app_mpu_int_pin_cfg_t;

/**@brief MPU interrupt pin default configuration. */
#define MPU_DEFAULT_INT_PIN_CONFIG()    \
{                                       \
    .clkout_en          = 0,    \
    .i2c_bypass_en      = 0,    \
    .fsync_int_en       = 0,    \
    .fsync_int_level    = 0,    \
    .int_rd_clear       = 1,    \
    .latch_int_en       = 0,    \
    .int_open           = 0,    \
    .int_level          = 0,    \
}

/**@brief MPU driver interrupt source configuration structure. */
typedef struct
{
    uint8_t data_rdy_en     :1; // When set to 1, this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed.
    uint8_t                 :2; // 
    uint8_t i2c_mst_int_en  :1; // When set to 1, this bit enables any of the I2C Master interrupt sources to generate an interrupt
    uint8_t fifo_oflow_en   :1; // When set to 1, this bit enables a FIFO buffer overflow to generate an interrupt.
    uint8_t zmot_en         :1; // When set to 1, this bit enables Zero Motion detection to generate an interrupt.
    uint8_t mot_en          :1; // When set to 1, this bit enables Motion detection to generate an interrupt.
    uint8_t ff_en           :1; // When set to 1, this bit enables Free Fall detection to generate an interrupt.
}app_mpu_int_enable_t;

/**@brief MPU interrupt sources default configuration. */
#define MPU_DEFAULT_INT_ENABLE_CONFIG() \
{                           \
    .data_rdy_en    = 0,    \
    .i2c_mst_int_en = 0,    \
    .fifo_oflow_en  = 0,    \
    .zmot_en        = 0,    \
    .mot_en         = 0,    \
    .ff_en          = 0,    \
}
 

/**@brief Function for initiating MPU and MPU library
 * 
 * Resets gyro, accelerometer and temperature sensor signal paths.
 * Function resets the analog and digital signal paths of the gyroscope, accelerometer,
 * and temperature sensors.
 * The reset will revert the signal path analog to digital converters and filters to their power up
 * configurations.
 *
 * @retval      uint32_t        Error code
 */
uint32_t app_mpu9250_init(void);

/**@brief Function for reading device id
 * Reads Device ID of MPU and verify it.
 *
 * @retval      bool      1= Device found, 0= Device not found
 */
bool app_mpu9250_whoami(void);    

/**@brief Function for basic configuring of the MPU
 * 
 * Register 25 Sample Rate Divider SMPRT_DIV. This register specifies 
 * the divider from the gyroscope output rate used to generate the Sample Rate for the MPU.
 * 
 * Register 26  Configuration CONFIG. This register configures 
 * the external Frame Synchronization (FSYNC) pin sampling and the Digital Low Pass Filter (DLPF) 
 * setting for both the gyroscopes and accelerometers.
 * 
 * Register 27  Gyroscope Configuration GYRO_CONFIG. This register is used to configure the gyroscopes full scale range.
 * 
 * Register 28 Accelerometer Configuration ACCEL_CONFIG. This register is 
 * used to trigger accelerometer self test and configure the accelerometer full scale range. 
 * This register also configures the Digital High Pass Filter (DHPF).
 *
 * @param[in]   configuration           Pointer to configuration structure
 * @retval      uint32_t                Error code
 */
uint32_t app_mpu9250_config(app_mpu_config_t * configuration);


/**@brief Function for configuring the behaviour of the interrupt pin of the MPU
 *
 * @param[in]   configuration     Pointer to configuration structure
 * @retval      uint32_t          Error code
 */
uint32_t app_mpu9250_int_cfg_pin(app_mpu_int_pin_cfg_t *configuration);


/**@brief Function for eneabling interrupts sources in MPU
 *
 * @param[in]   configuration          Pointer to configuration structure
 * @retval      uint32_t                Error code
 */
uint32_t app_mpu9250_int_enable(app_mpu_int_enable_t *configuration);

/**@brief Function for calibrating MPU9250
 *
 * @param[in]   None
 * @retval      None
 */    
void app_mpu9250_calibrate_acc_gyro(void);

/**@brief Function for reading MPU accelerometer data.
 *
 * @param[in]   accel_values    Pointer to variable to hold accelerometer data
 * @retval      uint32_t        Error code
 */
uint32_t app_mpu9250_read_accel(accel_values_t * accel_values);


/**@brief Function for reading MPU gyroscope data.
 *
 * @param[in]   gyro_values     Pointer to variable to hold gyroscope data
 * @retval      uint32_t        Error code
 */
uint32_t app_mpu9250_read_gyro(gyro_values_t * gyro_values);


/**@brief Function for reading MPU temperature data.
 *
 * @param[in]   temp_values     Pointer to variable to hold temperature data
 * @retval      uint32_t        Error code
 */
uint32_t app_mpu9250_read_int_source(uint8_t * int_source);

/**@brief Function for calibration configuration MPU9250
 *
 * @param[in]   None
 * @retval      None
 */
void app_mpu9250_cal_config(void);

/**@brief Function for setting acc and gyro calibration 
 *
 * @param[in]   None
 * @retval      None
 */
void app_mpu9250_set_acc_gyro_to_calibration(void);

/**@brief Function for getting aac and gyro data
 *
 * @param[in]   Accelerometer and gyro variables
 * @retval      None
 */
void app_mpu9250_collect_acc_gyro_data_to(float* a_bias, float* g_bias);

/**@brief Function for setting accelerometer offset
 *
 * @param[in]   None
 * @retval      None
 */
void app_mpu9250_write_accel_offset(void);

/**@brief Function for setting gyroscope offset
 *
 * @param[in]   None
 * @retval      None
 */
void app_mpu9250_write_gyro_offset(void);


/*********************************************************************************************************************
 * FUNCTIONS FOR MAGNETOMETER.
 * MPU9250 has AK8963 internal magnetometer.
 * AK8963 has adjustable resoultion (14 and 16 bits)
 */


/**@brief Enum defining possible magnetometer operating modes */
enum magn_op_mode {
	POWER_DOWN_MODE = 0,   				// Power to almost all internal circuits is turned off.
	SINGLE_MEASUREMENT_MODE,       		// Sensor is measured, and after sensor measurement and signal processing is finished, measurement data is stored to measurement data registers (HXL to HZH), then AK8963 transits to power-down mode automatically.
	CONTINUOUS_MEASUREMENT_8Hz_MODE,      //  Sensor is measured periodically at 8Hz
	EXTERNAL_TRIGGER_MODE = 4,       	// When external trigger measurement mode is set, AK89xx waits for trigger input. When a pulse is input from TRG pin, sensor measurement is started on the rising edge of TRG pin.
	CONTINUOUS_MEASUREMENT_100Hz_MODE = 6,  //  Sensor is measured periodically at 100Hz
	SELF_TEST_MODE = 8,					// Self-test mode is used to check if the sensor is working normally
	FUSE_ROM_ACCESS_MODE = 0xFF			// Fuse ROM access mode is used to read Fuse ROM data. Sensitivity adjustment data for each axis is stored in fuse ROM.
};

/**@brief Enum defining possible output bit resolutions for MPU9255 */
typedef enum MAG_RESOLUTION {
	OUTPUT_RESOLUTION_14bit = 0,
	OUTPUT_RESOLUTION_16bit
}MAG_RESOLUTION_t;


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}magn_values_t;

/**@brief Configuration structure used to set magnetometer operation mode
 * (and bit resolution for MPU9255).
 */
typedef struct
{
	uint8_t mode : 4;  
	uint8_t resolution : 1; // 0 = 14 but, 1 = 16 bit output
}app_mpu_magn_config_t;


/**@brief Structure to hold data read from MPU_AK89XX_REG_ST2 after reading sensor values.
 */
typedef struct
{
	uint8_t 			: 3;  
	uint8_t overflow 	: 1; //  single measurement mode, continuous measurement mode, external trigger measurement mode and self-test mode, magnetic sensor may overflow even though measurement data regiseter is not saturated. 
	uint8_t res_mirror 	: 1; // Output bit setting (mirror) 
}app_mpu_magn_read_status_t;


/**@brief Function for enabling and starting the magnetometer
 *
 * @param[in]   None
 * @retval      uint32_t        	Error code
 */
uint32_t app_mpu9250_magnetometer_init(void);


/**@brief Function for reading out magnetometer values
 *
 * @param[in]   magn_values_t *				Magnetometer values struct
 * @param[in]   app_mpu_magn_read_status_t *	Value of status register 2 (MPU_AK89XX_REG_ST2) after magnetometer data is read. NULL can be passed as argument if status is not needed
 * @retval      uint32_t     				Error code
 */
uint32_t app_mpu9250_read_magnetometer(magn_values_t * p_magnetometer_values, app_mpu_magn_read_status_t * p_read_status);

// Test function for development purposes
uint32_t app_mpu9250_read_magnetometer_test(uint8_t reg, uint8_t * registers, uint8_t len);


//bool app_mpu9250_read_mag(int16_t* destination);


/**@brief Function for getting mag data
 *
 * @param[in]   Magnetometer variables
 * @retval      None
 */
void app_mpu9250_collect_mag_data_to(float* m_bias, float* m_scale);

/**@brief Function for calibrating magnetometer
 *
 * @param[in]   None
 * @retval      None
 */
void app_mpu9250_calibrate_mag(void);

#endif /* APP_MPU9250_H__ */
