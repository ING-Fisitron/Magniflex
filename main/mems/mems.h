/*
 * test_i2c_snsr.h
 *
 *  Created on: Feb 21, 2019
 *      Author: leo
 */

#ifndef MAIN_TEST_I2C_SNSR_H_
#define MAIN_TEST_I2C_SNSR_H_

#include "driver/i2c.h"
#include "mems/mems_env_sensors.h"
#include "mems/mems_env_sensors_ex.h"
#include "mems/mems_motion_sensors.h"
#include "mems/mems_motion_sensors_ex.h"

// HW_1_3
#define MEMS_I2C_SCL 	21				/*!< GPIO number for I2C master clock */
#define MEMS_I2C_SDA 	18				/*!< GPIO number for I2C master data  */

#define MEMS_I2C_FREQ_HZ 			100000 		/*!< I2C master clock frequency */
#define MEMS_I2C_PORT_NUM 			I2C_NUM_0	/*!< I2C port number for master DEV */
#define MEMS_I2C_TX_BUF_DISABLE 	0   		/*!< I2C master do not need buffer */
#define MEMSE_I2C_RX_BUF_DISABLE 	0 			/*!< I2C master do not need buffer */

#define MESM_DATA_LENGTH 					512		/*!< Data buffer length of test buffer */
#define MEMS_RW_TEST_LENGTH 				128		/*!< Data length for r/w test, [0,DATA_LENGTH] */
#define MEMS_DELAY_TIME_BETWEEN_ITEMS_MS 	1000 	/*!< delay time between different test items */

#define MEMS_I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)	/*!< I2C slave TX buffer size */
#define MEMS_I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)	/*!< I2C slave RX buffer size */

#define WRITE_BIT 			I2C_MASTER_WRITE 	/*!< I2C master write */
#define READ_BIT 			I2C_MASTER_READ		/*!< I2C master read */
#define ACK_CHECK_EN 		0x1					/*!< I2C master will check ACK from slave*/
#define ACK_CHECK_DIS 		0x0					/*!< I2C master will not check ACK from slave */
#define ACK_VAL 			0x0           		/*!< I2C ACK value */
#define NACK_VAL 			0x1           		/*!< I2C NACK value */


/* -------------------------------------------------------------------------------------------
* **************************************** LSM6DS3US ****************************************
* ---------------------------------------------------------------------------------------- */
////
////#define LSM6DS3US_ACC_GYRO_ADDR  (0b1101011)	/* slave address for LSM6DS3US sensor */
////#define LSM6DS3US_ACC_GYRO_WHO_AM_I    0X69
////
/////************** Device Register  *******************/
////
////#define LSM6DS3US_ACC_GYRO_FUNC_CFG_ACCESS    0X01
////
////#define LSM6DS3US_ACC_GYRO_SENSOR_SYNC_TIME   0X04
////#define LSM6DS3US_ACC_GYRO_SENSOR_RES_RATIO   0X05
////
////#define LSM6DS3US_ACC_GYRO_FIFO_CTRL1   0X06
////#define LSM6DS3US_ACC_GYRO_FIFO_CTRL2   0X07
////#define LSM6DS3US_ACC_GYRO_FIFO_CTRL3   0X08
////#define LSM6DS3US_ACC_GYRO_FIFO_CTRL4   0X09
////#define LSM6DS3US_ACC_GYRO_FIFO_CTRL5   0X0A
////
////#define LSM6DS3US_ACC_GYRO_ORIENT_CFG_G   0X0B
////
////#define LSM6DS3US_ACC_GYRO_INT1_CTRL    0X0D
////#define LSM6DS3US_ACC_GYRO_INT2_CTRL    0X0E
////
////#define LSM6DS3US_ACC_GYRO_WHO_AM_I_REG   0X0F
////
////#define LSM6DS3US_ACC_GYRO_CTRL1_XL   0X10
////#define LSM6DS3US_ACC_GYRO_CTRL2_G    0X11
////#define LSM6DS3US_ACC_GYRO_CTRL3_C    0X12
////#define LSM6DS3US_ACC_GYRO_CTRL4_C    0X13
////#define LSM6DS3US_ACC_GYRO_CTRL5_C    0X14
////#define LSM6DS3US_ACC_GYRO_CTRL6_G    0X15
////#define LSM6DS3US_ACC_GYRO_CTRL7_G    0X16
////#define LSM6DS3US_ACC_GYRO_CTRL8_XL   0X17
////#define LSM6DS3US_ACC_GYRO_CTRL9_XL   0X18
////#define LSM6DS3US_ACC_GYRO_CTRL10_C   0X19
////
////#define LSM6DS3US_ACC_GYRO_MASTER_CONFIG	0X1A 	/* LSM6DS3US i2c master configuration */
////#define LSM6DS3US_ACC_GYRO_WAKE_UP_SRC   	0X1B
////#define LSM6DS3US_ACC_GYRO_TAP_SRC    		0X1C
////#define LSM6DS3US_ACC_GYRO_D6D_SRC    		0X1D
////#define LSM6DS3US_ACC_GYRO_STATUS_REG   	0X1E
////
////#define LSM6DS3US_ACC_GYRO_OUT_TEMP_L   0X20
////#define LSM6DS3US_ACC_GYRO_OUT_TEMP_H   0X21
////
////#define LSM6DS3US_ACC_GYRO_OUTX_L_G   0X22
////#define LSM6DS3US_ACC_GYRO_OUTX_H_G   0X23
////#define LSM6DS3US_ACC_GYRO_OUTY_L_G   0X24
////#define LSM6DS3US_ACC_GYRO_OUTY_H_G   0X25
////#define LSM6DS3US_ACC_GYRO_OUTZ_L_G   0X26
////#define LSM6DS3US_ACC_GYRO_OUTZ_H_G   0X27
////
////#define LSM6DS3US_ACC_GYRO_OUTX_L_XL    0X28
////#define LSM6DS3US_ACC_GYRO_OUTX_H_XL    0X29
////#define LSM6DS3US_ACC_GYRO_OUTY_L_XL    0X2A
////#define LSM6DS3US_ACC_GYRO_OUTY_H_XL    0X2B
////#define LSM6DS3US_ACC_GYRO_OUTZ_L_XL    0X2C
////#define LSM6DS3US_ACC_GYRO_OUTZ_H_XL    0X2D
////
////#define LSM6DS3US_ACC_GYRO_SENSORHUB1_REG   0X2E
////#define LSM6DS3US_ACC_GYRO_SENSORHUB2_REG   0X2F
////#define LSM6DS3US_ACC_GYRO_SENSORHUB3_REG   0X30
////#define LSM6DS3US_ACC_GYRO_SENSORHUB4_REG   0X31
////#define LSM6DS3US_ACC_GYRO_SENSORHUB5_REG   0X32
////#define LSM6DS3US_ACC_GYRO_SENSORHUB6_REG   0X33
////#define LSM6DS3US_ACC_GYRO_SENSORHUB7_REG   0X34
////#define LSM6DS3US_ACC_GYRO_SENSORHUB8_REG   0X35
////#define LSM6DS3US_ACC_GYRO_SENSORHUB9_REG   0X36
////#define LSM6DS3US_ACC_GYRO_SENSORHUB10_REG    0X37
////#define LSM6DS3US_ACC_GYRO_SENSORHUB11_REG    0X38
////#define LSM6DS3US_ACC_GYRO_SENSORHUB12_REG    0X39
////
////#define LSM6DS3US_ACC_GYRO_FIFO_STATUS1   0X3A
////#define LSM6DS3US_ACC_GYRO_FIFO_STATUS2   0X3B
////#define LSM6DS3US_ACC_GYRO_FIFO_STATUS3   0X3C
////#define LSM6DS3US_ACC_GYRO_FIFO_STATUS4   0X3D
////#define LSM6DS3US_ACC_GYRO_FIFO_DATA_OUT_L    0X3E
////#define LSM6DS3US_ACC_GYRO_FIFO_DATA_OUT_H    0X3F
////
////#define LSM6DS3US_ACC_GYRO_TIMESTAMP0_REG   0X40
////#define LSM6DS3US_ACC_GYRO_TIMESTAMP1_REG   0X41
////#define LSM6DS3US_ACC_GYRO_TIMESTAMP2_REG   0X42
////
////#define LSM6DS3US_ACC_GYRO_TIMESTAMP_L    0X49
////#define LSM6DS3US_ACC_GYRO_TIMESTAMP_H    0X4A
////
////#define LSM6DS3US_ACC_GYRO_STEP_COUNTER_L   0X4B
////#define LSM6DS3US_ACC_GYRO_STEP_COUNTER_H   0X4C
////
////#define LSM6DS3US_ACC_GYRO_SENSORHUB13_REG    0X4D
////#define LSM6DS3US_ACC_GYRO_SENSORHUB14_REG    0X4E
////#define LSM6DS3US_ACC_GYRO_SENSORHUB15_REG    0X4F
////#define LSM6DS3US_ACC_GYRO_SENSORHUB16_REG    0X50
////#define LSM6DS3US_ACC_GYRO_SENSORHUB17_REG    0X51
////#define LSM6DS3US_ACC_GYRO_SENSORHUB18_REG    0X52
////
////#define LSM6DS3US_ACC_GYRO_FUNC_SRC   0X53
////#define LSM6DS3US_ACC_GYRO_TAP_CFG1   0X58
////#define LSM6DS3US_ACC_GYRO_TAP_THS_6D   0X59
////#define LSM6DS3US_ACC_GYRO_INT_DUR2   0X5A
////#define LSM6DS3US_ACC_GYRO_WAKE_UP_THS    0X5B
////#define LSM6DS3US_ACC_GYRO_WAKE_UP_DUR    0X5C
////#define LSM6DS3US_ACC_GYRO_FREE_FALL    0X5D
////#define LSM6DS3US_ACC_GYRO_MD1_CFG    0X5E
////#define LSM6DS3US_ACC_GYRO_MD2_CFG    0X5F
////
////#define LSM6DS3US_ACC_GYRO_OUT_MAG_RAW_X_L    0X66
////#define LSM6DS3US_ACC_GYRO_OUT_MAG_RAW_X_H    0X67
////#define LSM6DS3US_ACC_GYRO_OUT_MAG_RAW_Y_L    0X68
////#define LSM6DS3US_ACC_GYRO_OUT_MAG_RAW_Y_H    0X69
////#define LSM6DS3US_ACC_GYRO_OUT_MAG_RAW_Z_L    0X6A
////#define LSM6DS3US_ACC_GYRO_OUT_MAG_RAW_Z_H    0X6B
////
////
////typedef unsigned char u8_t;
////typedef unsigned short int u16_t;
////typedef unsigned int u32_t;
////typedef int i32_t;
////typedef short int i16_t;
////typedef signed char i8_t;
//
////typedef union
////{
////  i16_t i16bit[3];
////  u8_t u8bit[6];
////} Type3Axis16bit_U;
////
////typedef union
////{
////  i16_t i16bit;
////  u8_t u8bit[2];
////} Type1Axis16bit_U;
////
////typedef union
////{
////  i32_t i32bit;
////  u8_t u8bit[4];
////} Type1Axis32bit_U;
////
////typedef enum
////{
////  MEMS_SUCCESS        =   0x01,
////  MEMS_ERROR        =   0x00
////} status_t;
//
///**
// * @brief  Component's Status enumerator definition.
// */
////typedef enum {
////  COMPONENT_OK = 0,
////  COMPONENT_ERROR,
////  COMPONENT_TIMEOUT,
////  COMPONENT_NOT_IMPLEMENTED
////} snsr_state_t ;
//
///**
// * @brief  Sensor axes raw data structure definition
// */
////typedef struct
////{
////  int16_t AXIS_X;
////  int16_t AXIS_Y;
////  int16_t AXIS_Z;
////} SensorAxesRaw_t;
//
///**
// * @brief  Sensor axes data structure definition
// */
////typedef struct
////{
////  int32_t AXIS_X;
////  int32_t AXIS_Y;
////  int32_t AXIS_Z;
////} SensorAxes_t;
//
///**
// * @brief  Sensor output data rate enumerator definition
// */
////typedef enum
////{
////  ODR_LOW,
////  ODR_MID_LOW,
////  ODR_MID,
////  ODR_MID_HIGH,
////  ODR_HIGH
////} SensorOdr_t;
//
///**
// * @brief  Sensor full scale enumerator definition
// */
////typedef enum
////{
////  FS_LOW,
////  FS_MID_LOW,
////  FS_MID,
////  FS_MID_HIGH,
////  FS_HIGH
////} SensorFs_t;
//
///**
// * @brief  Sensor interrupt pin enumerator definition
// */
////typedef enum
////{
////  INT1_PIN,
////  INT2_PIN
////} SensorIntPin_t;
//
///** @addtogroup LSM6DS3US_ACC_SENSITIVITY Accelero sensitivity values based on selected full scale
// * @{
// */
//#define LSM6DS3US_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
//#define LSM6DS3US_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
//#define LSM6DS3US_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
//#define LSM6DS3US_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */
///**
// * @}
// */
//
///** @addtogroup LSM6DS3US_GYRO_SENSITIVITY Gyro sensitivity values based on selected full scale
// * @{
// */
//#define LSM6DS3US_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
//#define LSM6DS3US_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
//#define LSM6DS3US_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
//#define LSM6DS3US_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
//#define LSM6DS3US_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */
///**
// * @}
// */
//
///** @addtogroup LSM6DS3US_PEDOMETER_THRESHOLD Pedometer threshold values
// * @{
// */
//#define LSM6DS3US_PEDOMETER_THRESHOLD_LOW       0x00  /**< Lowest  value of pedometer threshold */
//#define LSM6DS3US_PEDOMETER_THRESHOLD_MID_LOW   0x07
//#define LSM6DS3US_PEDOMETER_THRESHOLD_MID       0x0F
//#define LSM6DS3US_PEDOMETER_THRESHOLD_MID_HIGH  0x17
//#define LSM6DS3US_PEDOMETER_THRESHOLD_HIGH      0x1F  /**< Highest value of pedometer threshold */
///**
// * @}
// */
//
///* @addtogroup LSM6DS3US_WAKE_UP_THRESHOLD Wake up threshold values */
//#define LSM6DS3US_WAKE_UP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
//#define LSM6DS3US_WAKE_UP_THRESHOLD_MID_LOW   0x0F
//#define LSM6DS3US_WAKE_UP_THRESHOLD_MID       0x1F
//#define LSM6DS3US_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
//#define LSM6DS3US_WAKE_UP_THRESHOLD_HIGH      0x3F  /**< Highest value of wake up threshold */
//
///* @addtogroup LSM6DS3US_TAP_THRESHOLD Tap threshold values */
//#define LSM6DS3US_TAP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
//#define LSM6DS3US_TAP_THRESHOLD_MID_LOW   0x08
//#define LSM6DS3US_TAP_THRESHOLD_MID       0x10
//#define LSM6DS3US_TAP_THRESHOLD_MID_HIGH  0x18
//#define LSM6DS3US_TAP_THRESHOLD_HIGH      0x1F  /**< Highest value of wake up threshold */
//
///* @addtogroup LSM6DS3US_TAP_SHOCK_TIME Tap shock time window values */
//#define LSM6DS3US_TAP_SHOCK_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
//#define LSM6DS3US_TAP_SHOCK_TIME_MID_LOW   0x01
//#define LSM6DS3US_TAP_SHOCK_TIME_MID_HIGH  0x02
//#define LSM6DS3US_TAP_SHOCK_TIME_HIGH      0x03  /**< Highest value of wake up threshold */
//
///* @addtogroup LSM6DS3US_TAP_QUIET_TIME Tap quiet time window values */
//#define LSM6DS3US_TAP_QUIET_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
//#define LSM6DS3US_TAP_QUIET_TIME_MID_LOW   0x01
//#define LSM6DS3US_TAP_QUIET_TIME_MID_HIGH  0x02
//#define LSM6DS3US_TAP_QUIET_TIME_HIGH      0x03  /**< Highest value of wake up threshold */
//
///* @addtogroup LSM6DS3US_TAP_DURATION_TIME Tap duration time window values */
//#define LSM6DS3US_TAP_DURATION_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
//#define LSM6DS3US_TAP_DURATION_TIME_MID_LOW   0x04
//#define LSM6DS3US_TAP_DURATION_TIME_MID       0x08
//#define LSM6DS3US_TAP_DURATION_TIME_MID_HIGH  0x0C
//#define LSM6DS3US_TAP_DURATION_TIME_HIGH      0x0F  /**< Highest value of wake up threshold */

// Accelerometer
typedef enum
{
  ACCELERO_SENSORS_AUTO = -1,    	/* Always first element and equal to -1 */
  LSM6DS3_X_0,                   	/* . */
  LSM303AGR_X_0                  	/* . */
} ACCELERO_ID_t;
#define ACCELERO_SENSORS_MAX_NUM 2

// Gyroscope
typedef enum
{
  GYRO_SENSORS_AUTO = -1,       	/* Always first element and equal to -1 */
  LSM6DS3_G_0,                  	/* Default on board. */
} GYRO_ID_t;
#define GYRO_SENSORS_MAX_NUM 1

// Magnetometer
typedef enum
{
  MAGNETO_SENSORS_AUTO = -1,     	/* Always first element and equal to -1 */
  LSM303AGR_M_0                 	/* Default on board. */
} MAGNETO_ID_t;
#define MAGNETO_SENSORS_MAX_NUM 1

// Pressure
//typedef enum {
//	PRESSURE_SENSORS_AUTO = -1,    	/* Always first element and equal to -1 */
//	LPS22HB_P_0,                    /* Default on board. */
//} PRESSURE_ID_t;
//#define PRESSURE_SENSORS_MAX_NUM 1

// Temperature
typedef enum {
	TEMPERATURE_SENSORS_AUTO = -1, 		/* Always first element and equal to -1 */
	LPS22HB_T_0,      	              	/* LPS22HB temperature on SensorTile. */
	HTS221_T_0,                     	/* HTS221 temperature on the motherboard. */
} TEMPERATURE_ID_t;
#define TEMPERATURE_SENSORS_MAX_NUM 2

// Humidity
typedef enum {
	HUMIDITY_SENSORS_AUTO = -1,    		/* Always first element and equal to -1 */
	HTS221_H_0                     		/* Default on board. */
} HUMIDITY_ID_t;
#define HUMIDITY_SENSORS_MAX_NUM 1

typedef enum {
	//  TEMPERATURE_SENSORS_AUTO = -1, 	/* Always first element and equal to -1 */
	LSM6DS3 = 0,
	LSM303AGR_X,
	LSM303AGR_M,
	LPS22HB,
	HTS221
} I2C_Device_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mems_i2c_master_init( void );
esp_err_t mems_i2c_master_deinit( void );
esp_err_t mems_i2c_read_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, uint16_t size);
esp_err_t mems_i2c_write_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, uint16_t size);

// MEMS DRIVER FUNCTIONS ADAPTER TODO: IMPLEMENT ONLY THIS
//////////////////////////////////////////////////////////
esp_err_t MEMS_I2C_Init(void);
esp_err_t MEMS_I2C_DeInit(void);
esp_err_t MEMS_I2C_ReadReg(uint16_t i2c_addr, uint16_t i2c_reg, uint8_t* data_rd, uint16_t size);
esp_err_t MEMS_I2C_WriteReg(uint16_t i2c_addr, uint16_t i2c_reg, uint8_t* data_wr, uint16_t size);
uint32_t MEMS_GetTick(void);
//////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif

#endif /* MAIN_TEST_I2C_SNSR_H_ */
