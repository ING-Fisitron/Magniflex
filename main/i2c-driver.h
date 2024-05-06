/*
 * x-i2c-driver.h
 *
 *  Created on: Feb 17, 2020
 *      Author: X-Phase s.r.l., Leonardo Volpi
 */

#ifndef MAIN_X_I2C_DRIVER_H_
#define MAIN_X_I2C_DRIVER_H_


#include "driver/i2c.h"

#define I2C_SCL_IO 16				/*!< GPIO number for I2C master clock */
#define I2C_SDA_IO 4				/*!< GPIO number for I2C master data  */
#define I2C_FREQ_HZ 100000 			/*!< I2C master clock frequency */
#define I2C_PORT_NUM I2C_NUM_1 		/*!< I2C port number for master DEV */
#define I2C_TX_BUF_DISABLE 0   		/*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE 0 		/*!< I2C master do not need buffer */

#define DATA_LENGTH 512                  	/*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               	/*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 	/*!< delay time between different test items */

#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)		/*!< I2C slave TX buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)		/*!< I2C slave RX buffer size */

#define WRITE_BIT I2C_MASTER_WRITE              	/*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                	/*!< I2C master read */
#define ACK_CHECK_EN 0x1                        	/*!< I2C master will check ACK from slave*/
#define ACK_CHECK_DIS 0x0                       	/*!< I2C master will not check ACK from slave */
#define ACK_VAL 0x0                             	/*!< I2C ACK value */
#define NACK_VAL 0x1                            	/*!< I2C NACK value */


esp_err_t i2c_master_init( void );
esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size);
esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size);

#endif /* MAIN_X_I2C_DRIVER_H_ */
