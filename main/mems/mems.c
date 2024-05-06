/*
 * test_i2c_snsr.c
 *
 *  Created on: Feb 21, 2019
 *      Author: leo
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "mems.h"

/* ************* Generic Functions ****************** */

/**
 * read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 */
esp_err_t mems_i2c_read_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, uint16_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
    	i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * write i2c slave device with registered interface
 * 	Master device write data to slave(both esp32),
 * 	the data will be stored in slave buffer.
 * 	We can read them out from slave buffer.
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------|----------------------|------|
 */
esp_err_t mems_i2c_write_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, uint16_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
//esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t *data_wr, uint16_t size) {
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, (i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
//    i2c_master_stop(cmd);
//    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}

//uint8_t Sensor_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite) {
//	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
//	return i2c_master_write_slave_reg( I2C_PORT_NUM, ctx->address, WriteAddr, pBuffer, nBytesToWrite);
//}
//
//uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead ) {
//	DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
//	return i2c_master_read_slave_reg( I2C_PORT_NUM ,ctx->address, ReadAddr, pBuffer, nBytesToRead );
//}


/* ************* Exposed Functions  ****************** */

/* i2c master interface initialization functions */
esp_err_t i2c_init_flag = 0;

esp_err_t mems_i2c_master_init( void ) {
	esp_err_t ret = ESP_OK;
	if (i2c_init_flag == 1) {
		return ret;
	}
	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = MEMS_I2C_SDA,
#ifdef HW_1_3
			.sda_pullup_en = GPIO_PULLUP_DISABLE, // HW_1_3
			.scl_pullup_en = GPIO_PULLUP_DISABLE, // HW_1_3
#else
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
#endif
			.scl_io_num = MEMS_I2C_SCL,
			.master.clk_speed = MEMS_I2C_FREQ_HZ,
	};
    i2c_param_config( MEMS_I2C_PORT_NUM, &conf );
    ret = i2c_driver_install( MEMS_I2C_PORT_NUM, conf.mode, MEMSE_I2C_RX_BUF_DISABLE, MEMS_I2C_TX_BUF_DISABLE, 0 );
    if ( ret == ESP_OK ) {
    	i2c_init_flag = 1;
    }
    return ret;
}

// TODO: implement!.
esp_err_t mems_i2c_master_deinit( void ) {
	esp_err_t ret = ESP_OK;
    return ret;
}

// MEMS DRIVER FUNCTIONS ADAPTER TODO: IMPLEMENT ONLY THIS
//////////////////////////////////////////////////////////
esp_err_t MEMS_I2C_Init(void) {
	return mems_i2c_master_init();
}
esp_err_t MEMS_I2C_DeInit(void) {
	return mems_i2c_master_deinit();
}
esp_err_t MEMS_I2C_ReadReg(uint16_t i2c_addr, uint16_t i2c_reg, uint8_t* data_rd, uint16_t size) {
	return mems_i2c_read_reg(MEMS_I2C_PORT_NUM, i2c_addr, i2c_reg, data_rd, size);
}
esp_err_t MEMS_I2C_WriteReg(uint16_t i2c_addr, uint16_t i2c_reg, uint8_t* data_wr, uint16_t size) {
	return mems_i2c_write_reg(MEMS_I2C_PORT_NUM, i2c_addr, i2c_reg, data_wr, size);
}
uint32_t MEMS_GetTick(void) {
	return (uint32_t) esp_timer_get_time();
}
//////////////////////////////////////////////////////////

