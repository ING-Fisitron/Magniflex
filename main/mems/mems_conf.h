/**
 ******************************************************************************
 * @file    iks01a2_conf_template.h
 * @author  MEMS Application Team
 * @brief   IKS01A2 configuration template file.
 *          This file should be copied to the application folder and renamed
 *          to iks01a2_conf.h.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Replace the header file names with the ones of the target platform */
#include "../mems/mems_errno.h"
#include "mems.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMS_CONF_H__
#define __MEMS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

// ENVIRONMENT SENSORS
#define USE_MEMS_ENV_SENSOR_HTS221_0                1U
//#define USE_MEMS_ENV_SENSOR_LPS22HB_0               1U

// MOTION SENSORS
//#define USE_MEMS_MOTION_SENSOR_LSM6DSL_0            1U
//#define USE_MEMS_MOTION_SENSOR_LIS2MDL_0            1U

//#define IKS01A2_I2C_Init BSP_I2C1_Init
//#define IKS01A2_I2C_DeInit BSP_I2C1_DeInit
//#define IKS01A2_I2C_ReadReg BSP_I2C1_ReadReg
//#define IKS01A2_I2C_WriteReg BSP_I2C1_WriteReg
//#define IKS01A2_GetTick BSP_GetTick

#ifdef __cplusplus
}
#endif

#endif /* __MEMS_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

