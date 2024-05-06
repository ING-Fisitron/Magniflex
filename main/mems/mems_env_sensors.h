/**
 ******************************************************************************
 * @file    iks01a2_env_sensors.h
 * @author  MEMS Software Solutions Team
 * @brief   This file provides a set of functions needed to manage the environmental sensors
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MEMS_ENV_SENSORS_H
#define MEMS_ENV_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../mems/mems_conf.h"
#include "../mems/env_sensor.h"

#ifndef USE_MEMS_ENV_SENSOR_HTS221_0
#define USE_MEMS_ENV_SENSOR_HTS221_0          0
#endif

#ifndef USE_MEMS_ENV_SENSOR_LPS22HB_0
#define USE_MEMS_ENV_SENSOR_LPS22HB_0         0
#endif

#ifndef USE_MEMS_ENV_SENSOR_LPS33HW_0
#define USE_MEMS_ENV_SENSOR_LPS33HW_0         0
#endif

#ifndef USE_MEMS_ENV_SENSOR_LPS22HH_0
#define USE_MEMS_ENV_SENSOR_LPS22HH_0         0
#endif

#ifndef USE_MEMS_ENV_SENSOR_STTS22H_0
#define USE_MEMS_ENV_SENSOR_STTS22H_0         0
#endif

#if (USE_MEMS_ENV_SENSOR_LPS22HB_0 == 1)
#include "lps22hb.h"
#endif

#if (USE_MEMS_ENV_SENSOR_HTS221_0 == 1)
#include "hts221.h"
#endif

#if (USE_MEMS_ENV_SENSOR_LPS33HW_0 == 1)
#include "lps33hw.h"
#endif

#if (USE_MEMS_ENV_SENSOR_LPS22HH_0 == 1)
#include "lps22hh.h"
#endif

#if (USE_MEMS_ENV_SENSOR_STTS22H_0 == 1)
#include "stts22h.h"
#endif

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup MEMS MEMS
 * @{
 */

/** @addtogroup MEMS_ENV_SENSORS MEMS ENV SENSORS
 * @{
 */

/** @defgroup MEMS_ENV_SENSORS_Exported_Types MEMS ENV SENSORS Exported Types
 * @{
 */

/* Environmental Sensor instance Info */
typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} MEMS_ENV_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} MEMS_ENV_SENSOR_Ctx_t;

/**
 * @}
 */

/** @defgroup MEMS_ENV_SENSOR_Exported_Constants MEMS ENV SENSOR Exported Constants
 * @{
 */

#if (USE_MEMS_ENV_SENSOR_HTS221_0 == 1)
#define MEMS_HTS221_0 0
#endif

#if (USE_MEMS_ENV_SENSOR_LPS22HB_0 == 1)
#define MEMS_LPS22HB_0 (USE_MEMS_ENV_SENSOR_HTS221_0)
#endif

#if (USE_MEMS_ENV_SENSOR_LPS33HW_0 == 1)
#define MEMS_LPS33HW_0 (USE_MEMS_ENV_SENSOR_HTS221_0 + \
                           USE_MEMS_ENV_SENSOR_LPS22HB_0)
#endif

#if (USE_MEMS_ENV_SENSOR_LPS22HH_0 == 1)
#define MEMS_LPS22HH_0 (USE_MEMS_ENV_SENSOR_HTS221_0 + \
                           USE_MEMS_ENV_SENSOR_LPS22HB_0 + \
                           USE_MEMS_ENV_SENSOR_LPS33HW_0)
#endif

#if (USE_MEMS_ENV_SENSOR_STTS22H_0 == 1)
#define MEMS_STTS22H_0 (USE_MEMS_ENV_SENSOR_HTS221_0 + \
                           USE_MEMS_ENV_SENSOR_LPS22HB_0 + \
                           USE_MEMS_ENV_SENSOR_LPS33HW_0 + \
                           USE_MEMS_ENV_SENSOR_LPS22HH_0)
#endif

#ifndef ENV_TEMPERATURE
#define ENV_TEMPERATURE      1U
#endif
#ifndef ENV_PRESSURE
#define ENV_PRESSURE         2U
#endif
#ifndef ENV_HUMIDITY
#define ENV_HUMIDITY         4U
#endif

#define MEMS_ENV_FUNCTIONS_NBR    3U
#define MEMS_ENV_INSTANCES_NBR    (USE_MEMS_ENV_SENSOR_HTS221_0 + \
                                      USE_MEMS_ENV_SENSOR_LPS22HB_0 + \
                                      USE_MEMS_ENV_SENSOR_LPS33HW_0 + \
                                      USE_MEMS_ENV_SENSOR_LPS22HH_0 + \
                                      USE_MEMS_ENV_SENSOR_STTS22H_0)

#if (MEMS_ENV_INSTANCES_NBR == 0)
#error "No environmental sensor instance has been selected"
#endif

/**
 * @}
 */

/** @addtogroup MEMS_ENV_SENSORS_Exported_Functions MEMS ENV SENSOR Exported Functions
 * @{
 */

int32_t MEMS_ENV_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t MEMS_ENV_SENSOR_DeInit(uint32_t Instance);
int32_t MEMS_ENV_SENSOR_GetCapabilities(uint32_t Instance, MEMS_ENV_SENSOR_Capabilities_t *Capabilities);
int32_t MEMS_ENV_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t MEMS_ENV_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t MEMS_ENV_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t MEMS_ENV_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t MEMS_ENV_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t MEMS_ENV_SENSOR_GetValue(uint32_t Instance, uint32_t Function, float *Value);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MEMS_ENV_SENSORS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
