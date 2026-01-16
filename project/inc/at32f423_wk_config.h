/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f423_wk_config.h
  * @brief    header file of work bench config
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* define to prevent recursive inclusion -----------------------------------*/
#ifndef __AT32F423_WK_CONFIG_H
#define __AT32F423_WK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------------*/
#include "stdio.h"
#include "at32f423.h"

/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* DRV8311驱动器配置 */
#define DRV8311_CURRENT_GAIN        2.0f        /*!< 电流检测增益 2V/A */
#define DRV8311_PWM_MODE            3XPWM       /*!< 3xPWM模式 */
#define DRV8311_SUPPLY_VOLTAGE      12.0f       /*!< 电源电压 */
#define DRV8311_MAX_CURRENT         5.0f        /*!< 最大电流限制(A) */

/* 电机参数配置 */
#define MOTOR_POLE_PAIRS            7           /*!< 电机极对数 */
#define MOTOR_PHASE_RESISTANCE      5.0f        /*!< 相电阻(欧姆) */
#define MOTOR_KV_RATING             1000.0f     /*!< KV值 */

/* FOC控制参数 */
#define FOC_CURRENT_LIM             2.0f        /*!< 电流限制 */
#define FOC_VOLTAGE_LIM             11.0f       /*!< 电压限制 */
#define FOC_LOW_PASS_FILTER_TF      0.01f       /*!< 低通滤波器时间常数 */

/* add user code end exported macro */

/* add user code begin dma define */
/* user can only modify the dma define value */
#define DMA1_CHANNEL1_BUFFER_SIZE   3
#define DMA1_CHANNEL1_MEMORY_BASE_ADDR   ((uint32_t)adc_dma_buffer)
//#define DMA1_CHANNEL1_PERIPHERAL_BASE_ADDR  0

//#define DMA1_CHANNEL2_BUFFER_SIZE   0
//#define DMA1_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL3_BUFFER_SIZE   0
//#define DMA1_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL4_BUFFER_SIZE   0
//#define DMA1_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL5_BUFFER_SIZE   0
//#define DMA1_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL6_BUFFER_SIZE   0
//#define DMA1_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL7_BUFFER_SIZE   0
//#define DMA1_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL7_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL1_BUFFER_SIZE   0
//#define DMA2_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL1_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL2_BUFFER_SIZE   0
//#define DMA2_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL3_BUFFER_SIZE   0
//#define DMA2_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL4_BUFFER_SIZE   0
//#define DMA2_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL5_BUFFER_SIZE   0
//#define DMA2_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL6_BUFFER_SIZE   0
//#define DMA2_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL7_BUFFER_SIZE   0
//#define DMA2_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL7_PERIPHERAL_BASE_ADDR   0
/* add user code end dma define */

/* Private defines -------------------------------------------------------------*/
#define EA_PIN    GPIO_PINS_0
#define EA_GPIO_PORT    GPIOA
#define EB_PIN    GPIO_PINS_1
#define EB_GPIO_PORT    GPIOA
#define EI_PIN    GPIO_PINS_2
#define EI_GPIO_PORT    GPIOA
#define I_A_PIN    GPIO_PINS_3
#define I_A_GPIO_PORT    GPIOA
#define I_B_PIN    GPIO_PINS_4
#define I_B_GPIO_PORT    GPIOA
#define I_C_PIN    GPIO_PINS_5
#define I_C_GPIO_PORT    GPIOA
#define HA_PIN    GPIO_PINS_6
#define HA_GPIO_PORT    GPIOA
#define HB_PIN    GPIO_PINS_7
#define HB_GPIO_PORT    GPIOA
#define HC_PIN    GPIO_PINS_0
#define HC_GPIO_PORT    GPIOB
#define DRV8311_EN_PIN    GPIO_PINS_1
#define DRV8311_EN_GPIO_PORT    GPIOB
#define PWM_C_PIN    GPIO_PINS_8
#define PWM_C_GPIO_PORT    GPIOA
#define PWM_B_PIN    GPIO_PINS_9
#define PWM_B_GPIO_PORT    GPIOA
#define PWM_A_PIN    GPIO_PINS_10
#define PWM_A_GPIO_PORT    GPIOA
#define RUN_LED_PIN    GPIO_PINS_7
#define RUN_LED_GPIO_PORT    GPIOB

/* exported functions ------------------------------------------------------- */
  /* system clock config. */
  void wk_system_clock_config(void);

  /* config periph clock. */
  void wk_periph_clock_config(void);

  /* nvic config. */
  void wk_nvic_config(void);

/* add user code begin exported functions */

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
