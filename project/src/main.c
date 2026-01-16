/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

/* Includes ------------------------------------------------------------------*/
#include "at32f423_wk_config.h"
#include "wk_adc.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "foc_controller.h"
#include "debug_serial.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
uint16_t adc_dma_buffer[3];  /*!< ADC DMA缓冲区，用于存储电流检测值 */
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */


/**
  * @brief  take some delay for waiting power stable, delay is about 60ms with frequency 8MHz.
  * @param  none
  * @retval none
  */
static void wk_wait_for_power_stable(void)
{
  volatile uint32_t delay = 0;
  for(delay = 0; delay < 50000; delay++);
}

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* add a necessary delay to ensure that Vdd is higher than the operating
     voltage of battery powered domain (2.57V) when the battery powered 
     domain is powered on for the first time and being operated. */
  wk_wait_for_power_stable();
  
  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* nvic config. */
  wk_nvic_config();

  /* timebase config. */
  wk_timebase_init();

  /* init gpio function. */
  wk_gpio_config();

  /* init adc1 function. */
  wk_adc1_init();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&ADC1->odt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init usart1 function. */
  wk_usart1_init();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init tmr2 function. */
  wk_tmr2_init();

  /* init tmr3 function. */
  wk_tmr3_init();

  /* add user code begin 2 */
  
  /* 初始化串口调试功能 */
  debug_serial_init();
  
  /* 初始化FOC控制器 */
  if (foc_controller_init() != 0) {
      // 初始化失败处理
      while(1);
  }
  
  /* 初始化多个电机 (最多3个) */
  for(uint8_t motor_idx = 0; motor_idx < MAX_MOTORS; motor_idx++) {
      /* 设置控制模式为速度控制 */
      foc_controller_set_mode(motor_idx, FOC_CONTROL_VELOCITY);
      
      /* 设置目标速度 */
      switch(motor_idx) {
          case 0:
              foc_controller_set_target(motor_idx, 2.0f); // 电机0目标速度 2 rad/s
              break;
          case 1:
              foc_controller_set_target(motor_idx, 1.5f); // 电机1目标速度 1.5 rad/s
              break;
          case 2:
              foc_controller_set_target(motor_idx, 1.0f); // 电机2目标速度 1.0 rad/s
              break;
          default:
              break;
      }
      
      /* 启动电机 */
      foc_controller_start(motor_idx);
  }

  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    
    /* 更新所有电机的传感器数据 */
    for(uint8_t motor_idx = 0; motor_idx < MAX_MOTORS; motor_idx++) {
        update_hall_sensors(motor_idx);
        update_encoder(motor_idx);
        update_current_sense(motor_idx);
    }
    
    /* 对每个电机进行DRV8311安全检查 */
    for(uint8_t motor_idx = 0; motor_idx < MAX_MOTORS; motor_idx++) {
        if (drv8311_check_overcurrent(motor_idx)) {
            // 过流保护：停止电机并禁用DRV8311
            foc_controller_stop(motor_idx);
            drv8311_set_enable(motor_idx, false);
            // 可以在此处添加错误处理或报警
        } else if (drv8311_check_overtemperature(motor_idx)) {
            // 过温保护：停止电机并禁用DRV8311
            foc_controller_stop(motor_idx);
            drv8311_set_enable(motor_idx, false);
            // 可以在此处添加错误处理或报警
        }
    }
    
    /* 执行FOC控制循环 - 会在内部处理所有启用的电机 */
    foc_controller_loop();
    
    /* 处理串口命令 */
    handle_received_frames();
    
    /* 发送电机状态数据（每100ms发送一次） */
    static uint32_t last_send_time = 0;
    if (wk_get_tick() - last_send_time > 100) {
        for(uint8_t motor_idx = 0; motor_idx < MAX_MOTORS; motor_idx++) {
            motor_status_data_t status;
            status.angle = g_motors[motor_idx].motor.estimator.angle;
            status.velocity = g_motors[motor_idx].motor.estimator.velocity;
            status.current_q = g_motors[motor_idx].motor.Iq;
            status.current_d = g_motors[motor_idx].motor.Id;
            status.voltage_q = g_motors[motor_idx].motor.Uq;
            status.voltage_d = g_motors[motor_idx].motor.Ud;
            status.status = g_motors[motor_idx].motor.status;
            send_motor_status(motor_idx, &status);
        }
        last_send_time = wk_get_tick();
    }
    
    /* 延时，模拟控制周期 */
    for(volatile uint32_t i = 0; i < 1000; i++);

    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
