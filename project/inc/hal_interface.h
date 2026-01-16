/**
 * @file    hal_interface.h
 * @author  硬件抽象层接口
 * @date    2026-01-09
 * @brief   硬件抽象层接口定义，用于解耦单片机底层实现
 */

#ifndef __HAL_INTERFACE_H
#define __HAL_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f423.h"
#include "at32f423_wk_config.h"


/* GPIO接口宏定义 */
#define HAL_GPIO_SET(PORT, PIN)                 gpio_bits_set(PORT, PIN)
#define HAL_GPIO_RESET(PORT, PIN)               gpio_bits_reset(PORT, PIN)
#define HAL_GPIO_READ(PORT, PIN)                gpio_input_data_bit_read(PORT, PIN)

/* 定时器接口宏定义 */
#define HAL_TMR_GET_CNT(TMR)                    tmr_counter_value_get(TMR)
#define HAL_TMR_SET_PERIOD(TMR, VAL)            tmr_period_value_set(TMR, VAL)
#define HAL_TMR_GET_PERIOD(TMR)                 tmr_period_value_get(TMR)
#define HAL_TMR_SET_CMP(TMR, CHANNEL, VAL)      tmr_channel_value_set(TMR, CHANNEL, VAL)
#define HAL_TMR_ENABLE_COUNTER(TMR)             tmr_counter_enable(TMR, TRUE)
#define HAL_TMR_DISABLE_COUNTER(TMR)            tmr_counter_enable(TMR, FALSE)
#define HAL_TMR_ENABLE_OUTPUT(TMR)              tmr_output_enable(TMR, TRUE)
#define HAL_TMR_DISABLE_OUTPUT(TMR)             tmr_output_enable(TMR, FALSE)

/* ADC接口宏定义 */
#define HAL_ADC_GET_REG(ADCx)                   ADCx->odt

/* 数学运算宏定义 */
#ifndef HAL_MATH_DEFINED
#define HAL_MATH_DEFINED
#define HAL_SIN(x)                              sinf(x)
#define HAL_COS(x)                              cosf(x)
#define HAL_SQRT(x)                             sqrtf(x)
#define HAL_FMAX(a, b)                          fmaxf(a, b)
#define HAL_FMIN(a, b)                          fminf(a, b)
#define HAL_FABS(x)                             fabsf(x)
#endif

/* 系统延时宏定义 */
#define HAL_DELAY_US(us)                        do { \
                                                    volatile uint32_t nCount = (SystemCoreClock / 1000000) * us; \
                                                    for(; nCount != 0; nCount--); \
                                                } while(0)

/* 临界区保护宏定义 */
#define HAL_ENTER_CRITICAL()                    __disable_irq()
#define HAL_EXIT_CRITICAL()                     __enable_irq()

/* 内存操作宏定义 */
#define HAL_MEMSET(ptr, val, size)              memset(ptr, val, size)
#define HAL_MEMCPY(dst, src, size)              memcpy(dst, src, size)

/* 类型定义 */
typedef gpio_type* hal_gpio_port_t;
typedef uint16_t hal_gpio_pin_t;
typedef tmr_type* hal_timer_t;
typedef adc_type* hal_adc_t;

/* 特定外设实例定义 */
#define HAL_PWM_TIMER                           TMR1
#define HAL_ENCODER_TIMER                       TMR2
#define HAL_ADC_INSTANCE                        ADC1

/* PWM通道定义 */
#define HAL_PWM_CHANNEL_U                       TMR_SELECT_CHANNEL_1
#define HAL_PWM_CHANNEL_V                       TMR_SELECT_CHANNEL_2
#define HAL_PWM_CHANNEL_W                       TMR_SELECT_CHANNEL_3

/* GPIO定义 */
#define HAL_DRV8311_ENABLE_PORT                 DRV8311_ENABLE_PORT
#define HAL_DRV8311_ENABLE_PIN                  DRV8311_ENABLE_PIN
#define HAL_HALL_U_PORT                         HA_GPIO_PORT
#define HAL_HALL_V_PORT                         HB_GPIO_PORT
#define HAL_HALL_W_PORT                         HC_GPIO_PORT
#define HAL_HALL_U_PIN                          HA_PIN
#define HAL_HALL_V_PIN                          HB_PIN
#define HAL_HALL_W_PIN                          HC_PIN
#define HAL_CURRENT_U_PIN                       I_A_PIN
#define HAL_CURRENT_V_PIN                       I_B_PIN
#define HAL_CURRENT_W_PIN                       I_C_PIN
#define HAL_CURRENT_U_PORT                      I_A_GPIO_PORT
#define HAL_CURRENT_V_PORT                      I_B_GPIO_PORT
#define HAL_CURRENT_W_PORT                      I_C_GPIO_PORT

#ifdef __cplusplus
}
#endif

#endif /* __HAL_INTERFACE_H */