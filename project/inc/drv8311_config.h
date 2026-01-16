/**
 * @file    drv8311_config.h
 * @author  DRV8311驱动器配置
 * @date    2026-01-09
 * @brief   DRV8311电机驱动器专用配置
 */

#ifndef __DRV8311_CONFIG_H
#define __DRV8311_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f423_wk_config.h"

/* DRV8311专用配置 */
#define DRV8311_ENABLE_PIN          DRV8311_EN_PIN          /*!< EN引脚定义 */
#define DRV8311_ENABLE_PORT         DRV8311_EN_GPIO_PORT    /*!< EN端口定义 */

/* 电流检测配置 */
#define DRV8311_CS_GAIN             2.0f            /*!< 电流检测放大器增益 (V/A) */
#define DRV8311_CS_ZERO_OFFSET      2048.0f         /*!< 电流检测零点偏移 (ADC值) */
#define DRV8311_CS_MAX_CURRENT      10.0f           /*!< 最大可测电流 (A) */

/* PWM配置 */
#define DRV8311_PWM_FREQ            20000.0f        /*!< PWM频率 (Hz) */
#define DRV8311_DEAD_TIME_NS        1000.0f         /*!< 死区时间 (ns) */

/* 保护配置 */
#define DRV8311_OVERCURRENT_LIM     5.0f            /*!< 过流保护阈值 (A) */
#define DRV8311_TEMPERATURE_LIM     150.0f          /*!< 过温保护阈值 (°C) */

/* 故障检测配置 */
#define DRV8311_FAULT_PIN           0               /*!< 故障检测引脚 (如适用) */
#define DRV8311_FAULT_PORT          0               /*!< 故障检测端口 (如适用) */

/* FOC控制优化参数 */
#define DRV8311_FOC_VOLTAGE_LIM     (DRV8311_SUPPLY_VOLTAGE * 0.9f)  /*!< FOC电压限制 (90%母线电压) */
#define DRV8311_FOC_CURRENT_LIM     (DRV8311_MAX_CURRENT * 0.8f)     /*!< FOC电流限制 (80%最大电流) */

/* DRV8311操作模式 */
#define DRV8311_MODE_3XPWM          1               /*!< 3xPWM模式 */
#define DRV8311_MODE_6XPWM          0               /*!< 6xPWM模式 (不推荐用于FOC) */

/* DRV8311特定功能启用 */
#define DRV8311_ENABLE_CURRENT_SENSE    1           /*!< 启用电流检测 */
#define DRV8311_ENABLE_FAULT_DETECT   0             /*!< 启用故障检测 (根据硬件配置) */

#ifdef __cplusplus
}
#endif

#endif /* __DRV8311_CONFIG_H */