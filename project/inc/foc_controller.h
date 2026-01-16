/**
 * @file    foc_controller.h
 * @author  FOC控制器接口
 * @date    2026-01-09
 * @brief   用于集成SimpleFOC到AT32F423工程的接口
 */

#ifndef __FOC_CONTROLLER_H
#define __FOC_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "simplefoc_c.h"
#include "at32f423.h"
#include "hal_interface.h"

/* 宏定义 */
#define MAX_MOTORS                3                   /*!< 最大电机数量 */

/* 电机控制结构体 */
typedef struct {
    foc_motor_t motor;                  /*!< 电机对象 */
    float target_velocity;              /*!< 目标速度 */
    float target_position;              /*!< 目标位置 */
    float current_position;             /*!< 当前位置 */
    float current_velocity;             /*!< 当前速度 */
    bool enabled;                       /*!< 使能状态 */
    bool overcurrent;                   /*!< 过流状态 */
    uint8_t motor_index;                /*!< 电机索引 */
} motor_control_t;

/* 全局变量声明 */
extern motor_control_t g_motors[MAX_MOTORS];

/**
 * @brief FOC控制器初始化
 * @return 0表示成功，负数表示错误
 */
int foc_controller_init(void);

/**
 * @brief 设置指定电机的FOC控制模式
 * @param motor_index 电机索引 (0-2)
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int foc_controller_set_mode(uint8_t motor_index, control_mode_e mode);

/**
 * @brief 设置指定电机的目标值
 * @param motor_index 电机索引 (0-2)
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int foc_controller_set_target(uint8_t motor_index, float target);

/**
 * @brief 获取指定电机的当前位置
 * @param motor_index 电机索引 (0-2)
 * @return 当前位置
 */
float foc_controller_get_position(uint8_t motor_index);

/**
 * @brief 获取指定电机的当前速度
 * @param motor_index 电机索引 (0-2)
 * @return 当前速度
 */
float foc_controller_get_velocity(uint8_t motor_index);

/**
 * @brief 主控制循环
 * @return 0表示成功，负数表示错误
 */
int foc_controller_loop(void);

/**
 * @brief 启动指定电机
 * @param motor_index 电机索引 (0-2)
 * @return 0表示成功，负数表示错误
 */
int foc_controller_start(uint8_t motor_index);

/**
 * @brief 停止指定电机
 * @param motor_index 电机索引 (0-2)
 * @return 0表示成功，负数表示错误
 */
int foc_controller_stop(uint8_t motor_index);

/**
 * @brief 更新指定电机的霍尔传感器读取
 * @param motor_index 电机索引 (0-2)
 */
void update_hall_sensors(uint8_t motor_index);

/**
 * @brief 更新指定电机的编码器读取
 * @param motor_index 电机索引 (0-2)
 */
void update_encoder(uint8_t motor_index);

/**
 * @brief 更新指定电机的电流检测
 * @param motor_index 电机索引 (0-2)
 */
void update_current_sense(uint8_t motor_index);

/**
 * @brief 检查指定电机的DRV8311是否过流
 * @param motor_index 电机索引 (0-2)
 * @return true表示过流，false表示正常
 */
bool drv8311_check_overcurrent(uint8_t motor_index);

/**
 * @brief 检查指定电机的DRV8311是否过温
 * @param motor_index 电机索引 (0-2)
 * @return true表示过温，false表示正常
 */
bool drv8311_check_overtemperature(uint8_t motor_index);

/**
 * @brief 使能/禁用指定电机的DRV8311
 * @param motor_index 电机索引 (0-2)
 * @param enable true为使能，false为禁用
 */
void drv8311_set_enable(uint8_t motor_index, bool enable);

/**
 * @brief 初始化指定电机
 * @param motor_index 电机索引 (0-2)
 * @return 0表示成功，负数表示错误
 */
int init_motor(uint8_t motor_index);

/**
 * @brief 控制指定电机
 * @param motor_index 电机索引 (0-2)
 */
void control_motor(uint8_t motor_index);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_CONTROLLER_H */