/**
 * @file    multi_foc_controller.h
 * @author  多路FOC控制器
 * @date    2026-01-09
 * @brief   支持多路电机的FOC控制
 */

#ifndef __MULTI_FOC_CONTROLLER_H
#define __MULTI_FOC_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "simplefoc_c.h"
#include "hal_interface.h"

/* 电机数量定义 */
#define MAX_MOTORS              3               /*!< 最大电机数量 */

/* 电机ID定义 */
#define MOTOR_ID_1              0               /*!< 第1号电机 */
#define MOTOR_ID_2              1               /*!< 第2号电机 */
#define MOTOR_ID_3              2               /*!< 第3号电机 */

/* 电机控制模式 */
typedef enum {
    MULTI_MOTOR_MODE_INDIVIDUAL = 0,    /*!< 各电机独立控制 */
    MULTI_MOTOR_MODE_SYNCHRONIZED,      /*!< 多电机同步控制 */
    MULTI_MOTOR_MODE_FOLLOWER           /*!< 主从跟随控制 */
} multi_motor_mode_t;

/* 与simplefoc_c.h中的定义兼容的宏定义 */
#ifndef MOTOR_MODE_INDIVIDUAL
#define MOTOR_MODE_INDIVIDUAL           MULTI_MOTOR_MODE_INDIVIDUAL
#endif

/**
 * @brief 多电机控制器结构体
 */
typedef struct {
    foc_motor_t motors[MAX_MOTORS];     /*!< 电机数组 */
    multi_motor_mode_t mode;            /*!< 控制模式 */
    uint8_t active_motors;              /*!< 激活的电机数量 */
    float master_target;                /*!< 主电机目标值（用于同步/跟随模式） */
} multi_foc_controller_t;

/**
 * @brief 初始化多路FOC控制器
 * @param controller 多路控制器结构体指针
 * @param num_motors 电机数量 (1-3)
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int multi_foc_controller_init(multi_foc_controller_t *controller, uint8_t num_motors, multi_motor_mode_t mode);

/**
 * @brief 设置指定电机的控制模式
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_control_mode(multi_foc_controller_t *controller, uint8_t motor_id, control_mode_e mode);

/**
 * @brief 设置指定电机的目标值
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_target(multi_foc_controller_t *controller, uint8_t motor_id, float target);

/**
 * @brief 设置所有电机的统一目标值（同步模式下使用）
 * @param controller 多路控制器结构体指针
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_all_targets(multi_foc_controller_t *controller, float target);

/**
 * @brief 启动指定电机
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 0表示成功，负数表示错误
 */
int multi_foc_start_motor(multi_foc_controller_t *controller, uint8_t motor_id);

/**
 * @brief 停止指定电机
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 0表示成功，负数表示错误
 */
int multi_foc_stop_motor(multi_foc_controller_t *controller, uint8_t motor_id);

/**
 * @brief 启动所有电机
 * @param controller 多路控制器结构体指针
 * @return 0表示成功，负数表示错误
 */
int multi_foc_start_all(multi_foc_controller_t *controller);

/**
 * @brief 停止所有电机
 * @param controller 多路控制器结构体指针
 * @return 0表示成功，负数表示错误
 */
int multi_foc_stop_all(multi_foc_controller_t *controller);

/**
 * @brief 多电机控制主循环
 * @param controller 多路控制器结构体指针
 * @param sensor_angles 传感器角度数组
 * @param dt 时间步长
 * @return 0表示成功，负数表示错误
 */
int multi_foc_control_loop(multi_foc_controller_t *controller, float *sensor_angles, float dt);

/**
 * @brief 获取指定电机的位置
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 位置值
 */
float multi_foc_get_position(multi_foc_controller_t *controller, uint8_t motor_id);

/**
 * @brief 获取指定电机的速度
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 速度值
 */
float multi_foc_get_velocity(multi_foc_controller_t *controller, uint8_t motor_id);

/**
 * @brief 设置指定电机的PID参数
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @param pid_type PID类型 (0=angle, 1=velocity, 2=current_q, 3=current_d)
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_pid_constants(multi_foc_controller_t *controller, uint8_t motor_id, 
                               uint8_t pid_type, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* __MULTI_FOC_CONTROLLER_H */