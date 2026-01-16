/**
 * @file    multi_foc_controller.c
 * @author  多路FOC控制器实现
 * @date    2026-01-09
 * @brief   支持多路电机的FOC控制实现
 */

#include "multi_foc_controller.h"
#include "drv8311_config.h"
#include <string.h>

/**
 * @brief 初始化多路FOC控制器
 * @param controller 多路控制器结构体指针
 * @param num_motors 电机数量 (1-3)
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int multi_foc_controller_init(multi_foc_controller_t *controller, uint8_t num_motors, multi_motor_mode_t mode)
{
    if (controller == NULL || num_motors == 0 || num_motors > MAX_MOTORS) {
        return -1;
    }
    
    // 初始化控制器结构体
    memset(controller, 0, sizeof(multi_foc_controller_t));
    
    // 设置电机数量和模式
    controller->active_motors = num_motors;
    controller->mode = mode;
    
    // 初始化每个电机
    for (uint8_t i = 0; i < num_motors; i++) {
        // 初始化FOC电机对象
        if (foc_init(&controller->motors[i]) != 0) {
            return -1;
        }
        
        // 设置电机参数 (使用默认值，可根据实际需求调整)
        if (foc_set_motor_params(&controller->motors[i], MOTOR_PHASE_RESISTANCE, 
                                 MOTOR_KV_RATING, DRV8311_PWM_FREQ) != 0) {
            return -1;
        }
        
        // 设置驱动器参数 (每个电机可能有不同的PWM引脚配置)
        uint8_t pwm_a_pin, pwm_b_pin, pwm_c_pin;
        hal_gpio_port_t pwm_port;
        
        // 根据电机ID分配PWM引脚 (需要根据实际硬件配置调整)
        switch (i) {
            case 0: // 电机1 - 使用现有配置
                pwm_a_pin = PWM_A_PIN;
                pwm_b_pin = PWM_B_PIN;
                pwm_c_pin = PWM_C_PIN;
                pwm_port = PWM_A_GPIO_PORT; // 假设所有PWM都在同一端口
                break;
            case 1: // 电机2 - 使用备用引脚
                // 根据实际硬件连接定义备用PWM引脚
                // 这里使用虚拟引脚，需要根据实际硬件修改
                pwm_a_pin = PWM_A_PIN; // 需要根据实际硬件定义
                pwm_b_pin = PWM_B_PIN;
                pwm_c_pin = PWM_C_PIN;
                pwm_port = PWM_A_GPIO_PORT;
                break;
            case 2: // 电机3 - 使用备用引脚
                pwm_a_pin = PWM_A_PIN; // 需要根据实际硬件定义
                pwm_b_pin = PWM_B_PIN;
                pwm_c_pin = PWM_C_PIN;
                pwm_port = PWM_A_GPIO_PORT;
                break;
            default:
                return -1;
        }
        
        if (foc_set_driver_params(&controller->motors[i], DRV8311_SUPPLY_VOLTAGE, 
                                  DRV8311_FOC_VOLTAGE_LIM, pwm_a_pin, pwm_b_pin, pwm_c_pin) != 0) {
            return -1;
        }
        
        // 设置PID参数
        // 位置环PID
        foc_set_pid_constants(&controller->motors[i], 0, 2.0f, 0.0f, 0.0f);
        // 速度环PID
        foc_set_pid_constants(&controller->motors[i], 1, 0.2f, 0.1f, 0.0f);
        // 电流环PID
        foc_set_pid_constants(&controller->motors[i], 2, 1.0f, 0.0f, 0.0f);
        foc_set_pid_constants(&controller->motors[i], 3, 1.0f, 0.0f, 0.0f);
        
        // 默认使用编码器
        controller->motors[i].use_sensor = true;
    }
    
    return 0;
}

/**
 * @brief 设置指定电机的控制模式
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_control_mode(multi_foc_controller_t *controller, uint8_t motor_id, control_mode_e mode)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return -1;
    }
    
    return foc_set_control_mode(&controller->motors[motor_id], mode);
}

/**
 * @brief 设置指定电机的目标值
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_target(multi_foc_controller_t *controller, uint8_t motor_id, float target)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return -1;
    }
    
    controller->motors[motor_id].target = target;
    return 0;
}

/**
 * @brief 设置所有电机的统一目标值（同步模式下使用）
 * @param controller 多路控制器结构体指针
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int multi_foc_set_all_targets(multi_foc_controller_t *controller, float target)
{
    if (controller == NULL) {
        return -1;
    }
    
    for (uint8_t i = 0; i < controller->active_motors; i++) {
        controller->motors[i].target = target;
    }
    
    controller->master_target = target;
    return 0;
}

/**
 * @brief 启动指定电机
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 0表示成功，负数表示错误
 */
int multi_foc_start_motor(multi_foc_controller_t *controller, uint8_t motor_id)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return -1;
    }
    
    return foc_start(&controller->motors[motor_id]);
}

/**
 * @brief 停止指定电机
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 0表示成功，负数表示错误
 */
int multi_foc_stop_motor(multi_foc_controller_t *controller, uint8_t motor_id)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return -1;
    }
    
    return foc_stop(&controller->motors[motor_id]);
}

/**
 * @brief 启动所有电机
 * @param controller 多路控制器结构体指针
 * @return 0表示成功，负数表示错误
 */
int multi_foc_start_all(multi_foc_controller_t *controller)
{
    if (controller == NULL) {
        return -1;
    }
    
    for (uint8_t i = 0; i < controller->active_motors; i++) {
        foc_start(&controller->motors[i]);
    }
    
    return 0;
}

/**
 * @brief 停止所有电机
 * @param controller 多路控制器结构体指针
 * @return 0表示成功，负数表示错误
 */
int multi_foc_stop_all(multi_foc_controller_t *controller)
{
    if (controller == NULL) {
        return -1;
    }
    
    for (uint8_t i = 0; i < controller->active_motors; i++) {
        foc_stop(&controller->motors[i]);
    }
    
    return 0;
}

/**
 * @brief 多电机控制主循环
 * @param controller 多路控制器结构体指针
 * @param sensor_angles 传感器角度数组
 * @param dt 时间步长
 * @return 0表示成功，负数表示错误
 */
int multi_foc_control_loop(multi_foc_controller_t *controller, float *sensor_angles, float dt)
{
    if (controller == NULL || sensor_angles == NULL || dt <= 0) {
        return -1;
    }
    
    int result = 0;
    
    for (uint8_t i = 0; i < controller->active_motors; i++) {
        // 根据控制模式执行相应的操作
        if (controller->mode == MULTI_MOTOR_MODE_SYNCHRONIZED) {
            // 同步模式：所有电机使用相同的目标值
            controller->motors[i].target = controller->master_target;
        } else if (controller->mode == MULTI_MOTOR_MODE_FOLLOWER) {
            // 跟随模式：后续电机跟随第一个电机
            if (i > 0) {
                controller->motors[i].target = controller->motors[0].target;
            }
        }
        
        // 执行单个电机的FOC控制循环
        result = foc_control_cycle(&controller->motors[i], sensor_angles[i], dt);
        
        if (result != 0) {
            // 如果某个电机控制失败，可以选择继续或停止所有电机
            // 这里我们继续处理其他电机
            continue;
        }
        
        // 如果电机正在运行，更新PWM输出
        if (controller->motors[i].status == MOTOR_STATUS_RUNNING) {
            // 将计算出的PWM值应用到对应的定时器通道
            // 这里需要根据实际硬件配置设置对应的PWM输出
            uint16_t period = HAL_TMR_GET_PERIOD(HAL_PWM_TIMER);
            uint16_t pwm_a_val = (uint16_t)(controller->motors[i].Ua * period);
            uint16_t pwm_b_val = (uint16_t)(controller->motors[i].Ub * period);
            uint16_t pwm_c_val = (uint16_t)(controller->motors[i].Uc * period);
            
            // 根据电机ID设置对应的PWM通道
            switch (i) {
                case 0: // 电机1 - 使用主要通道
                    HAL_TMR_SET_CMP(HAL_PWM_TIMER, HAL_PWM_CHANNEL_U, pwm_a_val);
                    HAL_TMR_SET_CMP(HAL_PWM_TIMER, HAL_PWM_CHANNEL_V, pwm_b_val);
                    HAL_TMR_SET_CMP(HAL_PWM_TIMER, HAL_PWM_CHANNEL_W, pwm_c_val);
                    break;
                case 1: // 电机2 - 使用其他定时器或复用通道
                    // 根据实际硬件配置设置
                    break;
                case 2: // 电机3 - 使用其他定时器或复用通道
                    // 根据实际硬件配置设置
                    break;
                default:
                    break;
            }
        }
    }
    
    return result;
}

/**
 * @brief 获取指定电机的位置
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 位置值
 */
float multi_foc_get_position(multi_foc_controller_t *controller, uint8_t motor_id)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return 0.0f;
    }
    
    return controller->motors[motor_id].estimator.angle;
}

/**
 * @brief 获取指定电机的速度
 * @param controller 多路控制器结构体指针
 * @param motor_id 电机ID
 * @return 速度值
 */
float multi_foc_get_velocity(multi_foc_controller_t *controller, uint8_t motor_id)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return 0.0f;
    }
    
    return controller->motors[motor_id].estimator.velocity;
}

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
                               uint8_t pid_type, float kp, float ki, float kd)
{
    if (controller == NULL || motor_id >= controller->active_motors) {
        return -1;
    }
    
    return foc_set_pid_constants(&controller->motors[motor_id], pid_type, kp, ki, kd);
}