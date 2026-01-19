/**
 * @file    foc_controller.c
 * @author  FOC控制器接口实现
 * @date    2026-01-09
 * @brief   用于集成SimpleFOC到AT32F423工程的接口实现
 */

#include "foc_controller.h"
#include "at32f423_wk_config.h"
#include "wk_tmr.h"
#include "wk_gpio.h"
#include "drv8311_config.h"
#include "hal_interface.h"
#include <math.h>
#include <string.h>
#define M_PI		3.14159265358979323846
/* 全局变量 */
motor_control_t g_motors[MAX_MOTORS] = {0};  /*!< 多电机控制数组 */
/**
 * @brief 检查指定电机的DRV8311是否过流
 * @param motor_index 电机索引 (0-2)
 * @return true表示过流，false表示正常
 */
bool drv8311_check_overcurrent(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return false;
    
    // 检查任何一相电流超过限制
    if (HAL_FABS(g_motors[motor_index].motor.current_a) > DRV8311_OVERCURRENT_LIM || 
        HAL_FABS(g_motors[motor_index].motor.current_b) > DRV8311_OVERCURRENT_LIM || 
        HAL_FABS(g_motors[motor_index].motor.current_c) > DRV8311_OVERCURRENT_LIM) {
        g_motors[motor_index].overcurrent = true;
        return true;
    }
    g_motors[motor_index].overcurrent = false;
    return false;
}

/**
 * @brief 检查指定电机的DRV8311是否过温
 * @param motor_index 电机索引 (0-2)
 * @return true表示过温，false表示正常
 */
bool drv8311_check_overtemperature(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return false;
    
    // 在实际应用中，这里可能需要读取温度传感器
    // 暂时返回false（正常）
    return false;
}

/**
 * @brief 使能/禁用指定电机的DRV8311
 * @param motor_index 电机索引 (0-2)
 * @param enable true为使能，false为禁用
 */
void drv8311_set_enable(uint8_t motor_index, bool enable)
{
    if(motor_index >= MAX_MOTORS) return;
    
    // 注意：这里需要根据电机索引来选择对应的使能引脚
    // 由于用户没有提供多电机的引脚配置，暂时使用相同的引脚
    // 在实际应用中，每个电机应该有自己的使能引脚
    if (enable) {
        switch(motor_index) {
            case 0:
                HAL_GPIO_SET(HAL_DRV8311_ENABLE_PORT, HAL_DRV8311_ENABLE_PIN);
                break;
            // 如果有多个电机，需要在这里添加其他电机的使能引脚
            // case 1: HAL_GPIO_SET(MOTOR1_DRV8311_ENABLE_PORT, MOTOR1_DRV8311_ENABLE_PIN); break;
            // case 2: HAL_GPIO_SET(MOTOR2_DRV8311_ENABLE_PORT, MOTOR2_DRV8311_ENABLE_PIN); break;
            default:
                break;
        }
    } else {
        switch(motor_index) {
            case 0:
                HAL_GPIO_RESET(HAL_DRV8311_ENABLE_PORT, HAL_DRV8311_ENABLE_PIN);
                break;
            // 如果有多个电机，需要在这里添加其他电机的使能引脚
            // case 1: HAL_GPIO_RESET(MOTOR1_DRV8311_ENABLE_PORT, MOTOR1_DRV8311_ENABLE_PIN); break;
            // case 2: HAL_GPIO_RESET(MOTOR2_DRV8311_ENABLE_PORT, MOTOR2_DRV8311_ENABLE_PIN); break;
            default:
                break;
        }
    }
    
    g_motors[motor_index].enabled = enable;
}



/**
 * @brief 读取霍尔传感器值并计算角度
 */
static float read_hall_angle(void)
{
    // 读取霍尔传感器引脚
    uint8_t ha = HAL_GPIO_READ(HAL_HALL_U_PORT, HAL_HALL_U_PIN) ? 1 : 0;
    uint8_t hb = HAL_GPIO_READ(HAL_HALL_V_PORT, HAL_HALL_V_PIN) ? 1 : 0;
    uint8_t hc = HAL_GPIO_READ(HAL_HALL_W_PORT, HAL_HALL_W_PIN) ? 1 : 0;
    
    // 组合霍尔传感器值
    uint8_t hall_state = (hc << 2) | (hb << 1) | ha;
    
    // 根据霍尔传感器状态确定扇区 (标准6步换向序列)
    // DRV8311支持的霍尔传感器序列
    float hall_angles[8] = {
        0.0f,                    // 000 - 无效状态
        180.0f * M_PI / 180.0f,  // 001 - 扇区5 (C+ B-)
        120.0f * M_PI / 180.0f,  // 010 - 扇区3 (A+ C-)
        180.0f * M_PI / 180.0f,  // 011 - 扇区1 (B+ C-)
        60.0f * M_PI / 180.0f,   // 100 - 扇区2 (A+ B-)
        0.0f,                    // 101 - 扇区0 (A- B+)
        300.0f * M_PI / 180.0f,  // 110 - 扇区4 (B- A+)
        240.0f * M_PI / 180.0f   // 111 - 扇区6 (C- A+)
    };
    
    // 使用查表法获取粗略角度
    if (hall_state >= 1 && hall_state <= 7) {
        return hall_angles[hall_state];
    }
    
    return 0.0f; // 无效状态返回0
}

/**
 * @brief 读取编码器值
 */
static float read_encoder_angle(void)
{
    // 从定时器读取编码器计数值并转换为角度
    uint32_t encoder_count = HAL_TMR_GET_CNT(HAL_ENCODER_TIMER);
    
    // 假设编码器分辨率是4096线
    const float encoder_resolution = 4096.0f;
    float angle = (encoder_count % (uint32_t)encoder_resolution) * 2.0f * M_PI / encoder_resolution;
    
    return angle;
}

/**
 * @brief 读取指定电机的电流值
 * @param motor_index 电机索引 (0-2)
 */
static void read_current_values(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return;
    
    extern uint16_t adc_dma_buffer[3];  // 引用main.c中的DMA缓冲区
    
    // 从DMA缓冲区读取电流值
    // 假设缓冲区顺序为：I_A, I_B, I_C 对应 adc_dma_buffer[0], [1], [2]
    // 需要根据实际硬件连接和电路设计转换为实际电流值
    // DRV8311电流检测增益为2V/A，ADC参考电压为3.3V，12位ADC (4096)
    // 零点偏移在DRV8311_CONFIG中定义
    float adc_to_volts = 3.3f / 4096.0f;  // ADC到电压转换系数
    
    // 将ADC值转换为电流 (考虑零点偏移和电流增益)
    g_motors[motor_index].motor.current_a = ((float)adc_dma_buffer[0] - DRV8311_CS_ZERO_OFFSET) * adc_to_volts / DRV8311_CS_GAIN;
    g_motors[motor_index].motor.current_b = ((float)adc_dma_buffer[1] - DRV8311_CS_ZERO_OFFSET) * adc_to_volts / DRV8311_CS_GAIN;
    g_motors[motor_index].motor.current_c = ((float)adc_dma_buffer[2] - DRV8311_CS_ZERO_OFFSET) * adc_to_volts / DRV8311_CS_GAIN;
    
    // 电流限制检查
    if (HAL_FABS(g_motors[motor_index].motor.current_a) > DRV8311_OVERCURRENT_LIM || 
        HAL_FABS(g_motors[motor_index].motor.current_b) > DRV8311_OVERCURRENT_LIM || 
        HAL_FABS(g_motors[motor_index].motor.current_c) > DRV8311_OVERCURRENT_LIM) {
        // 过流保护：可以触发保护机制或降低输出
        // 这里只是简单限制电流值
        g_motors[motor_index].motor.current_a = HAL_FMAX(-DRV8311_OVERCURRENT_LIM, HAL_FMIN(DRV8311_OVERCURRENT_LIM, g_motors[motor_index].motor.current_a));
        g_motors[motor_index].motor.current_b = HAL_FMAX(-DRV8311_OVERCURRENT_LIM, HAL_FMIN(DRV8311_OVERCURRENT_LIM, g_motors[motor_index].motor.current_b));
        g_motors[motor_index].motor.current_c = HAL_FMAX(-DRV8311_OVERCURRENT_LIM, HAL_FMIN(DRV8311_OVERCURRENT_LIM, g_motors[motor_index].motor.current_c));
    }
}

/**
 * @brief 更新指定电机的霍尔传感器读取
 * @param motor_index 电机索引 (0-2)
 */
void update_hall_sensors(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return;
    g_motors[motor_index].motor.hall_angle = read_hall_angle();
}

/**
 * @brief 更新指定电机的编码器读取
 * @param motor_index 电机索引 (0-2)
 */
void update_encoder(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return;
    g_motors[motor_index].motor.encoder_angle = read_encoder_angle();
}

/**
 * @brief 更新指定电机的电流检测
 * @param motor_index 电机索引 (0-2)
 */
void update_current_sense(uint8_t motor_index)
{
    read_current_values(motor_index);
}

/**
 * @brief 初始化指定电机
 * @param motor_index 电机索引 (0-2)
 * @return 0表示成功，负数表示错误
 */
int init_motor(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return -1;

    if (foc_init(&g_motors[motor_index].motor) != 0) {
        return -1;
    }
    
    // 初始化电机对象
    g_motors[motor_index].motor.motor_type = MOTOR_TYPE_BLDC;
    g_motors[motor_index].motor.pole_pairs = 7;  // 根据用户配置
    g_motors[motor_index].motor.phase_resistance = 5.0f;  // 根据用户配置
    
    // 初始化PID控制器
    g_motors[motor_index].motor.velocity_pid.kp = 0.2f;
    g_motors[motor_index].motor.velocity_pid.ki = 0.01f;
    g_motors[motor_index].motor.velocity_pid.kd = 0.0f;
    g_motors[motor_index].motor.velocity_pid.output_ramp = 1000.0f;
    g_motors[motor_index].motor.velocity_pid.limit = 10.0f;
    
    g_motors[motor_index].motor.angle_pid.kp = 20.0f;
    g_motors[motor_index].motor.angle_pid.ki = 0.0f;
    g_motors[motor_index].motor.angle_pid.kd = 0.01f;
    g_motors[motor_index].motor.angle_pid.output_ramp = 1000.0f;
    g_motors[motor_index].motor.angle_pid.limit = 20.0f;
    
    // 初始化FOC状态
    g_motors[motor_index].motor.controller = FOC_CONTROL_ANGLE;  // 默认角度控制
    g_motors[motor_index].motor.velocity_limit = 10.0f;  // 限制最大速度
    g_motors[motor_index].motor.voltage_limit = 11.0f;   // 限制最大电压
    g_motors[motor_index].motor.current_limit = 2.0f;    // 限制最大电流
    
    // 初始化传感器
    g_motors[motor_index].motor.sensor_type = SENSOR_TYPE_ENCODER;
    
    g_motors[motor_index].motor_index = motor_index;
    g_motors[motor_index].enabled = false;
    g_motors[motor_index].overcurrent = false;
    g_motors[motor_index].target_velocity = 0.0f;
    g_motors[motor_index].target_position = 0.0f;
    g_motors[motor_index].current_position = 0.0f;
    g_motors[motor_index].current_velocity = 0.0f;
    
    return 0;
}

/**
 * @brief FOC控制器初始化
 * @return 0表示成功，负数表示错误
 */
int foc_controller_init(void)
{
    // 初始化所有电机
    for(uint8_t i = 0; i < MAX_MOTORS; i++) {
        int result = init_motor(i);
        if(result != 0) {
            return result;
        }
    }
    
    // 使能第一个DRV8311 (假设使能引脚已在系统初始化中配置)
    drv8311_set_enable(0, true);
    
    return 0;
}

/**
 * @brief 设置指定电机的FOC控制模式
 * @param motor_index 电机索引 (0-2)
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int foc_controller_set_mode(uint8_t motor_index, control_mode_e mode)
{
    if(motor_index >= MAX_MOTORS) return -1;
    return foc_set_control_mode(&g_motors[motor_index].motor, mode);
}

/**
 * @brief 设置指定电机的目标值
 * @param motor_index 电机索引 (0-2)
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int foc_controller_set_target(uint8_t motor_index, float target)
{
    if(motor_index >= MAX_MOTORS) return -1;
    return foc_set_target(&g_motors[motor_index].motor, target);
}

/**
 * @brief 获取指定电机的当前位置
 * @param motor_index 电机索引 (0-2)
 * @return 当前位置
 */
float foc_controller_get_position(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return 0.0f;
    return g_motors[motor_index].motor.estimator.angle;
}

/**
 * @brief 获取指定电机的当前速度
 * @param motor_index 电机索引 (0-2)
 * @return 当前速度
 */
float foc_controller_get_velocity(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return 0.0f;
    return g_motors[motor_index].motor.estimator.velocity;
}

/**
 * @brief 控制指定电机
 * @param motor_index 电机索引 (0-2)
 */
void control_motor(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return;
    
    // 计算时间差 (假设系统定时器提供时间基准)
    static float last_time[MAX_MOTORS] = {0.0f};
    float current_time = (float)HAL_TMR_GET_CNT(HAL_PWM_TIMER) / 1000000.0f; // 假设TMR3提供微秒时间
    float dt = current_time - last_time[motor_index];
    if (dt <= 0) dt = 0.001f; // 默认1ms
    last_time[motor_index] = current_time;
    
    // 选择传感器角度（如果使用编码器优先使用编码器）
    float sensor_angle = g_motors[motor_index].motor.use_sensor ? 
                         g_motors[motor_index].motor.encoder_angle : 
                         g_motors[motor_index].motor.hall_angle;
    
    // 执行FOC控制循环
    int result = foc_control_cycle(&g_motors[motor_index].motor, sensor_angle, dt);
    
    if (result == 0 && g_motors[motor_index].motor.status == MOTOR_STATUS_RUNNING) {
        // 将计算出的PWM值应用到定时器
        // 转换为定时器计数值
        uint16_t period = HAL_TMR_GET_PERIOD(HAL_PWM_TIMER);
        uint16_t pwm_a_val = (uint16_t)(g_motors[motor_index].motor.Ua * period);
        uint16_t pwm_b_val = (uint16_t)(g_motors[motor_index].motor.Ub * period);
        uint16_t pwm_c_val = (uint16_t)(g_motors[motor_index].motor.Uc * period);
        
        // 根据电机索引设置对应的PWM输出
        // 注意：这里需要根据实际硬件连接设置不同的PWM通道给不同电机
        switch(motor_index) {
            case 0:
                // 设置PWM定时器的比较寄存器值 - 电机0使用默认通道
                HAL_TMR_SET_CMP(HAL_PWM_TIMER, HAL_PWM_CHANNEL_U, pwm_a_val);
                HAL_TMR_SET_CMP(HAL_PWM_TIMER, HAL_PWM_CHANNEL_V, pwm_b_val);
                HAL_TMR_SET_CMP(HAL_PWM_TIMER, HAL_PWM_CHANNEL_W, pwm_c_val);
                break;
            // case 1: 
            //     // 电机1使用其他通道 (需根据实际硬件配置)
            //     HAL_TMR_SET_CMP(HAL_PWM_TIMER, TMR_SELECT_CHANNEL_4, pwm_a_val);
            //     HAL_TMR_SET_CMP(HAL_PWM_TIMER, TMR_SELECT_CHANNEL_5, pwm_b_val);
            //     HAL_TMR_SET_CMP(HAL_PWM_TIMER, TMR_SELECT_CHANNEL_6, pwm_c_val);
            //     break;
            // case 2:
            //     // 电机2使用其他通道 (需根据实际硬件配置)
            //     HAL_TMR_SET_CMP(HAL_PWM_TIMER, TMR_SELECT_CHANNEL_7, pwm_a_val);
            //     HAL_TMR_SET_CMP(HAL_PWM_TIMER, TMR_SELECT_CHANNEL_8, pwm_b_val);
            //     HAL_TMR_SET_CMP(HAL_PWM_TIMER, TMR_SELECT_CHANNEL_9, pwm_c_val);
            //     break;
            default:
                break;
        }
    }
}

/**
 * @brief 主控制循环 - 控制所有启用的电机
 * @return 0表示成功，负数表示错误
 */
int foc_controller_loop(void)
{
    // 控制所有启用的电机
    for(uint8_t i = 0; i < MAX_MOTORS; i++) {
        if(g_motors[i].enabled) {
            control_motor(i);
        }
    }
    
    return 0;
}

/**
 * @brief 启动指定电机
 * @param motor_index 电机索引 (0-2)
 * @return 0表示成功，负数表示错误
 */
int foc_controller_start(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return -1;
    drv8311_set_enable(motor_index, true);
    return foc_start(&g_motors[motor_index].motor);
}

/**
 * @brief 停止指定电机
 * @param motor_index 电机索引 (0-2)
 * @return 0表示成功，负数表示错误
 */
int foc_controller_stop(uint8_t motor_index)
{
    if(motor_index >= MAX_MOTORS) return -1;
    int result = foc_stop(&g_motors[motor_index].motor);
    drv8311_set_enable(motor_index, false);
    return result;
}
