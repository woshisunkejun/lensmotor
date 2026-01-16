/**
 * @file    foc_utils.c
 * @author  FOC工具函数实现
 * @date    2026-01-09
 * @brief   提供FOC控制的一些实用函数实现
 */

#include "foc_utils.h"
#include "foc_controller.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 将角度归一化到[-π, π]范围
 * @param angle 输入角度
 * @return 归一化后的角度
 */
float normalize_angle(float angle)
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle <= -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

/**
 * @brief 将电角度转换为机械角度
 * @param electrical_angle 电角度
 * @param pole_pairs 极对数
 * @return 机械角度
 */
float mechanical_angle_calc(float electrical_angle, int pole_pairs)
{
    if (pole_pairs == 0) return 0.0f;
    return electrical_angle / pole_pairs;
}

/**
 * @brief 电机参数校准函数
 * @return 0表示成功，负数表示错误
 */
int motor_calibration(void)
{
    // 停止电机（默认停止第一个电机）
    foc_controller_stop(0);
    
    // TODO: 实现电机校准流程
    // 1. 零偏置校准
    // 2. 电阻/电感测量
    // 3. 极对数检测
    // 4. 霍尔传感器/编码器方向校准
    
    return 0;
}

/**
 * @brief 设置FOC控制参数
 * @param kp 位置环比例增益
 * @param ki 位置环积分增益
 * @param kd 位置环微分增益
 * @return 0表示成功，负数表示错误
 */
int set_position_pid(float kp, float ki, float kd)
{
    // 设置第一个电机的位置环PID参数
    g_motors[0].motor.angle_pid.kp = kp;
    g_motors[0].motor.angle_pid.ki = ki;
    g_motors[0].motor.angle_pid.kd = kd;
    return 0;
}

/**
 * @brief 设置速度环PID参数
 * @param kp 速度环比例增益
 * @param ki 速度环积分增益
 * @param kd 速度环微分增益
 * @return 0表示成功，负数表示错误
 */
int set_velocity_pid(float kp, float ki, float kd)
{
    // 设置第一个电机的速度环PID参数
    g_motors[0].motor.velocity_pid.kp = kp;
    g_motors[0].motor.velocity_pid.ki = ki;
    g_motors[0].motor.velocity_pid.kd = kd;
    return 0;
}

/**
 * @brief 设置电流环PID参数
 * @param kp 电流环比例增益
 * @param ki 电流环积分增益
 * @param kd 电流环微分增益
 * @return 0表示成功，负数表示错误
 */
int set_current_pid(float kp, float ki, float kd)
{
    // 设置第一个电机的电流环PID参数
    g_motors[0].motor.current_q_pid.kp = kp;
    g_motors[0].motor.current_q_pid.ki = ki;
    g_motors[0].motor.current_q_pid.kd = kd;
    g_motors[0].motor.current_d_pid.kp = kp;
    g_motors[0].motor.current_d_pid.ki = ki;
    g_motors[0].motor.current_d_pid.kd = kd;
    return 0;
}