/**
 * @file    foc_utils.h
 * @author  FOC工具函数
 * @date    2026-01-09
 * @brief   提供FOC控制的一些实用函数
 */

#ifndef __FOC_UTILS_H
#define __FOC_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 将角度归一化到[-π, π]范围
 * @param angle 输入角度
 * @return 归一化后的角度
 */
float normalize_angle(float angle);

/**
 * @brief 将电角度转换为机械角度
 * @param electrical_angle 电角度
 * @param pole_pairs 极对数
 * @return 机械角度
 */
float mechanical_angle_calc(float electrical_angle, int pole_pairs);

/**
 * @brief 电机参数校准函数
 * @return 0表示成功，负数表示错误
 */
int motor_calibration(void);

/**
 * @brief 设置FOC控制参数
 * @param kp 位置环比例增益
 * @param ki 位置环积分增益
 * @param kd 位置环微分增益
 * @return 0表示成功，负数表示错误
 */
int set_position_pid(float kp, float ki, float kd);

/**
 * @brief 设置速度环PID参数
 * @param kp 速度环比例增益
 * @param ki 速度环积分增益
 * @param kd 速度环微分增益
 * @return 0表示成功，负数表示错误
 */
int set_velocity_pid(float kp, float ki, float kd);

/**
 * @brief 设置电流环PID参数
 * @param kp 电流环比例增益
 * @param ki 电流环积分增益
 * @param kd 电流环微分增益
 * @return 0表示成功，负数表示错误
 */
int set_current_pid(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_UTILS_H */