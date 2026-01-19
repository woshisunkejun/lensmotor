/**
 * @file    simplefoc_c.c
 * @author  SimpleFOC C语言实现
 * @date    2026-01-09
 * @brief   SimpleFOC库的C语言实现，用于FOC（磁场定向控制）无刷电机控制
 */

#include "simplefoc_c.h"
#include "hal_interface.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief PID控制器初始化
 * @param pid PID结构体指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param limit 输出限幅
 * @param output_ramp 输出斜坡
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float limit, float output_ramp)
{
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_ramp = output_ramp;
    pid->limit = limit;
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output_prev = 0.0f;
}

/**
 * @brief PID控制器计算
 * @param pid PID结构体指针
 * @param error 误差
 * @param dt 时间步长
 * @return PID输出
 */
float pid_compute(pid_controller_t *pid, float error, float dt)
{
    if (pid == NULL || dt <= 0) return 0.0f;
    
    float output;
    
    // 积分项
    pid->integral += error * dt;
    
    // 积分限幅
    if (pid->integral > pid->limit) pid->integral = pid->limit;
    if (pid->integral < -pid->limit) pid->integral = -pid->limit;
    
    // 微分项
    pid->derivative = (error - pid->error_prev) / dt;
    pid->error_prev = error;
    
    // PID输出计算
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * pid->derivative;
    
    // 输出限幅
    if (output > pid->limit) output = pid->limit;
    if (output < -pid->limit) output = -pid->limit;
    
    // 输出斜坡限制
    if (pid->output_ramp > 0) {
        float output_rate = output - pid->output_prev;
        float max_step = pid->output_ramp * dt;
        if (output_rate > max_step) {
            output = pid->output_prev + max_step;
        } else if (output_rate < -max_step) {
            output = pid->output_prev - max_step;
        }
        pid->output_prev = output;
    }
    
    return output;
}

/**
 * @brief 低通滤波器初始化
 * @param lpf 低通滤波器结构体指针
 * @param Tf 时间常数
 */
void lpf_init(low_pass_filter_t *lpf, float Tf)
{
    if (lpf == NULL) return;
    
    lpf->Tf = Tf;
    lpf->y_prev = 0.0f;
    lpf->x_prev = 0.0f;
}

/**
 * @brief 低通滤波器计算
 * @param lpf 低通滤波器结构体指针
 * @param input 输入值
 * @param dt 时间步长
 * @return 滤波后输出
 */
float lpf_compute(low_pass_filter_t *lpf, float input, float dt)
{
    if (lpf == NULL || dt <= 0) return 0.0f;
    
    // 一阶低通滤波器实现
    float alpha = dt / (lpf->Tf + dt);
    float output = alpha * input + (1.0f - alpha) * lpf->y_prev;
    lpf->y_prev = output;
    lpf->x_prev = input;
    
    return output;
}

/**
 * @brief 克拉克变换 (3相 -> 2相静止坐标系)
 * @param a A相输入
 * @param b B相输入
 * @param c C相输入
 * @param alpha 阿尔法轴输出
 * @param beta 贝塔轴输出
 */
void clarke_transform(float a, float b, float c, float *alpha, float *beta)
{
    if (alpha == NULL || beta == NULL) return;
    
    *alpha = a;
    *beta = (a + 2.0f * b + c) / 1.7320508075688772f; // sqrt(3)
}

/**
 * @brief 帕克变换 (2相静止 -> 2相旋转坐标系)
 * @param alpha 阿尔法轴输入
 * @param beta 贝塔轴输入
 * @param angle 电角度
 * @param d D轴输出
 * @param q Q轴输出
 */
void park_transform(float alpha, float beta, float angle, float *d, float *q)
{
    if (d == NULL || q == NULL) return;
    
    float sin_angle = HAL_SIN(angle);
    float cos_angle = HAL_COS(angle);
    
    *d = alpha * cos_angle + beta * sin_angle;
    *q = beta * cos_angle - alpha * sin_angle;
}

/**
 * @brief 反帕克变换 (2相旋转 -> 2相静止坐标系)
 * @param d D轴输入
 * @param q Q轴输入
 * @param angle 电角度
 * @param alpha 阿尔法轴输出
 * @param beta 贝塔轴输出
 */
void inverse_park_transform(float d, float q, float angle, float *alpha, float *beta)
{
    if (alpha == NULL || beta == NULL) return;
    
    float sin_angle = HAL_SIN(angle);
    float cos_angle = HAL_COS(angle);
    
    *alpha = d * cos_angle - q * sin_angle;
    *beta = d * sin_angle + q * cos_angle;
}

/**
 * @brief 反克拉克变换 (2相静止 -> 3相)
 * @param alpha 阿尔法轴输入
 * @param beta 贝塔轴输入
 * @param a A相输出
 * @param b B相输出
 * @param c C相输出
 */
void inverse_clarke_transform(float alpha, float beta, float *a, float *b, float *c)
{
    if (a == NULL || b == NULL || c == NULL) return;
    
    *a = alpha;
    *b = -0.5f * alpha + 0.8660254037844386f * beta; // -0.5 * alpha + sqrt(3)/2 * beta
    *c = -0.5f * alpha - 0.8660254037844386f * beta; // -0.5 * alpha - sqrt(3)/2 * beta
}

/**
 * @brief SVPWM (空间矢量脉宽调制) 计算 - 针对DRV8311 3xPWM模式优化
 * @param u_alpha 阿尔法轴电压
 * @param u_beta 贝塔轴电压
 * @param voltage_limit 电压限制
 * @param dt 时间步长
 * @param pwm_a A相PWM占空比输出
 * @param pwm_b B相PWM占空比输出
 * @param pwm_c C相PWM占空比输出
 */
void svpwm_compute(float u_alpha, float u_beta, float voltage_limit, float dt, 
                   float *pwm_a, float *pwm_b, float *pwm_c)
{
    if (pwm_a == NULL || pwm_b == NULL || pwm_c == NULL) return;
    
    // 限制电压幅值
    float amplitude = HAL_SQRT(u_alpha * u_alpha + u_beta * u_beta);
    if (amplitude > voltage_limit) {
        u_alpha = (u_alpha / amplitude) * voltage_limit;
        u_beta = (u_beta / amplitude) * voltage_limit;
    }
    
    // 反克拉克变换得到三相电压
    inverse_clarke_transform(u_alpha, u_beta, pwm_a, pwm_b, pwm_c);
    
    // 对于DRV8311的3xPWM模式，需要进行适当的偏置和归一化
    // 找到最小值和最大值，以便进行七段SVPWM调制
    float min_pwm = HAL_FMIN(HAL_FMIN(*pwm_a, *pwm_b), *pwm_c);
    float max_pwm = HAL_FMAX(HAL_FMAX(*pwm_a, *pwm_b), *pwm_c);
    
    // 将范围从[min_pwm, max_pwm]映射到[0, 1]
    float range = max_pwm - min_pwm;
    if (range > 0.001f) { // 避免除零
        *pwm_a = (*pwm_a - min_pwm) / range;
        *pwm_b = (*pwm_b - min_pwm) / range;
        *pwm_c = (*pwm_c - min_pwm) / range;
    } else {
        // 如果所有值都相同，则设置为0.5 (50%占空比)
        *pwm_a = *pwm_b = *pwm_c = 0.5f;
    }
    
    // 限制在0-1范围内
    *pwm_a = HAL_FMAX(0.0f, HAL_FMIN(1.0f, *pwm_a));
    *pwm_b = HAL_FMAX(0.0f, HAL_FMIN(1.0f, *pwm_b));
    *pwm_c = HAL_FMAX(0.0f, HAL_FMIN(1.0f, *pwm_c));
}

/**
 * @brief 估算电角度
 * @param mechanical_angle 机械角度
 * @param pole_pairs 极对数
 * @return 电角度
 */
float electrical_angle_calc(float mechanical_angle, int pole_pairs)
{
    return mechanical_angle * pole_pairs;
}

/**
 * @brief 磁场定向控制初始化函数
 * @param motor 电机结构体指针
 * @return 0表示成功，负数表示错误
 */
int foc_init(foc_motor_t *motor)
{
    if (motor == NULL) return -1;
    
    // 初始化结构体
    memset(motor, 0, sizeof(foc_motor_t));
    
    // 设置默认参数
    motor->motor_type = MOTOR_TYPE_BLDC;
    motor->controller = FOC_CONTROL_DISABLED;
    motor->status = MOTOR_STATUS_STOPPED;
    motor->phase_resistance = 5.0f;  // 5欧姆默认值
    motor->kv_rating = 1000.0f;      // 1000 KV默认值
    motor->driver.voltage_power_supply = 12.0f;  // 12V默认值
    motor->driver.voltage_limit = 11.0f;         // 11V默认值
    motor->driver.pwm_frequency = 20000.0f;      // 20kHz默认值
    
    // 初始化PID控制器
    pid_init(&motor->angle_pid, 20.0f, 0.0f, 0.01f, 20.0f, 1000.0f);
    pid_init(&motor->velocity_pid, 0.2f, 0.01f, 0.0f, 10.0f, 1000.0f);
    pid_init(&motor->current_q_pid, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f);
    pid_init(&motor->current_d_pid, 1.0f, 0.0f, 0.0f, 10.0f, 0.0f);
    
    // 初始化滤波器
    lpf_init(&motor->velocity_filter, 0.01f);
    lpf_init(&motor->angle_filter, 0.01f);
    
    // 初始化其他参数
    motor->velocity_limit = 10.0f;
    motor->voltage_limit = 11.0f;
    motor->current_limit = 2.0f;
    motor->pole_pairs = 7;
    motor->use_sensor = true;
    motor->target_velocity = 0.0f;
    
    return 0;
}

/**
 * @brief 设置电机参数
 * @param motor 电机结构体指针
 * @param phase_resistance 相电阻
 * @param kv_rating KV值
 * @param pwm_frequency PWM频率
 * @return 0表示成功，负数表示错误
 */
int foc_set_motor_params(foc_motor_t *motor, float phase_resistance, float kv_rating, float pwm_frequency)
{
    if (motor == NULL) return -1;
    
    motor->phase_resistance = phase_resistance;
    motor->kv_rating = kv_rating;
    motor->driver.pwm_frequency = pwm_frequency;
    
    return 0;
}

/**
 * @brief 设置PID参数
 * @param motor 电机结构体指针
 * @param pid_type PID类型 (0=angle, 1=velocity, 2=current_q, 3=current_d)
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 0表示成功，负数表示错误
 */
int foc_set_pid_constants(foc_motor_t *motor, uint8_t pid_type, float kp, float ki, float kd)
{
    if (motor == NULL) return -1;
    
    switch (pid_type) {
        case 0: // angle PID
            motor->angle_pid.kp = kp;
            motor->angle_pid.ki = ki;
            motor->angle_pid.kd = kd;
            break;
        case 1: // velocity PID
            motor->velocity_pid.kp = kp;
            motor->velocity_pid.ki = ki;
            motor->velocity_pid.kd = kd;
            break;
        case 2: // current_q PID
            motor->current_q_pid.kp = kp;
            motor->current_q_pid.ki = ki;
            motor->current_q_pid.kd = kd;
            break;
        case 3: // current_d PID
            motor->current_d_pid.kp = kp;
            motor->current_d_pid.ki = ki;
            motor->current_d_pid.kd = kd;
            break;
        default:
            return -1;
    }
    
    return 0;
}

/**
 * @brief 设置控制模式
 * @param motor 电机结构体指针
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int foc_set_control_mode(foc_motor_t *motor, control_mode_e mode)
{
    if (motor == NULL) return -1;
    
    motor->controller = mode;
    if (mode == FOC_CONTROL_OPENLOOP) {
        motor->use_sensor = false;
    }
    return 0;
}

/**
 * @brief 设置目标值
 * @param motor 电机结构体指针
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int foc_set_target(foc_motor_t *motor, float target)
{
    if (motor == NULL) return -1;
    
    motor->target = target;
    motor->target_velocity = target;
    return 0;
}

/**
 * @brief 设置驱动器参数
 * @param motor 电机结构体指针
 * @param voltage_power_supply 电源电压
 * @param voltage_limit 电压限制
 * @param pwm_a A相PWM引脚
 * @param pwm_b B相PWM引脚
 * @param pwm_c C相PWM引脚
 * @return 0表示成功，负数表示错误
 */
int foc_set_driver_params(foc_motor_t *motor, float voltage_power_supply, float voltage_limit, 
                          uint8_t pwm_a, uint8_t pwm_b, uint8_t pwm_c)
{
    if (motor == NULL) return -1;
    
    motor->driver.voltage_power_supply = voltage_power_supply;
    motor->driver.voltage_limit = voltage_limit;
    motor->driver.pwm_a = pwm_a;
    motor->driver.pwm_b = pwm_b;
    motor->driver.pwm_c = pwm_c;
    
    return 0;
}

/**
 * @brief 启动电机
 * @param motor 电机结构体指针
 * @return 0表示成功，负数表示错误
 */
int foc_start(foc_motor_t *motor)
{
    if (motor == NULL) return -1;
    
    motor->status = MOTOR_STATUS_RUNNING;
    return 0;
}

/**
 * @brief 停止电机
 * @param motor 电机结构体指针
 * @return 0表示成功，负数表示错误
 */
int foc_stop(foc_motor_t *motor)
{
    if (motor == NULL) return -1;
    
    motor->status = MOTOR_STATUS_STOPPED;
    
    // 设置PWM为0，使电机停止
    motor->Ua = 0.0f;
    motor->Ub = 0.0f;
    motor->Uc = 0.0f;
    
    return 0;
}

/**
 * @brief 主控制循环
 * @param motor 电机结构体指针
 * @param sensor_angle 传感器角度
 * @param dt 时间步长
 * @return 0表示成功，负数表示错误
 */
int foc_control_cycle(foc_motor_t *motor, float sensor_angle, float dt)
{
    if (motor == NULL || dt <= 0) return -1;
    
    motor->dt = dt;
    
    if (motor->controller == FOC_CONTROL_OPENLOOP) {
        float angle_step = motor->target_velocity * dt * motor->pole_pairs;
        motor->angle_el += angle_step;
        if (motor->angle_el > 2.0f * M_PI || motor->angle_el < -2.0f * M_PI) {
            motor->angle_el = fmodf(motor->angle_el, 2.0f * M_PI);
        }
        motor->estimator.angle_prev = motor->estimator.angle;
        motor->estimator.angle = motor->angle_el / motor->pole_pairs;
        motor->estimator.velocity = motor->target_velocity;
    } else {
    // 更新估算器
    motor->estimator.angle_prev = motor->estimator.angle;
    motor->estimator.angle = sensor_angle;
    motor->estimator.velocity = (motor->estimator.angle - motor->estimator.angle_prev) / dt;
    
    // 通过低通滤波器过滤速度
    motor->estimator.velocity = lpf_compute(&motor->velocity_filter, motor->estimator.velocity, dt);
    
    // 计算电角度
    // 注意：这里需要根据实际电机的极对数来计算，暂时使用默认值7对极
    motor->angle_el = electrical_angle_calc(motor->estimator.angle, 7);
    }
    
    // 根据控制模式执行相应的控制算法
    switch (motor->controller) {
        case FOC_CONTROL_OPENLOOP:
            // 开环模式：直接输出电压
            motor->Ud = 0.0f;
            motor->Uq = motor->target;
            break;
            
        case FOC_CONTROL_CURRENT:
            // 电流模式：使用电流PID控制
            // 这里简化处理，实际应用中需要电流反馈
            motor->Id = pid_compute(&motor->current_d_pid, 0.0f, dt);  // 目标电流与反馈电流的误差
            motor->Iq = pid_compute(&motor->current_q_pid, 0.0f, dt);
            motor->Ud = motor->Id;
            motor->Uq = motor->Iq;
            break;
            
        case FOC_CONTROL_VELOCITY:
            // 速度模式：使用速度PID控制
            {
                float velocity_error = motor->target_velocity - motor->estimator.velocity;
                motor->Iq = pid_compute(&motor->velocity_pid, velocity_error, dt);
                motor->Id = 0.0f;  // D轴电流通常设为0
                motor->Uq = motor->Iq;
                motor->Ud = motor->Id;
            }
            break;
            
        case FOC_CONTROL_ANGLE:
            // 角度模式：使用角度PID控制
            {
                float angle_error = motor->target - motor->estimator.angle;
                float velocity_target = pid_compute(&motor->angle_pid, angle_error, dt);
                float velocity_error = velocity_target - motor->estimator.velocity;
                motor->Iq = pid_compute(&motor->velocity_pid, velocity_error, dt);
                motor->Id = 0.0f;
                motor->Uq = motor->Iq;
                motor->Ud = motor->Id;
            }
            break;
            
        case FOC_CONTROL_DISABLED:
        default:
            // 禁用模式：不输出
            motor->Ud = 0.0f;
            motor->Uq = 0.0f;
            break;
    }
    
    // 反帕克变换：从DQ轴转换到阿尔法贝塔轴
    float u_alpha, u_beta;
    inverse_park_transform(motor->Ud, motor->Uq, motor->angle_el, &u_alpha, &u_beta);
    
    // SVPWM计算：从阿尔法贝塔轴转换到三相PWM
    svpwm_compute(u_alpha, u_beta, motor->driver.voltage_limit, dt, 
                  &motor->Ua, &motor->Ub, &motor->Uc);
    
    return 0;
}
