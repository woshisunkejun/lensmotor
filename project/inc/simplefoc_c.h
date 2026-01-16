/**
 * @file    simplefoc_c.h
 * @author  SimpleFOC C语言实现
 * @date    2026-01-09
 * @brief   SimpleFOC库的C语言实现，用于FOC（磁场定向控制）无刷电机控制
 */

#ifndef __SIMPLEFOC_C_H
#define __SIMPLEFOC_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* PID控制器结构体 */
typedef struct {
    float kp;           /*!< 比例增益 */
    float ki;           /*!< 积分增益 */
    float kd;           /*!< 微分增益 */
    float output_ramp;  /*!< 输出斜坡限制 */
    float limit;        /*!< 输出限幅 */
    
    float error_prev;   /*!< 上一次误差 */
    float integral;     /*!< 积分项 */
    float derivative;   /*!< 微分项 */
} pid_controller_t;

/* 低通滤波器结构体 */
typedef struct {
    float Tf;           /*!< 时间常数 */
    float y_prev;       /*!< 上一次输出 */
    float x_prev;       /*!< 上一次输入 */
} low_pass_filter_t;

/* 位置/速度估算器结构体 */
typedef struct {
    float angle_prev;   /*!< 上一次角度 */
    float velocity;     /*!< 当前速度 */
    float angle;        /*!< 当前角度 */
} estimator_t;

/* 电机驱动器结构体 */
typedef struct {
    float voltage_power_supply;     /*!< 电源电压 */
    float voltage_limit;            /*!< 电压限制 */
    float pwm_frequency;            /*!< PWM频率 */
    uint8_t pwm_a;                  /*!< A相PWM引脚 */
    uint8_t pwm_b;                  /*!< B相PWM引脚 */
    uint8_t pwm_c;                  /*!< C相PWM引脚 */
} driver_t;

/* 电机状态枚举 */
typedef enum {
    MOTOR_STATUS_STOPPED = 0,
    MOTOR_STATUS_STARTING,
    MOTOR_STATUS_RUNNING,
    MOTOR_STATUS_ERROR
} motor_status_e;

/* 控制模式枚举 */
typedef enum {
    FOC_CONTROL_DISABLED = 0,
    FOC_CONTROL_OPENLOOP,
    FOC_CONTROL_ANGLE,
    FOC_CONTROL_VELOCITY,
    FOC_CONTROL_CURRENT
} control_mode_e;

/* 传感器类型枚举 */
typedef enum {
    SENSOR_TYPE_NONE = 0,
    SENSOR_TYPE_ENCODER,
    SENSOR_TYPE_HALL,
    SENSOR_TYPE_SINE_COSINE
} sensor_type_e;

/* 电机类型枚举 */
typedef enum {
    MOTOR_TYPE_BLDC = 0,
    MOTOR_TYPE_PMSM,
    MOTOR_TYPE_DC
} motor_type_e;

/* 电机对象结构体 */
typedef struct {
    // 电机参数
    motor_type_e motor_type;              /*!< 电机类型 */
    uint8_t pole_pairs;                   /*!< 极对数 */
    float phase_resistance;               /*!< 相电阻 */
    float kv_rating;                      /*!< 电机KV值 */
    
    // 控制参数
    control_mode_e controller;            /*!< 控制器类型 */
    float velocity_limit;                 /*!< 速度限制 */
    float voltage_limit;                  /*!< 电压限制 */
    float current_limit;                  /*!< 电流限制 */
    
    // PID控制器
    pid_controller_t velocity_pid;        /*!< 速度PID控制器 */
    pid_controller_t angle_pid;           /*!< 角度PID控制器 */
    pid_controller_t current_q_pid;       /*!< Q轴电流PID控制器 */
    pid_controller_t current_d_pid;       /*!< D轴电流PID控制器 */
    
    // 传感器数据
    sensor_type_e sensor_type;            /*!< 传感器类型 */
    bool use_sensor;                      /*!< 是否使用传感器 */
    float hall_angle;                     /*!< 霍尔传感器角度 */
    float encoder_angle;                  /*!< 编码器角度 */
    
    // 估计器
    estimator_t estimator;                /*!< 位置/速度估计器 */
    
    // 滤波器
    low_pass_filter_t velocity_filter;    /*!< 速度滤波器 */
    low_pass_filter_t angle_filter;       /*!< 角度滤波器 */
    
    // 控制输出
    float Ua, Ub, Uc;                     /*!< 三相电压输出 */
    float Ud, Uq;                         /*!< DQ轴电压输出 */
    
    // 电流检测
    float current_a;                      /*!< A相电流 */
    float current_b;                      /*!< B相电流 */
    float current_c;                      /*!< C相电流 */
    float Id, Iq;                         /*!< DQ轴电流 */
    
    // 状态
    motor_status_e status;                /*!< 电机状态 */
    float target;                         /*!< 目标值 */
    float target_velocity;                /*!< 目标速度 */
    
    // 驱动器
    driver_t driver;                      /*!< 驱动器参数 */
    
    // 其他参数
    float dt;                             /*!< 时间步长 */
    float angle_el;                       /*!< 电角度 */
} foc_motor_t;

/* 函数声明 */

/**
 * @brief 磁场定向控制初始化函数
 * @param motor 电机结构体指针
 * @return 0表示成功，负数表示错误
 */
int foc_init(foc_motor_t *motor);

/**
 * @brief 设置电机参数
 * @param motor 电机结构体指针
 * @param phase_resistance 相电阻
 * @param kv_rating KV值
 * @param pwm_frequency PWM频率
 * @return 0表示成功，负数表示错误
 */
int foc_set_motor_params(foc_motor_t *motor, float phase_resistance, float kv_rating, float pwm_frequency);

/**
 * @brief 设置PID参数
 * @param motor 电机结构体指针
 * @param pid_type PID类型 (0=angle, 1=velocity, 2=current_q, 3=current_d)
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 0表示成功，负数表示错误
 */
int foc_set_pid_constants(foc_motor_t *motor, uint8_t pid_type, float kp, float ki, float kd);

/**
 * @brief 设置控制模式
 * @param motor 电机结构体指针
 * @param mode 控制模式
 * @return 0表示成功，负数表示错误
 */
int foc_set_control_mode(foc_motor_t *motor, control_mode_e mode);

/**
 * @brief 设置目标值
 * @param motor 电机结构体指针
 * @param target 目标值
 * @return 0表示成功，负数表示错误
 */
int foc_set_target(foc_motor_t *motor, float target);

/**
 * @brief 主控制循环
 * @param motor 电机结构体指针
 * @param sensor_angle 传感器角度
 * @param dt 时间步长
 * @return 0表示成功，负数表示错误
 */
int foc_control_cycle(foc_motor_t *motor, float sensor_angle, float dt);

/**
 * @brief 启动电机
 * @param motor 电机结构体指针
 * @return 0表示成功，负数表示错误
 */
int foc_start(foc_motor_t *motor);

/**
 * @brief 停止电机
 * @param motor 电机结构体指针
 * @return 0表示成功，负数表示错误
 */
int foc_stop(foc_motor_t *motor);

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
                          uint8_t pwm_a, uint8_t pwm_b, uint8_t pwm_c);

/**
 * @brief PID控制器初始化
 * @param pid PID结构体指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param limit 输出限幅
 * @param output_ramp 输出斜坡
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float limit, float output_ramp);

/**
 * @brief PID控制器计算
 * @param pid PID结构体指针
 * @param error 误差
 * @param dt 时间步长
 * @return PID输出
 */
float pid_compute(pid_controller_t *pid, float error, float dt);

/**
 * @brief 低通滤波器初始化
 * @param lpf 低通滤波器结构体指针
 * @param Tf 时间常数
 */
void lpf_init(low_pass_filter_t *lpf, float Tf);

/**
 * @brief 低通滤波器计算
 * @param lpf 低通滤波器结构体指针
 * @param input 输入值
 * @param dt 时间步长
 * @return 滤波后输出
 */
float lpf_compute(low_pass_filter_t *lpf, float input, float dt);

/**
 * @brief 克拉克变换 (3相 -> 2相静止坐标系)
 * @param a A相输入
 * @param b B相输入
 * @param c C相输入
 * @param alpha 阿尔法轴输出
 * @param beta 贝塔轴输出
 */
void clarke_transform(float a, float b, float c, float *alpha, float *beta);

/**
 * @brief 帕克变换 (2相静止 -> 2相旋转坐标系)
 * @param alpha 阿尔法轴输入
 * @param beta 贝塔轴输入
 * @param angle 电角度
 * @param d D轴输出
 * @param q Q轴输出
 */
void park_transform(float alpha, float beta, float angle, float *d, float *q);

/**
 * @brief 反帕克变换 (2相旋转 -> 2相静止坐标系)
 * @param d D轴输入
 * @param q Q轴输入
 * @param angle 电角度
 * @param alpha 阿尔法轴输出
 * @param beta 贝塔轴输出
 */
void inverse_park_transform(float d, float q, float angle, float *alpha, float *beta);

/**
 * @brief 反克拉克变换 (2相静止 -> 3相)
 * @param alpha 阿尔法轴输入
 * @param beta 贝塔轴输入
 * @param a A相输出
 * @param b B相输出
 * @param c C相输出
 */
void inverse_clarke_transform(float alpha, float beta, float *a, float *b, float *c);

/**
 * @brief SVPWM (空间矢量脉宽调制) 计算
 * @param u_alpha 阿尔法轴电压
 * @param u_beta 贝塔轴电压
 * @param voltage_limit 电压限制
 * @param dt 时间步长
 * @param pwm_a A相PWM占空比输出
 * @param pwm_b B相PWM占空比输出
 * @param pwm_c C相PWM占空比输出
 */
void svpwm_compute(float u_alpha, float u_beta, float voltage_limit, float dt, 
                   float *pwm_a, float *pwm_b, float *pwm_c);

/**
 * @brief 估算电角度
 * @param mechanical_angle 机械角度
 * @param pole_pairs 极对数
 * @return 电角度
 */
float electrical_angle_calc(float mechanical_angle, int pole_pairs);

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLEFOC_C_H */