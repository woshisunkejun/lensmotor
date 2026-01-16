/**
 * @file    debug_serial.h
 * @author  串口调试功能
 * @date    2026-01-12
 * @brief   提供基于二进制协议的串口调试功能
 */

#ifndef __DEBUG_SERIAL_H
#define __DEBUG_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "simplefoc_c.h"

/* 通信协议定义 */
#define SERIAL_BAUDRATE        115200
#define FRAME_START            0xAA
#define FRAME_END              0x55

/* 命令/类型定义 */
#define MSG_MOTOR_STATUS       0x01    /* 电机状态数据 */
#define MSG_PID_PARAMS         0x02    /* PID参数数据 */
#define MSG_SYSTEM_STATUS      0x03    /* 系统状态数据 */

#define CMD_SET_PID            0x10    /* 设置PID参数 */
#define CMD_SET_MODE           0x11    /* 设置控制模式 */
#define CMD_SET_TARGET         0x12    /* 设置目标值 */
#define CMD_MOTOR_CTRL         0x13    /* 电机控制 (启动/停止) */
#define CMD_REQUEST_DATA       0x14    /* 请求数据 */

/* PID类型定义 */
#define PID_TYPE_POSITION      0x00    /* 位置环PID */
#define PID_TYPE_VELOCITY      0x01    /* 速度环PID */
#define PID_TYPE_CURRENT       0x02    /* 电流环PID */

/* 控制模式定义 */
#define MODE_DISABLED          0x00    /* 禁用 */
#define MODE_OPENLOOP          0x01    /* 开环 */
#define MODE_ANGLE             0x02    /* 角度控制 */
#define MODE_VELOCITY          0x03    /* 速度控制 */
#define MODE_CURRENT           0x04    /* 电流控制 */

/* 电机动作定义 */
#define MOTOR_ACTION_STOP      0x00    /* 停止 */
#define MOTOR_ACTION_START     0x01    /* 启动 */

/* 数据请求类型 */
#define REQ_MOTOR_STATUS       0x01    /* 请求电机状态 */
#define REQ_PID_PARAMS         0x02    /* 请求PID参数 */
#define REQ_SYSTEM_STATUS      0x03    /* 请求系统状态 */

/* 电机状态定义 */
#define MOTOR_STATUS_STOPPED   0x00    /* 停止 */
#define MOTOR_STATUS_STARTING  0x01    /* 启动中 */
#define MOTOR_STATUS_RUNNING   0x02    /* 运行 */
#define MOTOR_STATUS_ERROR     0x03    /* 错误 */

/* 帧结构体 */
typedef struct {
    uint8_t start;           /* 起始字节 */
    uint8_t type;            /* 命令/类型 */
    uint8_t motor_index;     /* 电机索引 */
    uint16_t data_length;    /* 数据长度 */
    uint8_t data[64];        /* 数据内容 */
    uint8_t checksum;        /* 校验和 */
    uint8_t end;             /* 结束字节 */
} serial_frame_t;

/* 电机状态数据 */
typedef struct {
    float angle;             /* 角度 */
    float velocity;          /* 速度 */
    float current_q;         /* Q轴电流 */
    float current_d;         /* D轴电流 */
    float voltage_q;         /* Q轴电压 */
    float voltage_d;         /* D轴电压 */
    uint8_t status;          /* 状态 */
} motor_status_data_t;

/* PID参数数据 */
typedef struct {
    float pos_kp;            /* 位置环KP */
    float pos_ki;            /* 位置环KI */
    float pos_kd;            /* 位置环KD */
    float vel_kp;            /* 速度环KP */
    float vel_ki;            /* 速度环KI */
    float vel_kd;            /* 速度环KD */
    float cur_kp;            /* 电流环KP */
    float cur_ki;            /* 电流环KI */
    float cur_kd;            /* 电流环KD */
} pid_params_data_t;

/* 系统状态数据 */
typedef struct {
    float temperature;       /* 温度 */
    uint16_t error_code;     /* 错误代码 */
    uint32_t uptime;         /* 运行时间 */
} system_status_data_t;

/* 设置PID参数命令 */
typedef struct {
    uint8_t pid_type;        /* PID类型 */
    float kp;                /* 比例增益 */
    float ki;                /* 积分增益 */
    float kd;                /* 微分增益 */
} set_pid_cmd_t;

/* 设置控制模式命令 */
typedef struct {
    uint8_t mode;            /* 控制模式 */
} set_mode_cmd_t;

/* 设置目标值命令 */
typedef struct {
    float target;            /* 目标值 */
} set_target_cmd_t;

/* 电机控制命令 */
typedef struct {
    uint8_t action;          /* 动作 */
} motor_ctrl_cmd_t;

/* 请求数据命令 */
typedef struct {
    uint8_t data_type;       /* 数据类型 */
} request_data_cmd_t;

/* 函数声明 */

/**
 * @brief 初始化串口调试功能
 */
void debug_serial_init(void);

/**
 * @brief 发送电机状态数据
 * @param motor_index 电机索引
 * @param status 电机状态数据
 */
void send_motor_status(uint8_t motor_index, const motor_status_data_t *status);

/**
 * @brief 发送PID参数数据
 * @param motor_index 电机索引
 * @param params PID参数数据
 */
void send_pid_params(uint8_t motor_index, const pid_params_data_t *params);

/**
 * @brief 发送系统状态数据
 * @param status 系统状态数据
 */
void send_system_status(const system_status_data_t *status);

/**
 * @brief 处理接收到的命令
 * @param frame 接收到的帧
 */
void process_serial_command(const serial_frame_t *frame);

/**
 * @brief 串口中断处理函数
 */
void serial_rx_handler(void);

/**
 * @brief 将浮点数转换为字节数组
 * @param f 浮点数
 * @param bytes 字节数组
 */
void float_to_bytes(float f, uint8_t *bytes);

/**
 * @brief 将字节数组转换为浮点数
 * @param bytes 字节数组
 * @return 浮点数
 */
float bytes_to_float(const uint8_t *bytes);

/**
 * @brief 计算校验和
 * @param data 数据
 * @param length 长度
 * @return 校验和
 */
uint8_t calculate_checksum(const uint8_t *data, uint16_t length);

/**
 * @brief 处理接收到的帧
 */
void handle_received_frames(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_SERIAL_H */
