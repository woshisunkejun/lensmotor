/**
 * @file    debug_serial.c
 * @author  串口调试功能
 * @date    2026-01-12
 * @brief   提供基于二进制协议的串口调试功能实现
 */

#include "debug_serial.h"
#include "wk_usart.h"
#include "foc_controller.h"
#include "at32f423.h"
#include <string.h>

/* 全局变量 */
static serial_frame_t rx_frame;
static uint8_t rx_buffer[128];
static uint16_t rx_index = 0;
static bool frame_received = false;

/**
 * @brief 初始化串口调试功能
 */
void debug_serial_init(void)
{
    /* 初始化USART1 */
    wk_usart1_init();
    
    /* 启用USART1接收中断 */
    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
}

/**
 * @brief 发送数据到串口
 * @param data 数据指针
 * @param length 数据长度
 */
static void serial_send(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
        usart_data_transmit(USART1, data[i]);
    }
}

/**
 * @brief 发送完整帧
 * @param frame 帧结构体
 */
static void send_frame(const serial_frame_t *frame)
{
    uint8_t buffer[128];
    uint16_t index = 0;
    
    /* 填充帧数据 */
    buffer[index++] = frame->start;
    buffer[index++] = frame->type;
    buffer[index++] = frame->motor_index;
    buffer[index++] = (frame->data_length >> 8) & 0xFF;
    buffer[index++] = frame->data_length & 0xFF;
    
    /* 填充数据内容 */
    memcpy(&buffer[index], frame->data, frame->data_length);
    index += frame->data_length;
    
    /* 填充校验和 */
    buffer[index++] = frame->checksum;
    buffer[index++] = frame->end;
    
    /* 发送帧 */
    serial_send(buffer, index);
}

/**
 * @brief 发送电机状态数据
 * @param motor_index 电机索引
 * @param status 电机状态数据
 */
void send_motor_status(uint8_t motor_index, const motor_status_data_t *status)
{
    serial_frame_t frame;
    uint8_t data_buffer[25];
    uint16_t data_index = 0;
    
    /* 填充起始和类型 */
    frame.start = FRAME_START;
    frame.type = MSG_MOTOR_STATUS;
    frame.motor_index = motor_index;
    
    /* 转换角度 */
    float_to_bytes(status->angle, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换速度 */
    float_to_bytes(status->velocity, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换Q轴电流 */
    float_to_bytes(status->current_q, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换D轴电流 */
    float_to_bytes(status->current_d, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换Q轴电压 */
    float_to_bytes(status->voltage_q, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换D轴电压 */
    float_to_bytes(status->voltage_d, &data_buffer[data_index]);
    data_index += 4;
    
    /* 添加状态 */
    data_buffer[data_index++] = status->status;
    
    /* 填充数据 */
    frame.data_length = data_index;
    memcpy(frame.data, data_buffer, data_index);
    
    /* 计算校验和 */
    frame.checksum = calculate_checksum((uint8_t*)&frame.type, 4 + frame.data_length);
    frame.end = FRAME_END;
    
    /* 发送帧 */
    send_frame(&frame);
}

/**
 * @brief 发送PID参数数据
 * @param motor_index 电机索引
 * @param params PID参数数据
 */
void send_pid_params(uint8_t motor_index, const pid_params_data_t *params)
{
    serial_frame_t frame;
    uint8_t data_buffer[36];
    uint16_t data_index = 0;
    
    /* 填充起始和类型 */
    frame.start = FRAME_START;
    frame.type = MSG_PID_PARAMS;
    frame.motor_index = motor_index;
    
    /* 转换位置环参数 */
    float_to_bytes(params->pos_kp, &data_buffer[data_index]);
    data_index += 4;
    float_to_bytes(params->pos_ki, &data_buffer[data_index]);
    data_index += 4;
    float_to_bytes(params->pos_kd, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换速度环参数 */
    float_to_bytes(params->vel_kp, &data_buffer[data_index]);
    data_index += 4;
    float_to_bytes(params->vel_ki, &data_buffer[data_index]);
    data_index += 4;
    float_to_bytes(params->vel_kd, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换电流环参数 */
    float_to_bytes(params->cur_kp, &data_buffer[data_index]);
    data_index += 4;
    float_to_bytes(params->cur_ki, &data_buffer[data_index]);
    data_index += 4;
    float_to_bytes(params->cur_kd, &data_buffer[data_index]);
    data_index += 4;
    
    /* 填充数据 */
    frame.data_length = data_index;
    memcpy(frame.data, data_buffer, data_index);
    
    /* 计算校验和 */
    frame.checksum = calculate_checksum((uint8_t*)&frame.type, 4 + frame.data_length);
    frame.end = FRAME_END;
    
    /* 发送帧 */
    send_frame(&frame);
}

/**
 * @brief 发送系统状态数据
 * @param status 系统状态数据
 */
void send_system_status(const system_status_data_t *status)
{
    serial_frame_t frame;
    uint8_t data_buffer[10];
    uint16_t data_index = 0;
    
    /* 填充起始和类型 */
    frame.start = FRAME_START;
    frame.type = MSG_SYSTEM_STATUS;
    frame.motor_index = 0;
    
    /* 转换温度 */
    float_to_bytes(status->temperature, &data_buffer[data_index]);
    data_index += 4;
    
    /* 转换错误代码 */
    data_buffer[data_index++] = (status->error_code >> 8) & 0xFF;
    data_buffer[data_index++] = status->error_code & 0xFF;
    
    /* 转换运行时间 */
    data_buffer[data_index++] = (status->uptime >> 24) & 0xFF;
    data_buffer[data_index++] = (status->uptime >> 16) & 0xFF;
    data_buffer[data_index++] = (status->uptime >> 8) & 0xFF;
    data_buffer[data_index++] = status->uptime & 0xFF;
    
    /* 填充数据 */
    frame.data_length = data_index;
    memcpy(frame.data, data_buffer, data_index);
    
    /* 计算校验和 */
    frame.checksum = calculate_checksum((uint8_t*)&frame.type, 4 + frame.data_length);
    frame.end = FRAME_END;
    
    /* 发送帧 */
    send_frame(&frame);
}

/**
 * @brief 处理接收到的命令
 * @param frame 接收到的帧
 */
void process_serial_command(const serial_frame_t *frame)
{
    switch (frame->type) {
        case CMD_SET_PID: {
            if (frame->data_length >= 13) {
                uint8_t pid_type = frame->data[0];
                float kp = bytes_to_float(&frame->data[1]);
                float ki = bytes_to_float(&frame->data[5]);
                float kd = bytes_to_float(&frame->data[9]);
                
                switch (pid_type) {
                    case PID_TYPE_POSITION:
                        g_motors[frame->motor_index].motor.angle_pid.kp = kp;
                        g_motors[frame->motor_index].motor.angle_pid.ki = ki;
                        g_motors[frame->motor_index].motor.angle_pid.kd = kd;
                        break;
                    case PID_TYPE_VELOCITY:
                        g_motors[frame->motor_index].motor.velocity_pid.kp = kp;
                        g_motors[frame->motor_index].motor.velocity_pid.ki = ki;
                        g_motors[frame->motor_index].motor.velocity_pid.kd = kd;
                        break;
                    case PID_TYPE_CURRENT:
                        g_motors[frame->motor_index].motor.current_q_pid.kp = kp;
                        g_motors[frame->motor_index].motor.current_q_pid.ki = ki;
                        g_motors[frame->motor_index].motor.current_q_pid.kd = kd;
                        g_motors[frame->motor_index].motor.current_d_pid.kp = kp;
                        g_motors[frame->motor_index].motor.current_d_pid.ki = ki;
                        g_motors[frame->motor_index].motor.current_d_pid.kd = kd;
                        break;
                }
            }
            break;
        }
        
        case CMD_SET_MODE: {
            if (frame->data_length >= 1) {
                uint8_t mode = frame->data[0];
                control_mode_e ctrl_mode;
                
                switch (mode) {
                    case MODE_DISABLED:
                        ctrl_mode = FOC_CONTROL_DISABLED;
                        break;
                    case MODE_OPENLOOP:
                        ctrl_mode = FOC_CONTROL_OPENLOOP;
                        break;
                    case MODE_ANGLE:
                        ctrl_mode = FOC_CONTROL_ANGLE;
                        break;
                    case MODE_VELOCITY:
                        ctrl_mode = FOC_CONTROL_VELOCITY;
                        break;
                    case MODE_CURRENT:
                        ctrl_mode = FOC_CONTROL_CURRENT;
                        break;
                    default:
                        ctrl_mode = FOC_CONTROL_DISABLED;
                        break;
                }
                
                foc_controller_set_mode(frame->motor_index, ctrl_mode);
            }
            break;
        }
        
        case CMD_SET_TARGET: {
            if (frame->data_length >= 4) {
                float target = bytes_to_float(frame->data);
                foc_controller_set_target(frame->motor_index, target);
            }
            break;
        }
        
        case CMD_MOTOR_CTRL: {
            if (frame->data_length >= 1) {
                uint8_t action = frame->data[0];
                if (action == MOTOR_ACTION_START) {
                    foc_controller_start(frame->motor_index);
                } else if (action == MOTOR_ACTION_STOP) {
                    foc_controller_stop(frame->motor_index);
                }
            }
            break;
        }
        
        case CMD_REQUEST_DATA: {
            if (frame->data_length >= 1) {
                uint8_t data_type = frame->data[0];
                
                switch (data_type) {
                    case REQ_MOTOR_STATUS: {
                        motor_status_data_t status;
                        status.angle = g_motors[frame->motor_index].motor.estimator.angle;
                        status.velocity = g_motors[frame->motor_index].motor.estimator.velocity;
                        status.current_q = g_motors[frame->motor_index].motor.Iq;
                        status.current_d = g_motors[frame->motor_index].motor.Id;
                        status.voltage_q = g_motors[frame->motor_index].motor.Uq;
                        status.voltage_d = g_motors[frame->motor_index].motor.Ud;
                        status.status = g_motors[frame->motor_index].motor.status;
                        send_motor_status(frame->motor_index, &status);
                        break;
                    }
                    
                    case REQ_PID_PARAMS: {
                        pid_params_data_t params;
                        params.pos_kp = g_motors[frame->motor_index].motor.angle_pid.kp;
                        params.pos_ki = g_motors[frame->motor_index].motor.angle_pid.ki;
                        params.pos_kd = g_motors[frame->motor_index].motor.angle_pid.kd;
                        params.vel_kp = g_motors[frame->motor_index].motor.velocity_pid.kp;
                        params.vel_ki = g_motors[frame->motor_index].motor.velocity_pid.ki;
                        params.vel_kd = g_motors[frame->motor_index].motor.velocity_pid.kd;
                        params.cur_kp = g_motors[frame->motor_index].motor.current_q_pid.kp;
                        params.cur_ki = g_motors[frame->motor_index].motor.current_q_pid.ki;
                        params.cur_kd = g_motors[frame->motor_index].motor.current_q_pid.kd;
                        send_pid_params(frame->motor_index, &params);
                        break;
                    }
                    
                    case REQ_SYSTEM_STATUS: {
                        system_status_data_t sys_status;
                        sys_status.temperature = 45.5f; /* 模拟温度 */
                        sys_status.error_code = 0;
                        sys_status.uptime = 12345; /* 模拟运行时间 */
                        send_system_status(&sys_status);
                        break;
                    }
                }
            }
            break;
        }
    }
}

/**
 * @brief 串口中断处理函数
 */
void serial_rx_handler(void)
{
    if (usart_flag_get(USART1, USART_RDBF_FLAG) != RESET) {
        uint8_t data = usart_data_receive(USART1);
        
        /* 处理接收数据 */
        if (rx_index == 0 && data == FRAME_START) {
            /* 开始接收新帧 */
            rx_buffer[rx_index++] = data;
        } else if (rx_index > 0) {
            /* 继续接收帧数据 */
            rx_buffer[rx_index++] = data;
            
            /* 检查是否接收到完整帧 */
            if (rx_index >= 7) {
                uint16_t data_length = ((uint16_t)rx_buffer[3] << 8) | rx_buffer[4];
                if (rx_index >= (7 + data_length)) {
                    /* 帧接收完成 */
                    memcpy(&rx_frame, rx_buffer, 7 + data_length);
                    frame_received = true;
                    rx_index = 0;
                }
            }
            
            /* 防止缓冲区溢出 */
            if (rx_index >= sizeof(rx_buffer)) {
                rx_index = 0;
            }
        }
    }
}

/**
 * @brief 将浮点数转换为字节数组
 * @param f 浮点数
 * @param bytes 字节数组
 */
void float_to_bytes(float f, uint8_t *bytes)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    
    u.f = f;
    bytes[0] = u.b[3];
    bytes[1] = u.b[2];
    bytes[2] = u.b[1];
    bytes[3] = u.b[0];
}

/**
 * @brief 将字节数组转换为浮点数
 * @param bytes 字节数组
 * @return 浮点数
 */
float bytes_to_float(const uint8_t *bytes)
{
    union {
        float f;
        uint8_t b[4];
    } u;
    
    u.b[3] = bytes[0];
    u.b[2] = bytes[1];
    u.b[1] = bytes[2];
    u.b[0] = bytes[3];
    return u.f;
}

/**
 * @brief 计算校验和
 * @param data 数据
 * @param length 长度
 * @return 校验和
 */
uint8_t calculate_checksum(const uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief 处理接收到的帧
 */
void handle_received_frames(void)
{
    if (frame_received) {
        /* 验证校验和 */
        uint8_t checksum = calculate_checksum((uint8_t*)&rx_frame.type, 4 + rx_frame.data_length);
        if (checksum == rx_frame.checksum && rx_frame.end == FRAME_END) {
            /* 处理命令 */
            process_serial_command(&rx_frame);
        }
        frame_received = false;
    }
}
