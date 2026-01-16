#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@file    motor_foc_gui.py
@author  FOC电机调试上位机
@date    2026-01-12
@brief   基于PyQt5的FOC电机调试上位机软件
"""

import sys
import struct
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QTabWidget, QLabel, QComboBox, 
                            QPushButton, QLineEdit, QGroupBox, QGridLayout,
                            QCheckBox, QSlider, QSpinBox, QDoubleSpinBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import time

# 通信协议定义
FRAME_START = 0xAA
FRAME_END = 0x55

# 命令/类型定义
MSG_MOTOR_STATUS = 0x01    # 电机状态数据
MSG_PID_PARAMS = 0x02      # PID参数数据
MSG_SYSTEM_STATUS = 0x03   # 系统状态数据

CMD_SET_PID = 0x10         # 设置PID参数
CMD_SET_MODE = 0x11        # 设置控制模式
CMD_SET_TARGET = 0x12      # 设置目标值
CMD_MOTOR_CTRL = 0x13      # 电机控制 (启动/停止)
CMD_REQUEST_DATA = 0x14    # 请求数据

# PID类型定义
PID_TYPE_POSITION = 0x00   # 位置环PID
PID_TYPE_VELOCITY = 0x01   # 速度环PID
PID_TYPE_CURRENT = 0x02    # 电流环PID

# 控制模式定义
MODE_DISABLED = 0x00       # 禁用
MODE_OPENLOOP = 0x01       # 开环
MODE_ANGLE = 0x02          # 角度控制
MODE_VELOCITY = 0x03       # 速度控制
MODE_CURRENT = 0x04        # 电流控制

# 电机动作定义
MOTOR_ACTION_STOP = 0x00   # 停止
MOTOR_ACTION_START = 0x01  # 启动

# 数据请求类型
REQ_MOTOR_STATUS = 0x01    # 请求电机状态
REQ_PID_PARAMS = 0x02      # 请求PID参数
REQ_SYSTEM_STATUS = 0x03   # 请求系统状态

# 电机状态定义
MOTOR_STATUS_STOPPED = 0x00   # 停止
MOTOR_STATUS_STARTING = 0x01  # 启动中
MOTOR_STATUS_RUNNING = 0x02   # 运行
MOTOR_STATUS_ERROR = 0x03     # 错误

class SerialThread(QThread):
    """串口通信线程"""
    data_received = pyqtSignal(dict)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.buffer = bytearray()
    
    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            while self.running:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    self.buffer.extend(data)
                    self.process_buffer()
                time.sleep(0.01)
        except Exception as e:
            self.error_occurred.emit(f"串口错误: {str(e)}")
            self.running = False
    
    def process_buffer(self):
        """处理接收到的缓冲区数据"""
        while len(self.buffer) >= 7:
            # 查找帧起始
            start_idx = self.buffer.find(FRAME_START)
            if start_idx == -1:
                self.buffer.clear()
                return
            
            # 移除起始前的数据
            if start_idx > 0:
                self.buffer = self.buffer[start_idx:]
            
            # 检查帧长度
            if len(self.buffer) < 7:
                return
            
            # 解析帧头
            frame_type = self.buffer[1]
            motor_index = self.buffer[2]
            data_length = (self.buffer[3] << 8) | self.buffer[4]
            
            # 检查完整帧
            total_length = 7 + data_length
            if len(self.buffer) < total_length:
                return
            
            # 提取帧
            frame = self.buffer[:total_length]
            self.buffer = self.buffer[total_length:]
            
            # 验证校验和
            checksum = self.calculate_checksum(frame[1:5+data_length])
            if checksum != frame[5+data_length] or frame[-1] != FRAME_END:
                continue
            
            # 解析数据
            self.parse_frame(frame_type, motor_index, frame[5:5+data_length])
    
    def parse_frame(self, frame_type, motor_index, data):
        """解析接收到的帧数据"""
        if frame_type == MSG_MOTOR_STATUS:
            if len(data) >= 25:
                angle = self.bytes_to_float(data[0:4])
                velocity = self.bytes_to_float(data[4:8])
                current_q = self.bytes_to_float(data[8:12])
                current_d = self.bytes_to_float(data[12:16])
                voltage_q = self.bytes_to_float(data[16:20])
                voltage_d = self.bytes_to_float(data[20:24])
                status = data[24]
                
                status_str = ["停止", "启动中", "运行", "错误"][status] if status < 4 else "未知"
                
                self.data_received.emit({
                    'type': 'motor_status',
                    'index': motor_index,
                    'angle': angle,
                    'velocity': velocity,
                    'current_q': current_q,
                    'current_d': current_d,
                    'voltage_q': voltage_q,
                    'voltage_d': voltage_d,
                    'status': status_str
                })
        
        elif frame_type == MSG_PID_PARAMS:
            if len(data) >= 36:
                pos_kp = self.bytes_to_float(data[0:4])
                pos_ki = self.bytes_to_float(data[4:8])
                pos_kd = self.bytes_to_float(data[8:12])
                vel_kp = self.bytes_to_float(data[12:16])
                vel_ki = self.bytes_to_float(data[16:20])
                vel_kd = self.bytes_to_float(data[20:24])
                cur_kp = self.bytes_to_float(data[24:28])
                cur_ki = self.bytes_to_float(data[28:32])
                cur_kd = self.bytes_to_float(data[32:36])
                
                self.data_received.emit({
                    'type': 'pid_params',
                    'index': motor_index,
                    'position': {'kp': pos_kp, 'ki': pos_ki, 'kd': pos_kd},
                    'velocity': {'kp': vel_kp, 'ki': vel_ki, 'kd': vel_kd},
                    'current': {'kp': cur_kp, 'ki': cur_ki, 'kd': cur_kd}
                })
        
        elif frame_type == MSG_SYSTEM_STATUS:
            if len(data) >= 10:
                temperature = self.bytes_to_float(data[0:4])
                error_code = (data[4] << 8) | data[5]
                uptime = (data[6] << 24) | (data[7] << 16) | (data[8] << 8) | data[9]
                
                self.data_received.emit({
                    'type': 'system_status',
                    'temperature': temperature,
                    'error_code': error_code,
                    'uptime': uptime
                })
    
    def send_frame(self, frame_type, motor_index, data):
        """发送帧数据"""
        if not self.serial or not self.serial.is_open:
            return
        
        # 构建帧
        frame = bytearray()
        frame.append(FRAME_START)
        frame.append(frame_type)
        frame.append(motor_index)
        frame.append((len(data) >> 8) & 0xFF)
        frame.append(len(data) & 0xFF)
        frame.extend(data)
        
        # 计算校验和
        checksum = self.calculate_checksum(frame[1:])
        frame.append(checksum)
        frame.append(FRAME_END)
        
        # 发送
        try:
            self.serial.write(frame)
        except Exception as e:
            self.error_occurred.emit(f"发送错误: {str(e)}")
    
    def calculate_checksum(self, data):
        """计算校验和"""
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum
    
    def float_to_bytes(self, f):
        """将浮点数转换为字节数组"""
        return struct.pack('>f', f)
    
    def bytes_to_float(self, b):
        """将字节数组转换为浮点数"""
        return struct.unpack('>f', b)[0]
    
    def stop(self):
        """停止线程"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.wait()

class MotorFOCDebugger(QMainWindow):
    """FOC电机调试上位机主窗口"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC电机调试上位机")
        self.setGeometry(100, 100, 1200, 800)
        
        # 初始化变量
        self.serial_thread = None
        self.current_motor = 0
        self.data_history = {
            'angle': [],
            'velocity': [],
            'current_q': [],
            'current_d': [],
            'voltage_q': [],
            'voltage_d': []
        }
        self.data_buffer_size = 200
        
        # 创建主界面
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)
        
        # 创建顶部控制面板
        self.create_control_panel()
        
        # 创建标签页
        self.tab_widget = QTabWidget()
        self.main_layout.addWidget(self.tab_widget)
        
        # 创建数据监控标签页
        self.create_monitor_tab()
        
        # 创建参数设置标签页
        self.create_params_tab()
        
        # 创建系统状态标签页
        self.create_system_tab()
        
        # 创建定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(50)  # 20Hz更新
        
        # 初始化串口
        self.update_com_ports()
    
    def create_control_panel(self):
        """创建顶部控制面板"""
        control_panel = QGroupBox("控制面板")
        control_layout = QHBoxLayout()
        
        # 串口选择
        com_layout = QHBoxLayout()
        com_layout.addWidget(QLabel("串口:"))
        self.com_combo = QComboBox()
        com_layout.addWidget(self.com_combo)
        self.refresh_com_btn = QPushButton("刷新")
        self.refresh_com_btn.clicked.connect(self.update_com_ports)
        com_layout.addWidget(self.refresh_com_btn)
        control_layout.addLayout(com_layout)
        
        # 连接按钮
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        control_layout.addWidget(self.connect_btn)
        
        # 电机选择
        motor_layout = QHBoxLayout()
        motor_layout.addWidget(QLabel("电机:"))
        self.motor_combo = QComboBox()
        for i in range(3):
            self.motor_combo.addItem(f"电机 {i}")
        self.motor_combo.currentIndexChanged.connect(self.on_motor_changed)
        motor_layout.addWidget(self.motor_combo)
        control_layout.addLayout(motor_layout)
        
        # 启动/停止按钮
        self.start_btn = QPushButton("启动")
        self.start_btn.clicked.connect(self.start_motor)
        control_layout.addWidget(self.start_btn)
        self.stop_btn = QPushButton("停止")
        self.stop_btn.clicked.connect(self.stop_motor)
        control_layout.addWidget(self.stop_btn)
        
        control_panel.setLayout(control_layout)
        self.main_layout.addWidget(control_panel)
    
    def create_monitor_tab(self):
        """创建数据监控标签页"""
        monitor_tab = QWidget()
        monitor_layout = QVBoxLayout(monitor_tab)
        
        # 创建图形区域
        self.plot_widget = pg.GraphicsLayoutWidget()
        monitor_layout.addWidget(self.plot_widget)
        
        # 创建多个绘图
        self.plots = {}
        
        # 角度和速度
        self.plots['angle'] = self.plot_widget.addPlot(title="角度 (rad)", row=0, col=0)
        self.plots['velocity'] = self.plot_widget.addPlot(title="速度 (rad/s)", row=1, col=0)
        
        # 电流
        self.plots['current'] = self.plot_widget.addPlot(title="电流 (A)", row=0, col=1)
        self.plots['current'].addLegend()
        
        # 电压
        self.plots['voltage'] = self.plot_widget.addPlot(title="电压 (V)", row=1, col=1)
        self.plots['voltage'].addLegend()
        
        # 配置曲线
        self.curves = {}
        self.curves['angle'] = self.plots['angle'].plot(pen=pg.mkPen('r', width=2))
        self.curves['velocity'] = self.plots['velocity'].plot(pen=pg.mkPen('g', width=2))
        self.curves['current_q'] = self.plots['current'].plot(pen=pg.mkPen('b', width=2), name='Iq')
        self.curves['current_d'] = self.plots['current'].plot(pen=pg.mkPen('y', width=2), name='Id')
        self.curves['voltage_q'] = self.plots['voltage'].plot(pen=pg.mkPen('m', width=2), name='Uq')
        self.curves['voltage_d'] = self.plots['voltage'].plot(pen=pg.mkPen('c', width=2), name='Ud')
        
        # 添加数据显示
        data_layout = QGridLayout()
        self.data_labels = {}
        
        labels = [
            ('角度', 'angle', 'rad'),
            ('速度', 'velocity', 'rad/s'),
            ('Q轴电流', 'current_q', 'A'),
            ('D轴电流', 'current_d', 'A'),
            ('Q轴电压', 'voltage_q', 'V'),
            ('D轴电压', 'voltage_d', 'V'),
            ('状态', 'status', '')
        ]
        
        for i, (name, key, unit) in enumerate(labels):
            data_layout.addWidget(QLabel(name), i//3, (i%3)*3)
            label = QLabel("0.0")
            label.setMinimumWidth(100)
            label.setAlignment(Qt.AlignRight)
            data_layout.addWidget(label, i//3, (i%3)*3 + 1)
            data_layout.addWidget(QLabel(unit), i//3, (i%3)*3 + 2)
            self.data_labels[key] = label
        
        monitor_layout.addLayout(data_layout)
        self.tab_widget.addTab(monitor_tab, "数据监控")
    
    def create_params_tab(self):
        """创建参数设置标签页"""
        params_tab = QWidget()
        params_layout = QVBoxLayout(params_tab)
        
        # 控制模式设置
        mode_group = QGroupBox("控制模式")
        mode_layout = QHBoxLayout()
        self.mode_buttons = {}
        
        modes = [
            ("禁用", MODE_DISABLED),
            ("开环", MODE_OPENLOOP),
            ("角度控制", MODE_ANGLE),
            ("速度控制", MODE_VELOCITY),
            ("电流控制", MODE_CURRENT)
        ]
        
        for name, value in modes:
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, v=value: self.set_control_mode(v))
            mode_layout.addWidget(btn)
            self.mode_buttons[value] = btn
        
        mode_group.setLayout(mode_layout)
        params_layout.addWidget(mode_group)
        
        # 目标值设置
        target_group = QGroupBox("目标值")
        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("目标值:"))
        self.target_spin = QDoubleSpinBox()
        self.target_spin.setRange(-1000, 1000)
        self.target_spin.setSingleStep(0.1)
        target_layout.addWidget(self.target_spin)
        self.set_target_btn = QPushButton("设置")
        self.set_target_btn.clicked.connect(self.set_target)
        target_layout.addWidget(self.set_target_btn)
        target_group.setLayout(target_layout)
        params_layout.addWidget(target_group)
        
        # PID参数设置
        pid_group = QGroupBox("PID参数")
        pid_layout = QVBoxLayout()
        
        # PID类型选择
        pid_type_layout = QHBoxLayout()
        pid_type_layout.addWidget(QLabel("PID类型:"))
        self.pid_type_combo = QComboBox()
        self.pid_type_combo.addItems(["位置环", "速度环", "电流环"])
        self.pid_type_combo.currentIndexChanged.connect(self.on_pid_type_changed)
        pid_type_layout.addWidget(self.pid_type_combo)
        pid_layout.addLayout(pid_type_layout)
        
        # PID参数
        pid_params_layout = QGridLayout()
        
        # KP
        pid_params_layout.addWidget(QLabel("KP:"), 0, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0, 100)
        self.kp_spin.setSingleStep(0.1)
        pid_params_layout.addWidget(self.kp_spin, 0, 1)
        
        # KI
        pid_params_layout.addWidget(QLabel("KI:"), 1, 0)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0, 10)
        self.ki_spin.setSingleStep(0.01)
        pid_params_layout.addWidget(self.ki_spin, 1, 1)
        
        # KD
        pid_params_layout.addWidget(QLabel("KD:"), 2, 0)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0, 1)
        self.kd_spin.setSingleStep(0.001)
        pid_params_layout.addWidget(self.kd_spin, 2, 1)
        
        # 设置按钮
        self.set_pid_btn = QPushButton("设置PID参数")
        self.set_pid_btn.clicked.connect(self.set_pid_params)
        pid_params_layout.addWidget(self.set_pid_btn, 3, 0, 1, 2)
        
        pid_layout.addLayout(pid_params_layout)
        pid_group.setLayout(pid_layout)
        params_layout.addWidget(pid_group)
        
        # 请求参数按钮
        self.request_params_btn = QPushButton("请求参数")
        self.request_params_btn.clicked.connect(self.request_pid_params)
        params_layout.addWidget(self.request_params_btn)
        
        self.tab_widget.addTab(params_tab, "参数设置")
    
    def create_system_tab(self):
        """创建系统状态标签页"""
        system_tab = QWidget()
        system_layout = QVBoxLayout(system_tab)
        
        # 系统状态显示
        status_layout = QGridLayout()
        
        self.system_labels = {
            'temperature': QLabel("0.0 °C"),
            'error_code': QLabel("0"),
            'uptime': QLabel("0 s")
        }
        
        status_layout.addWidget(QLabel("温度:"), 0, 0)
        status_layout.addWidget(self.system_labels['temperature'], 0, 1)
        status_layout.addWidget(QLabel("错误代码:"), 1, 0)
        status_layout.addWidget(self.system_labels['error_code'], 1, 1)
        status_layout.addWidget(QLabel("运行时间:"), 2, 0)
        status_layout.addWidget(self.system_labels['uptime'], 2, 1)
        
        system_layout.addLayout(status_layout)
        
        # 请求系统状态按钮
        self.request_system_btn = QPushButton("请求系统状态")
        self.request_system_btn.clicked.connect(self.request_system_status)
        system_layout.addWidget(self.request_system_btn)
        
        # 日志显示
        log_group = QGroupBox("通信日志")
        log_layout = QVBoxLayout()
        self.log_text = QLineEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(100)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        system_layout.addWidget(log_group)
        
        self.tab_widget.addTab(system_tab, "系统状态")
    
    def update_com_ports(self):
        """更新可用串口列表"""
        ports = serial.tools.list_ports.comports()
        self.com_combo.clear()
        for port in ports:
            self.com_combo.addItem(f"{port.device} - {port.description}")
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.serial_thread and self.serial_thread.isRunning():
            # 断开连接
            self.serial_thread.stop()
            self.connect_btn.setText("连接")
            self.log("已断开串口连接")
        else:
            # 连接
            if self.com_combo.currentIndex() == -1:
                self.log("请选择串口")
                return
            
            port = self.com_combo.currentText().split(' ')[0]
            self.serial_thread = SerialThread(port)
            self.serial_thread.data_received.connect(self.handle_data)
            self.serial_thread.error_occurred.connect(self.log)
            self.serial_thread.start()
            self.connect_btn.setText("断开")
            self.log(f"已连接到 {port}")
    
    def on_motor_changed(self, index):
        """电机选择变化"""
        self.current_motor = index
        self.log(f"切换到电机 {index}")
    
    def start_motor(self):
        """启动电机"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        # 发送启动命令
        data = bytearray([MOTOR_ACTION_START])
        self.serial_thread.send_frame(CMD_MOTOR_CTRL, self.current_motor, data)
        self.log(f"启动电机 {self.current_motor}")
    
    def stop_motor(self):
        """停止电机"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        # 发送停止命令
        data = bytearray([MOTOR_ACTION_STOP])
        self.serial_thread.send_frame(CMD_MOTOR_CTRL, self.current_motor, data)
        self.log(f"停止电机 {self.current_motor}")
    
    def set_control_mode(self, mode):
        """设置控制模式"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        # 发送设置模式命令
        data = bytearray([mode])
        self.serial_thread.send_frame(CMD_SET_MODE, self.current_motor, data)
        
        # 更新按钮状态
        for btn_mode, btn in self.mode_buttons.items():
            btn.setChecked(btn_mode == mode)
        
        mode_names = ["禁用", "开环", "角度控制", "速度控制", "电流控制"]
        if mode < len(mode_names):
            self.log(f"设置电机 {self.current_motor} 模式为 {mode_names[mode]}")
    
    def set_target(self):
        """设置目标值"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        target = self.target_spin.value()
        data = self.serial_thread.float_to_bytes(target)
        self.serial_thread.send_frame(CMD_SET_TARGET, self.current_motor, data)
        self.log(f"设置电机 {self.current_motor} 目标值为 {target}")
    
    def on_pid_type_changed(self, index):
        """PID类型变化"""
        pid_types = ["位置环", "速度环", "电流环"]
        self.log(f"切换到 {pid_types[index]} PID")
    
    def set_pid_params(self):
        """设置PID参数"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        pid_type = self.pid_type_combo.currentIndex()
        kp = self.kp_spin.value()
        ki = self.ki_spin.value()
        kd = self.kd_spin.value()
        
        # 构建数据
        data = bytearray([pid_type])
        data.extend(self.serial_thread.float_to_bytes(kp))
        data.extend(self.serial_thread.float_to_bytes(ki))
        data.extend(self.serial_thread.float_to_bytes(kd))
        
        # 发送命令
        self.serial_thread.send_frame(CMD_SET_PID, self.current_motor, data)
        
        pid_types = ["位置环", "速度环", "电流环"]
        self.log(f"设置电机 {self.current_motor} {pid_types[pid_type]} PID: KP={kp}, KI={ki}, KD={kd}")
    
    def request_motor_status(self):
        """请求电机状态"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            return
        
        data = bytearray([REQ_MOTOR_STATUS])
        self.serial_thread.send_frame(CMD_REQUEST_DATA, self.current_motor, data)
    
    def request_pid_params(self):
        """请求PID参数"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        data = bytearray([REQ_PID_PARAMS])
        self.serial_thread.send_frame(CMD_REQUEST_DATA, self.current_motor, data)
        self.log(f"请求电机 {self.current_motor} PID参数")
    
    def request_system_status(self):
        """请求系统状态"""
        if not self.serial_thread or not self.serial_thread.isRunning():
            self.log("请先连接串口")
            return
        
        data = bytearray([REQ_SYSTEM_STATUS])
        self.serial_thread.send_frame(CMD_REQUEST_DATA, 0, data)
        self.log("请求系统状态")
    
    def handle_data(self, data):
        """处理接收到的数据"""
        data_type = data.get('type')
        
        if data_type == 'motor_status':
            index = data.get('index')
            if index == self.current_motor:
                # 更新显示
                self.data_labels['angle'].setText(f"{data.get('angle', 0):.2f}")
                self.data_labels['velocity'].setText(f"{data.get('velocity', 0):.2f}")
                self.data_labels['current_q'].setText(f"{data.get('current_q', 0):.2f}")
                self.data_labels['current_d'].setText(f"{data.get('current_d', 0):.2f}")
                self.data_labels['voltage_q'].setText(f"{data.get('voltage_q', 0):.2f}")
                self.data_labels['voltage_d'].setText(f"{data.get('voltage_d', 0):.2f}")
                self.data_labels['status'].setText(data.get('status', '未知'))
                
                # 更新历史数据
                for key in ['angle', 'velocity', 'current_q', 'current_d', 'voltage_q', 'voltage_d']:
                    self.data_history[key].append(data.get(key, 0))
                    if len(self.data_history[key]) > self.data_buffer_size:
                        self.data_history[key].pop(0)
        
        elif data_type == 'pid_params':
            index = data.get('index')
            if index == self.current_motor:
                # 更新PID参数显示
                position = data.get('position', {})
                velocity = data.get('velocity', {})
                current = data.get('current', {})
                
                pid_type = self.pid_type_combo.currentIndex()
                if pid_type == PID_TYPE_POSITION:
                    self.kp_spin.setValue(position.get('kp', 0))
                    self.ki_spin.setValue(position.get('ki', 0))
                    self.kd_spin.setValue(position.get('kd', 0))
                elif pid_type == PID_TYPE_VELOCITY:
                    self.kp_spin.setValue(velocity.get('kp', 0))
                    self.ki_spin.setValue(velocity.get('ki', 0))
                    self.kd_spin.setValue(velocity.get('kd', 0))
                elif pid_type == PID_TYPE_CURRENT:
                    self.kp_spin.setValue(current.get('kp', 0))
                    self.ki_spin.setValue(current.get('ki', 0))
                    self.kd_spin.setValue(current.get('kd', 0))
        
        elif data_type == 'system_status':
            # 更新系统状态
            self.system_labels['temperature'].setText(f"{data.get('temperature', 0):.1f} °C")
            self.system_labels['error_code'].setText(str(data.get('error_code', 0)))
            self.system_labels['uptime'].setText(f"{data.get('uptime', 0)} s")
    
    def update_data(self):
        """更新数据显示"""
        # 请求电机状态
        self.request_motor_status()
        
        # 更新图形
        for key, curve in self.curves.items():
            if key in self.data_history:
                curve.setData(self.data_history[key])
    
    def log(self, message):
        """记录日志"""
        timestamp = time.strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}"
        self.log_text.setText(log_message)
        print(log_message)
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        if self.timer:
            self.timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle("Fusion")
    
    # 设置暗色主题
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    # 创建主窗口
    window = MotorFOCDebugger()
    window.show()
    
    sys.exit(app.exec_())