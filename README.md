# Sensored FOC Library v1.0

A comprehensive Field-Oriented Control (FOC) library designed for sensored permanent magnet synchronous motors (PMSM). This library provides robust and efficient motor control with multiple control modes and advanced features.

为有传感器的永磁同步电机 (PMSM) 设计的综合场向控制 (FOC) 库。该库提供稳健且高效的电机控制，具有多种控制模式和高级功能。

## Key Features

### 🎯 **Multiple Control Modes**
- **Open-loop Control**: Position and velocity modes
- **Single-loop Closed-loop**: Position, velocity, and current control
- **Cascade Control**: 
  - Position-Velocity dual-loop
  - Velocity-Current dual-loop  
  - Position-Velocity-Current triple-loop
 
- **开环控制**：位置和速度模式
- **单闭环控制**：位置、速度和电流控制
- **级联控制**：
  - 位置-速度双闭环
  - 速度-电流双闭环
  - 位置-速度-电流三闭环

### ⚡ **Advanced FOC Implementation**
- Space Vector PWM (SVPWM) with 7-segment modulation使用七段调制的空间矢量PWM（SVPWM
- Fast trigonometric approximations for real-time performance用于实时性能的快速三角函数近似
- Inverse Park and Clarke/Park transformations反帕克（Inverse Park）和克拉克/帕克（Clarke/Park）变换
- Automatic encoder alignment and offset calibration自动编码器对齐和偏移校准

### 🔧 **Technical Highlights**
- Configurable PID controllers for all control loops控制回路的可配置 PID 控制器
- Kalman filtering for current measurement noise reduction用于电流测量噪声抑制的卡尔曼滤波
- Dead-time compensation and safety limits死区补偿和安全限位
- Velocity-based angle advance compensation基于速度的角度提前补偿
- Support for different motor parameters and directions支持不同的电机参数和方向

### 📊 **Optimized Performance**
- Efficient polynomial approximations for sin/cos functions高效的正弦/余弦函数多项式近似
- Low-pass and Kalman filters for signal processing用于信号处理的低通滤波器和卡尔曼滤波器
- Normalized angle handling (0-2π range)标准化角度处理（0-2π 范围）
- Modular architecture with clear separation of concerns模块化架构，职责清晰分离

## Hardware Requirements
- Magnetic encoder (MT6701 compatible)
- Three-phase PWM capable timer
- Current sensing ADC channels
- Motor driver with dead-time protection

## Getting Started
1. Implement hardware abstraction functions in `SguanUser_data.h`
2. Configure motor parameters (pole pairs, resistance, etc.)
3. Initialize FOC system with `FOC_Init()`
4. Set control mode and targets
5. Call `FOC_LoopHandler()` in your main control loop

## Perfect For
- Robotic joint control
- CNC machines
- Precision motion systems
- DIY motor control projects
- Educational FOC implementations

This library forms a solid foundation for sensored FOC applications and will be extended with sensorless FOC capabilities in future releases.

该库函数为带传感器的FOC（场向控制）应用提供了坚实的基础，并将在未来版本中扩展支持无传感器FOC功能。
