# Sensored FOC Library v1.0

A comprehensive Field-Oriented Control (FOC) library designed for sensored permanent magnet synchronous motors (PMSM). This library provides robust and efficient motor control with multiple control modes and advanced features.

为有传感器的永磁同步电机 (PMSM) 设计的综合场向控制 (FOC) 库。该库提供稳健且高效的电机控制，具有多种控制模式和高级功能。
[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=flat&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/Sguan-ZhouQing/SguanFOC_Library)

https://zread.ai/Sguan-ZhouQing/SguanFOC_Library

SguanFOC 库是一个专为有感永磁同步电机（PMSM）设计的综合磁场定向控制（FOC）实现。该库为初级开发人员提供了高级电机控制应用的坚实基础，结合了数学精度与实用性。
该库采用模块化架构，将控制算法、硬件抽象和用户配置分离。其核心是，FOC 系统将三相电机控制转换为可管理的 d-q 坐标系操作，实现精确的转矩和位置控制。
该库围绕两个主要数据结构构建，管理所有 FOC 操作：
• SVPWM_HandleTypeDef：管理电压变换和 PWM 生成 SguanFOC.h#L11-L33

• FOC_HandleTypeDef：包含不同控制策略的所有 PID 控制器配置 SguanFOC.h#L35-L113

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
