# Sensored FOC Library v1.0

A comprehensive Field-Oriented Control (FOC) library designed for sensored permanent magnet synchronous motors (PMSM). This library provides robust and efficient motor control with multiple control modes and advanced features.

## Key Features

### 🎯 **Multiple Control Modes**
- **Open-loop Control**: Position and velocity modes
- **Single-loop Closed-loop**: Position, velocity, and current control
- **Cascade Control**: 
  - Position-Velocity dual-loop
  - Velocity-Current dual-loop  
  - Position-Velocity-Current triple-loop

### ⚡ **Advanced FOC Implementation**
- Space Vector PWM (SVPWM) with 7-segment modulation
- Fast trigonometric approximations for real-time performance
- Inverse Park and Clarke/Park transformations
- Automatic encoder alignment and offset calibration

### 🔧 **Technical Highlights**
- Configurable PID controllers for all control loops
- Kalman filtering for current measurement noise reduction
- Dead-time compensation and safety limits
- Velocity-based angle advance compensation
- Support for different motor parameters and directions

### 📊 **Optimized Performance**
- Efficient polynomial approximations for sin/cos functions
- Low-pass and Kalman filters for signal processing
- Normalized angle handling (0-2π range)
- Modular architecture with clear separation of concerns

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
