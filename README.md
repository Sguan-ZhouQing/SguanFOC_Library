# SguanFOC v2.0 - 高性能磁场定向控制库

![Version](https://img.shields.io/badge/Version-2.0-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Language](https://img.shields.io/badge/Language-C-orange)

## 🚀 项目简介

**SguanFOC v2.0** 是一个完全使用**纯C语言**编写的开源磁场定向控制(FOC)库，专为嵌入式微控制器设计。该库提供了完整的FOC算法实现，支持多电机并行控制，具有高性能、易移植、可配置性强等特点。[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=flat&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/Sguan-ZhouQing/SguanFOC_Library)

## ✨ 核心特性

- 🎯 **纯C语言编写** - 无依赖，可快速移植到任何支持C语言的MCU
- 🔧 **多电机支持** - 最多支持4个电机同时运行
- 🎮 **多种控制模式** - 6种控制模式满足不同应用场景
- ⚡ **高性能算法** - 快速数学运算，优化执行效率
- 🔄 **完整FOC流程** - 从电流采样到SVPWM生成的完整链路
- 📊 **可配置参数** - 所有参数集中配置，易于调试

## 📁 项目结构

```
SguanFOC/
├── Sguan_math.c/.h           # 数学运算库
│   ├── 快速sin/cos计算
│   ├── 坐标变换(克拉克/帕克)
│   ├── 滤波器(低通/卡尔曼)
│   └── SVPWM空间矢量调制
├── SguanFOC.c/.h             # 电机控制核心
│   ├── PID控制器
│   ├── 多模式控制
│   ├── 电流/速度/位置环
│   └── FOC主循环
├── SguanUser_Data.h          # 用户接口配置
│   ├── 电机参数配置
│   ├── PID参数设置
│   └── 控制模式选择
└── Sguan_sensorless.c/.h     # 预留无感FOC(v2.1)
```

## 🎯 控制模式

| 模式 | 标识 | 描述 | 应用场景 |
|------|------|------|----------|
| 电流单环 | `Current_SINGLE_MODE` | 直接控制Id/Iq电流 | 力矩控制 |
| 速度单环 | `Velocity_SINGLE_MODE` | 控制电机转速 | 恒速运行 |
| 位置单环 | `Position_SINGLE_MODE` | 控制电机角度 | 定位控制 |
| 速度-电流串级 | `VelCur_DOUBLE_MODE` | 外环速度+内环电流 | 高动态响应 |
| 位置-速度串级 | `PosVel_DOUBLE_MODE` | 外环位置+内环速度 | 精确定位 |
| 三环控制 | `PosVelCur_THREE_MODE` | 位置+速度+电流三环 | 高性能伺服 |

## 🔧 快速开始

### 1. 硬件配置

在 `SguanUser_Data.h` 中启用电机并配置参数：

```c
#define SguanMotor0  // 启用电机0
// #define SguanMotor1  // 注释掉不使用的电机

void Sguan_ParameterSet(void) {
#ifdef SguanMotor0
    /* 1.控制模式 */
    Sguan0.Control_mode = VelCur_DOUBLE_MODE;
    
    /* 2.电机参数 */
    Sguan0.Motor.Polepairs = 7;      // 极对数
    Sguan0.Motor.Dir_n = 0;          // 方向
    
    /* 3.PID参数 */
    Sguan0.Speed.Kp = 0.02226f;      // 速度环P
    Sguan0.Speed.Ki = 0.000115f;     // 速度环I
    
    /* 4.电流采样参数 */
    Sguan0.Current.VCC_Voltage = 3.3f;
    Sguan0.Current.Sampling_resistor = 0.005f;
    Sguan0.Current.Amplifier_Gain = 10;
#endif
}
```

### 2. 实现硬件接口

实现以下硬件相关函数：

```c
// 初始化函数
void SguanUser_Init(void) {
    // 初始化PWM定时器、ADC、编码器等
}

// ADC采样函数
uint32_t SguanUser_ReadADC_Raw(uint8_t Motor_CH, uint8_t Current_CH) {
    // 返回指定通道的ADC原始值
    switch(Motor_CH) {
        case 0: // 电机0
            switch(Current_CH) {
                case 0: return /* IA相电流ADC值 */;
                case 1: return /* IB相电流ADC值 */;
                case 2: return /* IC相电流ADC值 */;
            }
            break;
        // 其他电机...
    }
    return 0;
}

// 编码器读取函数
float SguanUser_Encoder_ReadRad(uint8_t Motor_CH) {
    // 返回指定电机的编码器弧度值
    switch(Motor_CH) {
        case 0: return /* 电机0编码器角度 */;
        case 1: return /* 电机1编码器角度 */;
        // ...
    }
    return 0.0f;
}

// PWM设置函数
void SguanUser_PwmDuty_Set(uint8_t Motor_CH, uint8_t PWM_CH, uint16_t Duty) {
    // 设置指定电机的PWM占空比
    // 根据硬件设置对应的PWM输出
}
```

### 3. 主程序集成

```c
#include "SguanUser_Data.h"

int main(void) {
    // 系统初始化
    System_Init();
    
    // FOC库初始化
    Sguan_FocInit();
    
    while(1) {
        // 主循环任务
        // ...
    }
}

// 1ms定时中断服务函数
void TIM1_IRQHandler(void) {
    if(/* 检查中断标志 */) {
        SguanFOC_Run_Tick();  // FOC控制循环
    }
}

// ADC采样完成中断
void ADC_IRQHandler(void) {
    SguanFOC_GeneratePWM_Loop();  // PWM生成
}
```

## 📊 配置参数详解

### 电机参数
```c
Sguan0.Motor.Polepairs = 7;          // 电机极对数
Sguan0.Motor.Dir_n = 0;              // 机械方向
```

### FOC参数
```c
Sguan0.Foc.Period = 4249;            // PWM周期值
Sguan0.Foc.Dir_m = 0;                // 电气方向
Sguan0.Foc.Response_Num = 5;         // 内外环响应倍数
```

### PID参数配置
```c
// 速度环PID
Sguan0.Speed.Kp = 0.02226f;
Sguan0.Speed.Ki = 0.000115f;
Sguan0.Speed.OutMax = 0.9f;          // 输出限幅

// 电流环PID  
Sguan0.Id.Kp = 0.02226f;
Sguan0.Iq.Kp = 0.02226f;
```

## 🔄 控制流程

1. **初始化阶段**
   - 硬件初始化
   - 电流偏置校准
   - 编码器偏置校准
   - PID参数设置

2. **实时控制循环**
   - 电流采样与变换
   - 编码器位置/速度读取
   - PID控制器运算
   - SVPWM调制
   - PWM输出更新

## 🚀 性能优化

- **快速数学计算**: 使用多项式拟合实现快速sin/cos
- **内存优化**: 静态内存分配，无动态内存操作
- **执行效率**: 算法优化，适合实时控制
- **可配置性**: 所有参数可调，适应不同电机

## 📈 未来规划

### v2.1 版本预告
- 🔮 无位置传感器控制
- 🧮 PLL锁相环观测器  
- 🔍 SMO滑模观测器
- 🧲 非线性磁链观测器
- 📡 龙伯格观测器
- ⚡ HFI高频注入
- 🌀 弱磁控制

## 🤝 贡献指南

我们欢迎社区贡献！如果你有改进建议或bug修复，请：

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

## 📄 许可证

本项目采用自定义许可证，详情请查看源码文件头部的版权声明。

## 📞 联系支持

- 👨💻 作者: 星必尘Sguan
- 📧 邮箱: 3464647102@qq.com
- 🐛 问题反馈: [GitHub Issues]

## 🙏 致谢

感谢所有为项目做出贡献的开发者们！

---

**⭐ 如果这个项目对你有帮助，请给我们一个Star！**
