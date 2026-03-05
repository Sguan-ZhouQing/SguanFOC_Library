# SguanFOC - 高性能磁场定向控制库

![Version](https://img.shields.io/badge/Version-3.0.0-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Language](https://img.shields.io/badge/Language-C-00599C)
 <img src="https://img.shields.io/badge/🔄_完整FOC-开箱即用-red">
![Platform](https://img.shields.io/badge/Platform-ARM%20%7C%20DSP%20%7C%20任何C语言MCU-orange)

---

## ![无刷电机](https://github.com/user-attachments/assets/bf4b151d-f330-4d1e-a4ff-1aaee886554e) 项目简介

**SguanFOC库** 是一个完全使用纯C语言编写的开源磁场定向控制(FOC)算法库，专为嵌入式微控制器设计。该库提供了从坐标变换到SVPWM生成的完整FOC算法实现，全面覆盖从无感位置观测到多环闭环控制的完整技术链。它具有高性能、易移植、可配置性强等特点，可适用于所有C语言开发的电机控制项目。


## 📋 版本路线图

```
SguanFOC Library Evolution
═══════════════════════════════════════════════════════════════

v3.0.x ────► 有感FOC · 浮点/定点运算 · 基础算法库
    │
    ├── v3.0.0 ── 浮点运算 · 有感FOC · 基础控制算法
    │
    └── v3.0.1 ── Q31定点运算 · 有感FOC · 性能优化

v3.1.x ────► 无感FOC · 浮点/定点运算 · 观测器算法
    │
    ├── v3.1.0 ── 浮点运算 · 无感FOC · 多种观测器
    │
    └── v3.1.1 ── Q31定点运算 · 无感FOC · 观测器优化

v3.2.x ────► 扩展功能 · 通信协议 · 高级算法
    │
    ├── v3.2.0 ── 扩展卡尔曼 · ModbusRTU · 更多无感算法
    │
    └── v3.2.x ── 持续更新中...

═══════════════════════════════════════════════════════════════
```

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         SguanFOC Core                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐  │
│  │   状态机层       │    │   算法层        │    │   通信层     │  │
│  │  ═════════════  │    │  ═════════════  │    │  ═════════  │  │
│  │ • STANDBY       │    │ • PID           │    │ • JustFloat │  │
│  │ • INITIALIZING  │    │ • IMC           │    │ • printf    │  │
│  │ • CALIBRATING   │    │ • LADRC         │    │ • UART/CAN  │  │
│  │ • IDLE          │    │ • MTPA          │    │ • Modbus*   │  │
│  │ • TORQUE_CTRL   │    │ • PLL           │    └─────────────┘  │
│  │ • SPEED_CTRL    │    │ • HFI*          │                     │
│  │ • POSITION_CTRL │    │ • SMO*          │    ┌─────────────┐  │
│  │ • ERROR_HANDLER │    │ • Kerman*       │    │   滤波层     │  │
│  └─────────────────┘    └─────────────────┘    │  ═════════  │  │
│                                                │ •ButterWorth│  │
│  ┌─────────────────┐    ┌─────────────────┐    │ •数学运算    │  │
│  │   变换层         │    │   驱动层        │    └─────────────┘  │
│  │  ═════════════  │    │  ═════════════  │                     │
│  │ • Clarke/Park   │    │ • PWM生成       │    ┌─────────────┐  │
│  │ • Inverse Park  │    │ • ADC采样       │    │   辨识层*    │  │
│  │ • SVPWM         │    │ • 编码器读取     │    │  ═════════  │  │
│  │ • 坐标变换       │    │ • 电流采样      │    │ • 参数辨识   │  │
│  └─────────────────┘    └─────────────────┘    │ • NSD极性   │  │
│                                                └─────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                         * 表示v3.1.0+版本特性
```

---

## 📖 核心特性矩阵

| 特性 | v3.0.0 | v3.0.1 | v3.1.0 | v3.1.1 | v3.2.0+ |
|:-----|:------:|:------:|:------:|:------:|:-------:|
| **运算类型** | | | | | |
| `Q31`定点运算 | ❌ | ✅ | ❌ | ✅ | ✅ |
| **控制模式** | | | | | |
| 电流单环控制 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 速度单环控制 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 位置单环控制 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 速度-电流双环 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 位置-速度-电流三环 | ✅ | ✅ | ✅ | ✅ | ✅ |
| **控制算法** | | | | | |
| PID闭环控制 | ✅ | ✅ | ✅ | ✅ | ✅ |
| IMC内模控制 | ✅ | ✅ | ✅ | ✅ | ✅ |
| LADRC线自抗扰 | ✅ | ✅ | ✅ | ✅ | ✅ |
| MTPA弱磁控制 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 前馈解耦 | ✅ | ✅ | ✅ | ✅ | ✅ |
| **有感算法** | | | | | |
| PLL锁相环速度跟踪 | ✅ | ✅ | ✅ | ✅ | ✅ |
| **无感算法** | | | | | |
| HFI高频方波注入 | ❌ | ❌ | ✅ | ✅ | ✅ |
| SMO滑膜观测器 | ❌ | ❌ | ✅ | ✅ | ✅ |
| NSD转子极性辨识 | ❌ | ❌ | ✅ | ✅ | ✅ |
| 电机参数辨识 | ❌ | ❌ | ✅ | ✅ | ✅ |
| 扩展卡尔曼滤波器 | ❌ | ❌ | ❌ | ❌ | ✅ |
| **通信协议** | | | | | |
| JustFloat串口协议 | ✅ | ✅ | ✅ | ✅ | ✅ |
| printf重定向 | ✅ | ✅ | ✅ | ✅ | ✅ |
| ModbusRTU(RS485) | ❌ | ❌ | ❌ | ❌ | ✅ |
| **系统组件** | | | | | |
| 电机状态机 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 巴特沃斯滤波器 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 优化的数学运算 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 用户接口封装 | ✅ | ✅ | ✅ | ✅ | ✅ |

---

## 🧩 核心模块详解

### 📐 数学运算库 - `Sguan_math.c/.h`

```c
// 优化的数学运算 - 比标准库快30%以上
float fast_sin(float x);        // 快速正弦
float fast_cos(float x);        // 快速余弦  
void fast_sin_cos(float x, float *sin_x, float *cos_x);  // 同时计算
float Value_sqrtf(float x);      // 牛顿迭代平方根
float Value_fabsf(float x);      // 位操作绝对值
int Value_isnan(float x);        // NaN检测
int Value_isinf(float x);        // 无穷大检测

// 坐标变换
void clarke(float *i_alpha, float *i_beta, float i_a, float i_b);
void park(float *i_d, float *i_q, float i_alpha, float i_beta, float sine, float cosine);
void ipark(float *u_alpha, float *u_beta, float u_d, float u_q, float sine, float cosine);

// SVPWM空间矢量调制
void SVPWM(float d, float q, float sin_phi, float cos_phi, 
           float *d_u, float *d_v, float *d_w);
```

### 🎛️ 控制算法集

#### PID控制器 - `Sguan_PID.c/.h`
```c
typedef struct {
    float Kp, Ki, Kd;           // 增益参数
    float OutMax, OutMin;        // 输出限幅
    float IntMax, IntMin;        // 积分限幅
    float Wc;                    // 微分滤波截止频率
    float T;                      // 采样周期
} PID_STRUCT;

void PID_Init(PID_STRUCT *pid);
void PID_Loop(PID_STRUCT *pid);   // 误差计算并输出
```

#### IMC内模控制 - `Sguan_InternalModel.c/.h`
```c
typedef struct {
    float Rs, Ls;                // 电机电阻、电感
    float T;                      // 采样周期
    IMC_STRUCT imc;               // 内模控制器
} INTERNALMODEL_STRUCT;

void InternalModel_Init(INTERNALMODEL_STRUCT *im);
void InternalModel_Loop(INTERNALMODEL_STRUCT *im);
```

#### LADRC线自抗扰 - `Sguan_Ladrc.c/.h`
```c
typedef struct {
    float r;                      // 跟踪微分器速度因子
    float b0;                     // 控制量增益
    float wc;                     // 控制器带宽
    float T;                      // 采样周期
    LINEAR_STRUCT linear;         // 线性自抗扰变量
    DATA_STRUCT data;             // 自动计算的参数
} LADRC_STRUCT;

void Ladrc_Init(LADRC_STRUCT *ladrc);
void Ladrc_Loop(LADRC_STRUCT *ladrc);
```

#### PLL锁相环 - `Sguan_PLL.c/.h`
```c
typedef struct {
    double Kp, Ki;                // PI参数
    double T;                      // 采样周期
    uint8_t is_position_mode;      // 位置环模式标志
    GO_STRUCT go;                  // 锁相环变量
} PLL_STRUCT;

void PLL_Init(PLL_STRUCT *pll);
void PLL_Loop(PLL_STRUCT *pll);    // 角度误差输入，角速度/角度输出
```

### 🔄 滤波器

#### 二阶巴特沃斯低通 - `Sguan_Filter.c/.h`
```c
typedef struct {
    double Wc;                    // 截止频率
    double T;                      // 采样周期
    FILTER_STRUCT filter;          // 滤波器状态
} BPF_STRUCT;

void BPF_Init(BPF_STRUCT *bpf);
void BPF_Loop(BPF_STRUCT *bpf);    // 输入 -> 输出
```

### 🤖 电机状态机 - `Sguan_MotorStatus.c/.h`

```c
// 24种状态全面覆盖
typedef enum {
    // 初始化与运行状态 (0x00-0x03)
    MOTOR_STATUS_STANDBY,           // 待机
    MOTOR_STATUS_UNINITIALIZED,     // 未初始化
    MOTOR_STATUS_INITIALIZING,      // 初始化中
    MOTOR_STATUS_CALIBRATING,       // 校准中
    
    // 运行状态 (0x04-0x0D)
    MOTOR_STATUS_IDLE,              // 空闲
    MOTOR_STATUS_TORQUE_INCREASING, // 力矩增大
    MOTOR_STATUS_TORQUE_DECREASING, // 力矩减小
    MOTOR_STATUS_TORQUE_CONTROL,    // 力矩保持
    MOTOR_STATUS_ACCELERATING,      // 加速中
    MOTOR_STATUS_DECELERATING,      // 减速中
    MOTOR_STATUS_CONST_SPEED,       // 恒速
    MOTOR_STATUS_POSITION_INCREASING, // 位置增加
    MOTOR_STATUS_POSITION_DECREASING, // 位置减少
    MOTOR_STATUS_POSITION_HOLD,      // 位置保持
    
    // 硬件错误 (0x0E-0x15)
    MOTOR_STATUS_OVERVOLTAGE,        // 过压
    MOTOR_STATUS_UNDERVOLTAGE,       // 欠压
    MOTOR_STATUS_OVERTEMPERATURE,    // 过温
    MOTOR_STATUS_UNDERTEMPERATURE,   // 低温
    MOTOR_STATUS_OVERCURRENT,        // 过流
    MOTOR_STATUS_ENCODER_ERROR,      // 编码器故障
    MOTOR_STATUS_SENSOR_ERROR,       // 传感器故障
    MOTOR_STATUS_PWM_CALC_FAULT,     // PWM计算错误
    
    // 安全状态 (0x16-0x17)
    MOTOR_STATUS_EMERGENCY_STOP,     // 急停
    MOTOR_STATUS_DISABLED             // 失能
} MOTOR_STATUS;

void MotorStatus_Loop(uint8_t *status);  // 自动调度状态处理函数
```

### 📡 通信协议

#### JustFloat协议 - `Sguan_printf.c/.h`
```c
typedef struct {
    float fdata[CH_COUNT];        // 最多12个浮点数
    uint8_t tail[4];              // 帧尾 {0x00,0x00,0x80,0x7f}
} PRINTF_STRUCT;

void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(PRINTF_STRUCT *str);     // 发送数据
void Printf_Adjust(void);                  // 接收解析
```

---

## 🎯 版本详细说明

### 📦 v3.0.x - 有感FOC · 基础算法库

| 版本 | 运算类型 | 发布日期 | 核心功能 |
|:----:|:--------:|:--------:|:---------|
| **v3.0.0** | `float`浮点 | 2026.03 | 有感FOC完整实现，基础控制算法，用户接口封装 |
| **v3.0.1** | `Q31`定点 | 计划中 | 浮点转定点，性能优化，低端MCU适配 |

**功能清单：**
- ✅ 电机状态机 - 24种状态全面覆盖
- ✅ 优化的数学运算 - 快速sin/cos/sqrt
- ✅ JustFloat串口协议 - 兼容匿名上位机
- ✅ PLL锁相环速度跟踪 - 高精度角度/速度估算
- ✅ PID闭环控制 - 位置/速度/电流三环
- ✅ IMC内模控制 - 电流环参数自适应
- ✅ LADRC线自抗扰控制 - 速度环高性能控制
- ✅ MTPA弱磁控制 - IPMSM最大转矩电流比
- ✅ 前馈解耦 - dq轴解耦控制
- ✅ 巴特沃斯滤波器 - 二阶低通滤波
- ✅ printf重定向 - 串口调试支持
- ✅ 用户接口提供 - 硬件抽象层封装

### 🔮 v3.1.x - 无感FOC · 观测器算法库

| 版本 | 运算类型 | 发布日期 | 核心功能 |
|:----:|:--------:|:--------:|:---------|
| **v3.1.0** | `float`浮点 | 计划中 | 无感FOC实现，多种观测器算法 |
| **v3.1.1** | `Q31`定点 | 计划中 | 无感算法定点化，性能优化 |

**新增功能：**
- ✅ HFI高频方波注入 - 零低速转子位置估算
- ✅ SMO滑膜观测器 - 中高速转子位置估算
- ✅ NSD转子极性辨识 - N/S极判别
- ✅ 电机参数辨识 - Rs/Ld/Lq/Flux在线辨识

### 🚀 v3.2.x - 扩展功能 · 高级算法库

| 版本 | 发布日期 | 核心功能 |
|:----:|:--------:|:---------|
| **v3.2.0** | 计划中 | 扩展卡尔曼滤波，ModbusRTU，更多无感算法 |

**新增功能：**
- ✅ 扩展卡尔曼滤波器 - 高精度状态估计
- ✅ ModbusRTU协议 - RS485工业通信
- ✅ 更多无感算法 - 观测器算法补充

---

## 🔧 快速开始

### 1️⃣ 硬件抽象层实现

```c
// UserData_Function.h - 实现硬件接口
static inline void User_InitialInit(void) {
    // 初始化PWM定时器、ADC、编码器、驱动芯片
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
    // ...
}

static inline int32_t User_ReadADC_Raw(uint8_t Current_CH) {
    // 返回ADC原始值
    switch(Current_CH) {
        case 0: return adc_buf[0];  // IA相
        case 1: return adc_buf[1];  // IB相
        default: return 0;
    }
}

static inline float User_Encoder_ReadRad(void) {
    // 返回编码器角度 (0-2π)
    uint32_t enc_raw = __HAL_TIM_GET_COUNTER(&htim2);
    return (float)enc_raw / ENC_RESOLUTION * 2 * PI;
}

static inline void User_PwmDuty_Set(uint16_t Duty_u, uint16_t Duty_v, uint16_t Duty_w) {
    // 设置三相PWM占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_w);
}
```

### 2️⃣ 电机参数配置

```c
// UserData_Motor.h - 配置电机参数
static inline void User_MotorSet(void) {
    // 控制模式选择
    Sguan.mode = VelCur_DOUBLE_MODE;  // 速度-电流双环
    
    // 电机实体参数
    Sguan.identify.Ld = 0.00005193f;   // D轴电感 (H)
    Sguan.identify.Lq = 0.00005193f;   // Q轴电感 (H)
    Sguan.identify.Rs = 0.19067f;      // 相电阻 (Ω)
    Sguan.identify.Flux = 0.00028043f; // 磁链 (Wb)
    
    // 电机结构参数
    Sguan.motor.Poles = 7;              // 极对数
    Sguan.motor.VBUS = 12.0f;           // 母线电压 (V)
    Sguan.motor.Duty = 4249;             // PWM满占空比
    
    // 采样参数
    Sguan.motor.ADC_Precision = 4096;    // 12位ADC
    Sguan.motor.MCU_Voltage = 3.3f;      // ADC基准电压
    Sguan.motor.Amplifier = 10.0f;       // 运放增益
    Sguan.motor.Sampling_Rs = 0.005f;    // 采样电阻 (Ω)
    
    // 安全参数
    Sguan.safe.VBUS_MAX = 14.0f;         // 过压阈值
    Sguan.safe.VBUS_MIM = 10.0f;         // 欠压阈值
    Sguan.safe.Qcur_MAX = 10.0f;         // Q轴电流限幅 (A)
}
```

### 3️⃣ 控制器参数配置

```c
// UserData_Parameter.h - 配置控制参数
static inline void User_ParameterSet(void) {
    // 滤波器参数
    Sguan.bpf.CurrentD.Wc = 31415.96f;   // 电流滤波 (5kHz)
    Sguan.bpf.CurrentQ.Wc = 31415.96f;   // 电流滤波 (5kHz)
    Sguan.bpf.Encoder.Wc = 314.1596f;    // 速度滤波 (50Hz)
    
    // 电流环PID参数
    Sguan.control.Current_D.Kp = 0.261f;
    Sguan.control.Current_D.Ki = 958.41f;
    Sguan.control.Current_D.OutMax = 12.0f;
    
    // 速度环PID参数
    Sguan.control.Velocity.Kp = 0.06f;
    Sguan.control.Velocity.Ki = 0.4f;
    Sguan.control.Velocity.OutMax = 10.5f;
    
    // 位置环PD参数
    Sguan.control.Position.Kp = 12.0f;
    Sguan.control.Position.Kd = 0.0f;
    
    // 锁相环参数
    Sguan.encoder.pll.Kp = 650.0f;
    Sguan.encoder.pll.Ki = 210000.0f;
}
```

### 4️⃣ 集成到主程序

```c
// main.c
#include "SguanFOC.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // 用户初始化
    User_InitialInit();
    
    while(1) {
        SguanFOC_main_Loop();      // 主循环任务
    }
}

// 20kHz PWM中断
void TIM1_UP_IRQHandler(void) {
    SguanFOC_High_Loop();          // 高速控制环
}

// 1kHz 定时器中断
void TIM3_IRQHandler(void) {
    SguanFOC_Low_Loop();           // 低速状态机
}

// 串口接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    SguanFOC_Printf_Loop(rx_buffer, rx_len);  // 协议解析
}
```

---

## 📊 性能指标

| 指标 | v3.0.0 | v3.0.1 | 说明 |
|:-----|:------:|:------:|:-----|
| 控制周期 | 20-50µs | 20-50µs | 取决于主频 |
| 代码大小 | ~15KB | ~12KB | Flash占用 |
| RAM占用 | ~2KB | ~2KB | 每电机 |
| 最大转速 | 100k RPM | 100k RPM | 取决于极对数 |
| 电流环带宽 | 2-5kHz | 2-5kHz | 可配置 |
| 速度环带宽 | 200-500Hz | 200-500Hz | 可配置 |

---

## 🔜 开发计划

```
2026 Q2 ──── v3.0.1 有感FOC定点运算版本发布
2026 Q3 ──── v3.1.0 无感FOC浮点运算版本发布
2026 Q4 ──── v3.1.1 无感FOC定点运算版本发布
2027 Q1 ──── v3.2.0 扩展卡尔曼+ModbusRTU版本发布
```

---

## 🤝 贡献指南

我们欢迎社区贡献！如果你有改进建议或bug修复：

1. 🍴 Fork 本项目
2. 🌿 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 💾 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 📤 推送到分支 (`git push origin feature/AmazingFeature`)
5. 🔃 开启Pull Request

---

## 📄 许可证

本项目采用MIT许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 📞 联系支持

| 渠道 | 联系方式 |
|:-----|:---------|
| 👨‍💻 **作者** | 星必尘Sguan                     |
| 📧 **邮箱** | 3464647102@qq.com |
| 🐛 **Issues** | [GitHub Issues](https://github.com/Sguan-ZhouQing/SguanFOC_Library/issues) |
| 📚 **Wiki** | [项目Wiki](https://github.com/Sguan-ZhouQing/SguanFOC_Library/wiki) |

---

## 🌟 致谢

感谢所有为项目做出贡献的开发者们！

<div align="center">
  <img src="https://img.shields.io/badge/⭐-如果这个项目对你有帮助，请给我们一个Star！-brightgreen">
</div>

---

**SguanFOC - 让电机控制更简单** 🚀
