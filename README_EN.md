# SguanFOC - High-Performance Field-Oriented Control Library
![Version](https://img.shields.io/badge/Version-3.0.0-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Language](https://img.shields.io/badge/Language-C-00599C)
<img src="https://img.shields.io/badge/🔄_Full_FOC-Out-of-the-Box-red">
![Platform](https://img.shields.io/badge/Platform-ARM%20%7C%20DSP%20%7C%20Any_C_MCU-orange)
[English](README_EN.md) / [中文](README.md)

---

## ![Brushless Motor](https://github.com/user-attachments/assets/bf4b151d-f330-4d1e-a4ff-1aaee886554e) Project Overview
**SguanFOC Library** is an open-source Field-Oriented Control (FOC) algorithm library written entirely in standard C, designed for embedded MCUs. It implements the full FOC chain from coordinate transformation to SVPWM generation, covering sensorless position observation and multi-loop closed-loop control. It is high-performance, portable, highly configurable, and suitable for all C-language motor control projects.

## 📋 Version Roadmap
```
SguanFOC Library Evolution
═══════════════════════════════════════════════════════════════
SguanFOC Library v3.0.0 ->
Target: Sensor-based FOC motor control | Floating-point
Motor state machine | Optimized math | Justfloat UART protocol
PLL speed tracking | PID closed-loop | IMC internal model control
LADRC linear ADRC | MTPA flux-weakening | Feedforward decoupling
Butterworth filter | printf redirection | User HAL interface

SguanFOC Library v3.0.1 ->
Target: Sensor-based FOC motor control | Q31 fixed-point
Motor state machine | Optimized math | Justfloat UART protocol
PLL speed tracking | PID closed-loop | IMC internal model control
LADRC linear ADRC | MTPA flux-weakening | Feedforward decoupling
Butterworth filter | printf redirection | User HAL interface

SguanFOC Library v3.1.0 ->
Target: Sensorless FOC motor control | Floating-point
Motor state machine | Optimized math | Justfloat UART protocol
HFI high-frequency square-wave injection & rotor position estimation
SMO sliding-mode observer in stationary frame
NSD rotor polarity identification | Motor parameter identification
PLL speed tracking | PID closed-loop | IMC internal model control
LADRC linear ADRC | MTPA flux-weakening | Feedforward decoupling
Butterworth filter | printf redirection | User HAL interface

SguanFOC Library v3.1.1 ->
Target: Sensorless FOC motor control | Q31 fixed-point
Motor state machine | Optimized math | Justfloat UART protocol
HFI high-frequency square-wave injection & rotor position estimation
SMO sliding-mode observer in stationary frame
NSD rotor polarity identification | Motor parameter identification
PLL speed tracking | PID closed-loop | IMC internal model control
LADRC linear ADRC | MTPA flux-weakening | Feedforward decoupling
Butterworth filter | printf redirection | User HAL interface

SguanFOC Library v3.2.0 -> Planned: Extended Kalman filter, ModbusRTU over RS485, more sensorless algorithms
═══════════════════════════════════════════════════════════════
```

---

## 🏗️ System Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                         SguanFOC Core                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐  │
│  │ State Machine   │    │ Algorithm Layer │    │ Communication│  │
│  │ ════════════════│    │ ════════════════│    │ ════════════│  │
│  │ • STANDBY       │    │ • PID           │    │ • JustFloat │  │
│  │ • INITIALIZING  │    │ • IMC           │    │ • printf    │  │
│  │ • CALIBRATING   │    │ • LADRC         │    │ • UART/CAN  │  │
│  │ • IDLE          │    │ • MTPA          │    │ • Modbus*   │  │
│  │ • TORQUE_CTRL   │    │ • PLL           │    └─────────────┘  │
│  │ • SPEED_CTRL    │    │ • HFI*          │                     │
│  │ • POSITION_CTRL │    │ • SMO*          │    ┌─────────────┐  │
│  │ • ERROR_HANDLER │    │ • Kalman*       │    │ Filter Layer │  │
│  └─────────────────┘    └─────────────────┘    │ ════════════│  │
│                                                │ • Butterworth│  │
│  ┌─────────────────┐    ┌─────────────────┐    │ • Math Utils │  │
│  │ Transformation  │    │ Driver Layer    │    └─────────────┘  │
│  │ ════════════════│    │ ════════════════│                     │
│  │ • Clarke/Park   │    │ • PWM Generation│    ┌─────────────┐  │
│  │ • Inverse Park  │    │ • ADC Sampling  │    │ Ident Layer*│  │
│  │ • SVPWM         │    │ • Encoder Read  │    │ ════════════│  │
│  │ • Coord Trans   │    │ • Current Sample│    │ • Param ID  │  │
│  └─────────────────┘    └─────────────────┘    │ • NSD Polarity│ │
│                                                └─────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                         * Features for v3.1.0+
```

---

## 🧩 Core Modules

### 📐 Math Library – `Sguan_math.c/.h`
```c
// Optimized math – >30% faster than standard library
float fast_sin(float x);                                 // Fast sine
float fast_cos(float x);                                 // Fast cosine
void fast_sin_cos(float x, float *sin_x, float *cos_x);  // Simultaneous compute
float Value_sqrtf(float x);                              // Newton–Raphson sqrt
float Value_fabsf(float x);                              // Bitwise absolute value
int Value_isnan(float x);                                // NaN check
int Value_isinf(float x);                                // Inf check

// Coordinate transformations
void clarke(float *i_alpha, float *i_beta, float i_a, float i_b);
void park(float *i_d, float *i_q, float i_alpha, float i_beta, float sine, float cosine);
void ipark(float *u_alpha, float *u_beta, float u_d, float u_q, float sine, float cosine);

// SVPWM
void SVPWM(float d, float q, float sin_phi, float cos_phi,
           float *d_u, float *d_v, float *d_w);
```

### 🎛️ Control Algorithms

#### PID Controller – `Sguan_PID.c/.h`
```c
typedef struct {
    float Kp, Ki, Kd;                                    // Gains
    float OutMax, OutMin;                                // Output clamp
    float IntMax, IntMin;                                // Integral clamp
    float Wc;                                            // Derivative filter cutoff
    float T;                                             // Sample period
} PID_STRUCT;

void PID_Init(PID_STRUCT *pid);
void PID_Loop(PID_STRUCT *pid);                          // Compute error & output
```

#### IMC Internal Model Control – `Sguan_InternalModel.c/.h`
```c
typedef struct {
    float Rs, Ls;                                        // Motor resistance, inductance
    float T;                                             // Sample period
    IMC_STRUCT imc;                                      // IMC controller
} INTERNALMODEL_STRUCT;

void InternalModel_Init(INTERNALMODEL_STRUCT *im);
void InternalModel_Loop(INTERNALMODEL_STRUCT *im);
```

#### LADRC Linear ADRC – `Sguan_Ladrc.c/.h`
```c
typedef struct {
    float r;                                             // Tracking differentiator speed
    float b0;                                            // Control gain
    float wc;                                            // Controller bandwidth
    float T;                                             // Sample period
    LINEAR_STRUCT linear;                                // Linear ADRC state
    DATA_STRUCT data;                                    // Auto-calculated params
} LADRC_STRUCT;

void Ladrc_Init(LADRC_STRUCT *ladrc);
void Ladrc_Loop(LADRC_STRUCT *ladrc);
```

#### PLL Phase-Locked Loop – `Sguan_PLL.c/.h`
```c
typedef struct {
    double Kp, Ki;                                       // PI gains
    double T;                                            // Sample period
    uint8_t is_position_mode;                            // Position loop flag
    GO_STRUCT go;                                        // PLL internal state
} PLL_STRUCT;

void PLL_Init(PLL_STRUCT *pll);
void PLL_Loop(PLL_STRUCT *pll);                          // Angle error → speed/angle output
```

### 🔄 Filter
#### 2nd-Order Butterworth LPF – `Sguan_Filter.c/.h`
```c
typedef struct {
    double Wc;                                           // Cutoff frequency
    double T;                                            // Sample period
    FILTER_STRUCT filter;                                // Filter state
} BPF_STRUCT;

void BPF_Init(BPF_STRUCT *bpf);
void BPF_Loop(BPF_STRUCT *bpf);                          // Input → filtered output
```

### 🤖 Motor State Machine – `Sguan_MotorStatus.c/.h`
```c
// 24 states fully covered
typedef enum {
    // Init & startup (0x00–0x03)
    MOTOR_STATUS_STANDBY,                                // Standby
    MOTOR_STATUS_UNINITIALIZED,                          // Uninitialized
    MOTOR_STATUS_INITIALIZING,                           // Initializing
    MOTOR_STATUS_CALIBRATING,                            // Calibrating

    // Running (0x04–0x0D)
    MOTOR_STATUS_IDLE,                                   // Idle
    MOTOR_STATUS_TORQUE_INCREASING,                      // Torque rising
    MOTOR_STATUS_TORQUE_DECREASING,                      // Torque falling
    MOTOR_STATUS_TORQUE_CONTROL,                         // Torque hold
    MOTOR_STATUS_ACCELERATING,                           // Accelerating
    MOTOR_STATUS_DECELERATING,                           // Decelerating
    MOTOR_STATUS_CONST_SPEED,                            // Constant speed
    MOTOR_STATUS_POSITION_INCREASING,                    // Position increasing
    MOTOR_STATUS_POSITION_DECREASING,                    // Position decreasing
    MOTOR_STATUS_POSITION_HOLD,                          // Position hold

    // Hardware faults (0x0E–0x15)
    MOTOR_STATUS_OVERVOLTAGE,                            // Overvoltage
    MOTOR_STATUS_UNDERVOLTAGE,                           // Undervoltage
    MOTOR_STATUS_OVERTEMPERATURE,                        // Overtemperature
    MOTOR_STATUS_UNDERTEMPERATURE,                       // Undertemperature
    MOTOR_STATUS_OVERCURRENT,                            // Overcurrent
    MOTOR_STATUS_ENCODER_ERROR,                          // Encoder fault
    MOTOR_STATUS_SENSOR_ERROR,                           // Sensor fault
    MOTOR_STATUS_PWM_CALC_FAULT,                         // PWM calculation error

    // Safety (0x16–0x17)
    MOTOR_STATUS_EMERGENCY_STOP,                         // Emergency stop
    MOTOR_STATUS_DISABLED                                // Disabled
} MOTOR_STATUS;

void MotorStatus_Loop(uint8_t *status);                  // Auto state dispatcher
```

### 📡 Communication Protocol
#### JustFloat Protocol – `Sguan_printf.c/.h`
```c
typedef struct {
    float fdata[CH_COUNT];                               // Default 12 floats
    uint8_t tail[4];                                     // Frame tail {0x00,0x00,0x80,0x7f}
} PRINTF_STRUCT;

void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(PRINTF_STRUCT *str);                    // Transmit data
void Printf_Adjust(void);                                // Parse received data
```

---

## 🎯 Version Details

### 📦 v3.0.x – Sensor‑Based FOC · Core Library
| Version | Math Type | Release | Core Features |
|:-------:|:---------:|:-------:|:-------------|
| **v3.0.0** | `float` | 2026.03 | Full sensor‑based FOC, basic controls, user HAL |
| **v3.0.1** | `Q31` | Planned | Fixed-point, optimization, low‑end MCU support |

**Features:**
- ✅ Motor state machine (24 states)
- ✅ Optimized fast math (sin/cos/sqrt)
- ✅ JustFloat UART (compatible with Vofa/Anonymous上位机)
- ✅ PLL speed tracking
- ✅ PID position/speed/current triple loop
- ✅ IMC current loop adaptation
- ✅ LADRC high-performance speed loop
- ✅ MTPA flux-weakening for IPMSM
- ✅ dq-axis feedforward decoupling
- ✅ 2nd-order Butterworth LPF
- ✅ printf redirection for debugging
- ✅ Hardware abstraction layer

### 🔮 v3.1.x – Sensorless FOC · Observer Library
| Version | Math Type | Release | Core Features |
|:-------:|:---------:|:-------:|:-------------|
| **v3.1.0** | `float` | Planned | Full sensorless FOC, multiple observers |
| **v3.1.1** | `Q31` | Planned | Fixed-point sensorless, performance tuning |

**New Features:**
- ✅ HFI high‑frequency injection (zero/low speed)
- ✅ SMO sliding‑mode observer (medium/high speed)
- ✅ NSD rotor polarity detection
- ✅ Online parameter ID (Rs/Ld/Lq/Flux)

### 🚀 v3.2.x – Extended Features · Advanced Library
| Version | Release | Core Features |
|:-------:|:-------:|:-------------|
| **v3.2.0** | Planned | Extended Kalman filter, ModbusRTU, more sensorless algorithms |

**New Features:**
- ✅ Extended Kalman filter (high‑precision state estimation)
- ✅ ModbusRTU over RS485 (industrial comms)
- ✅ Additional sensorless observers

---

## 🔧 Quick Start

### 1️⃣ Hardware Abstraction Layer
```c
// UserData_Function.h – implement hardware hooks
static inline void User_InitialInit(void) {
    // Init PWM, ADC, encoder, driver
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
    // ...
}

static inline int32_t User_ReadADC_Raw(uint8_t Current_CH) {
    switch(Current_CH) {
        case 0: return adc_buf[0];  // Phase IA
        case 1: return adc_buf[1];  // Phase IB
        default: return 0;
    }
}

static inline float User_Encoder_ReadRad(void) {
    // Return angle 0–2π
    uint32_t enc_raw = __HAL_TIM_GET_COUNTER(&htim2);
    return (float)enc_raw / ENC_RESOLUTION * 2 * PI;
}

static inline void User_PwmDuty_Set(uint16_t Duty_u, uint16_t Duty_v, uint16_t Duty_w) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_w);
}
```

### 2️⃣ Motor Configuration
```c
// UserData_Motor.h
static inline void User_MotorSet(void) {
    Sguan.mode = VelCur_DOUBLE_MODE;  // Speed–current dual loop

    // Motor parameters
    Sguan.identify.Ld = 0.00005193f;
    Sguan.identify.Lq = 0.00005193f;
    Sguan.identify.Rs = 0.19067f;
    Sguan.identify.Flux = 0.00028043f;

    // Physical
    Sguan.motor.Poles = 7;
    Sguan.motor.VBUS = 12.0f;
    Sguan.motor.Duty = 4249;

    // Sampling
    Sguan.motor.ADC_Precision = 4096;
    Sguan.motor.MCU_Voltage = 3.3f;
    Sguan.motor.Amplifier = 10.0f;
    Sguan.motor.Sampling_Rs = 0.005f;

    // Safety
    Sguan.safe.VBUS_MAX = 14.0f;
    Sguan.safe.VBUS_MIM = 10.0f;
    Sguan.safe.Qcur_MAX = 10.0f;
}
```

### 3️⃣ Controller Tuning
```c
// UserData_Parameter.h
static inline void User_ParameterSet(void) {
    // Filters
    Sguan.bpf.CurrentD.Wc = 31415.96f;
    Sguan.bpf.CurrentQ.Wc = 31415.96f;
    Sguan.bpf.Encoder.Wc = 314.1596f;

    // Current loop PID
    Sguan.control.Current_D.Kp = 0.261f;
    Sguan.control.Current_D.Ki = 958.41f;
    Sguan.control.Current_D.OutMax = 12.0f;

    // Speed loop PID
    Sguan.control.Velocity.Kp = 0.06f;
    Sguan.control.Velocity.Ki = 0.4f;
    Sguan.control.Velocity.OutMax = 10.5f;

    // Position loop PD
    Sguan.control.Position.Kp = 12.0f;
    Sguan.control.Position.Kd = 0.0f;

    // PLL
    Sguan.encoder.pll.Kp = 650.0f;
    Sguan.encoder.pll.Ki = 210000.0f;
}
```

### 4️⃣ Main Integration
```c
// main.c
#include "SguanFOC.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    User_InitialInit();

    while(1) {
        SguanFOC_main_Loop();      // Background tasks
    }
}

// 20kHz PWM interrupt
void TIM1_UP_IRQHandler(void) {
    SguanFOC_High_Loop();          // High‑speed control loop
}

// 1kHz timer interrupt
void TIM3_IRQHandler(void) {
    SguanFOC_Low_Loop();           // State machine & slow loops
}

// UART RX complete
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    SguanFOC_Printf_Loop(rx_buffer, rx_len);
}
```

---

## 📊 Performance
| Metric | v3.0.0 | v3.0.1 | Notes |
|:-------|:------:|:------:|:------|
| Control cycle | 20–50µs | 20–50µs | Depends on clock |
| Flash size | ~15KB | ~12KB | |
| RAM per motor | ~2KB | ~2KB | |
| Max speed | 100k RPM | 100k RPM | Depends on pole pairs |
| Current loop bandwidth | 2–5kHz | 2–5kHz | Configurable |
| Speed loop bandwidth | 200–500Hz | 200–500Hz | Configurable |

---

## 🔜 Development Plan
```
2026 Q2 ──── v3.0.1 Sensor‑based FOC fixed‑point release
2026 Q3 ──── v3.1.0 Sensorless FOC floating‑point release
2026 Q4 ──── v3.1.1 Sensorless FOC fixed‑point release
2027 Q1 ──── v3.2.0 Extended Kalman + ModbusRTU release
```

---

## 🤝 Contributing
Community contributions are welcome!
1. Fork this project
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📞 Support & Credits
| Channel | Contact |
|:--------|:--------|
| 👨‍💻 **Author** | XingBichen Sguan |
| 📧 **Email** | 3464647102@qq.com |
| 🐛 **Issues** | [GitHub Issues](https://github.com/Sguan-ZhouQing/SguanFOC_Library/issues) |
| 📚 **Wiki** | [Project Wiki](https://github.com/Sguan-ZhouQing/SguanFOC_Library/wiki) |

Released under the MIT License – see [LICENSE](LICENSE). Thanks to all contributors!

<img src="https://img.shields.io/badge/⭐-Star_if_this_project_helps_you!-brightgreen">

---

**SguanFOC – Make Motor Control Simpler** 🚀
