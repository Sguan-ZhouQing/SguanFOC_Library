# SguanFOC - High-Performance Field-Oriented Control Library ![Brushless Motor](https://cdn.jsdelivr.net/gh/Sguan-ZhouQing/SguanFOC_Library@main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Image/MOTOR.svg)

> *"Pure C. A few lines of config. Your motor spins like silk."*

[![Version](https://img.shields.io/badge/Version-3.1.0-blue)](https://github.com/Sguan-ZhouQing/SguanFOC_Library)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Language](https://img.shields.io/badge/Language-C-00599C)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Platform](https://img.shields.io/badge/Platform-ARM%20%7C%20DSP%20%7C%20Any%20C%20MCU-orange)](https://github.com/Sguan-ZhouQing/SguanFOC_Library)
[![Full FOC](https://img.shields.io/badge/🔄_Full_FOC-Ready_to_Use-red)](https://github.com/Sguan-ZhouQing/SguanFOC_Library)
[**English**](https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/README_EN.md) | [**中文**](https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/README.md)

<br>

**Run a complete FOC in your MCU — from coordinate transform to SVPWM, from sensored to sensorless.**

<br>

5 minutes of initialization, and you can spin a brushless motor — sensored, sensorless, Hall, your choice. From **sensored three-loop FOC** to **HFI high-frequency injection + high-speed domain sensorless observer**, covering the full speed range. Not just a "it spins" level — it's servo-grade response, industrial-grade stability.

**Every control algorithm you see has been tested on real hardware.** Not a paper simulation, not a proof of concept — just C language + MCU!

![SguanFOC System Architecture Diagram](https://cdn.jsdelivr.net/gh/Sguan-ZhouQing/SguanFOC_Library@main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Image/SguanFOC_main.png)

<p align="center"><sub>
  ▲ [Code Framework Legend -> SguanFOC Library File Inclusion Relationships]
  👉 <a href="https://www.bilibili.com/video/BV13e9gBEEce?vd_source=7aed43223d7aff10698a55de16717980">Watch the latest SguanFOC porting video (Bilibili)</a>
</sub></p>

> 📣 **Open-sourced under the MIT license.** Free for personal and commercial use, no authorization required. Cross-platform universal... ARM, DSP, any C-language MCU can use it!

## 1. Simple Start 🚀

> *"Include the header. Implement the hooks. Motor closes loop."*

```c
#include "SguanFOC.h"

int main(void) {
    MCU_Init();                      // Your MCU platform + peripheral init
    while(1) {
        SguanFOC_main_Loop();        // ① Initialization + serial data Tx/Rx
    }
}

void TIM1_IRQHandler(void) {
    SguanFOC_High_Loop();            // ② High-frequency task: current loop, angle calculation, PWM generation
    // Your other high-priority interrupt tasks
}

void TIM2_IRQHandler(void) {
    SguanFOC_Low_Loop();             // ③ Low-frequency task: state machine switching, fault protection
    // Your other low-priority interrupt tasks
}

void USART1_IRQHandler(void) {
    SguanFOC_Printf_Loop(Sguan_PrintfBuff, Size); // ④ Serial debugging: receive parsing
    // Your other serial interrupt tasks
}

// ⑤ User-defined: Fill in motor control mode and operation parameters in UserData_*.h files in order
```

## 2. Star Trends (^^)

> *"Hit the star. Bookmark it. Your peers are already using it."*

<a href="https://www.star-history.com/?repos=Sguan-ZhouQing%2FSguanFOC_Library&type=date&legend=top-left">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=Sguan-ZhouQing/SguanFOC_Library&type=date&theme=dark&legend=top-left" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=Sguan-ZhouQing/SguanFOC_Library&type=date&legend=top-left" />
   <img alt="Star History Chart" src="https://api.star-history.com/chart?repos=Sguan-ZhouQing/SguanFOC_Library&type=date&legend=top-left" />
 </picture>
</a>

## 3. 📋 Version Roadmap (Library Code Functionality Showcase)

```
SguanFOC Library Evolution
═══════════════════════════════════════════════════════════════

SguanFOC Library v3.0.0 ->
Positioning: Sensored FOC motor control algorithm library, floating-point operations
Motor state machine, optimized mathematical operations, Justfloat serial protocol
PLL phase-locked loop speed tracking, PID closed-loop control, IMC internal model concept
LADRC linear active disturbance rejection control, MTPA maximum torque per ampere, feedforward decoupling
Butterworth filter, printf redirection, user interface provided

SguanFOC Library v3.0.1 ->
Positioning: Sensored FOC motor control algorithm library, Q31 fixed-point operations
Motor state machine, optimized mathematical operations, Justfloat serial protocol
PLL phase-locked loop speed tracking, PID closed-loop control, IMC internal model concept
STA second-order sliding mode control, MTPA maximum torque per ampere, feedforward decoupling
Butterworth filter, printf redirection, user interface provided

SguanFOC Library v3.1.0 ->
Positioning: Sensorless FOC motor control algorithm library, floating-point operations
Motor state machine, optimized mathematical operations, Justfloat serial protocol
HFI high-frequency sine wave injection and rotor position estimation, harmonic suppression
SMO stationary frame sliding mode observer algorithm, SVPWM and SPWM with third harmonic injection
NLFO nonlinear flux linkage observer, Hall mode, feedforward decoupling
DOB super-twisting sliding mode disturbance observer, field weakening control
Anti-cogging algorithm, angle phase delay compensation, dead-time compensation
NSD rotor polarity identification, motor parameter identification, SMC sliding mode control
PLL phase-locked loop speed tracking, PID closed-loop control, STA super-twisting sliding mode control
LADRC linear active disturbance rejection control, MTPA maximum torque per ampere, FW field weakening control
Three typical second-order filters, printf redirection, user interface provided
═══════════════════════════════════════════════════════════════
```

## 4. Sensorless Algorithm Hardware Verification Diagrams

![HFI to NLFO Diagram](https://cdn.jsdelivr.net/gh/Sguan-ZhouQing/SguanFOC_Library@main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Image/HFI_to_NLFO.jpg)

```
* The above shows HFI high-frequency sine wave injection switching to SMO sliding mode observer
* The above shows HFI high-frequency sine wave injection switching to NLFO nonlinear flux linkage observer
* (Illustrations are from simple parameter tuning verification; deeper debugging yields better results)
* (Other SguanFOC functions are normal; parameter identification and polarity identification have defects to be fixed in future projects)
```

![HFI to SMO Diagram](https://cdn.jsdelivr.net/gh/Sguan-ZhouQing/SguanFOC_Library@main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Image/HFI_to_SMO.jpg)

---

## 5. Control Mode Overview

> *"One macro. Switch modes. Motor adapts to any scenario."*

- 🎯 **19 Control Modes**: Open-loop → Sensored → Sensorless → Full-speed fusion
- 🎛️ **4 Control Algorithms**: PID / LADRC / SMC / STA
- 🔬 **3 Sensorless Observers**: HFI / SMO / NLFO
- ⚡ **10+ Advanced Features**: MTPA / Field Weakening / Harmonic Suppression / Parameter Identification / Dead-time Compensation...

| Index | Mode | Description | Positioning Method | Application Scenario |
|:----:|:-----|:-----|:---------|:---------|
| **0** | `MODE_VF_OPENLOOP` | V/F open-loop | Sensorless | Open-loop forced start, motor preliminary test |
| **1** | `MODE_IF_OPENLOOP` | I/F open-loop | Sensorless | Open-loop forced start, current limiting protection |
| **2** | `MODE_Voltag_OPEN` | Voltage open-loop | Encoder | Voltage mode debugging |
| **3** | `MODE_Current_SINGLE` | Single current loop | Encoder | Torque control mode |
| **4** | `MODE_VelCur_DOUBLE` | Velocity-current cascade loop | Encoder | General speed control |
| **5** | `MODE_PosVelCur_THREE` | Position-velocity-current three loops | Encoder | Servo positioning, precision control |
| **6** | `MODE_Sensor_Hall` | Sensored Hall speed loop | Hall sensor | Low-cost speed control |
| **7** | `MODE_Sensorless_HFI` | High-frequency injection speed loop | Sensorless (HFI) | Zero/low speed operation |
| **8** | `MODE_Sensorless_SMO` | Sliding mode observer speed loop | Sensorless (SMO) | Medium/high speed operation |
| **9** | `MODE_Sensorless_NLFO` | Nonlinear flux linkage speed loop | Sensorless (NLFO) | Medium/high speed operation |
| **10** | `MODE_Sensorless_HS` | HFI+SMO combined speed loop | Sensorless (HFI+SMO) | Smooth full-speed switching |
| **11** | `MODE_Sensorless_HN` | HFI+NLFO combined speed loop | Sensorless (HFI+NLFO) | Smooth full-speed switching |
| **12** | `MODE_Sensorless_AS` | Hall+SMO combined speed loop | Hall + SMO | Full-speed fusion control |
| **13** | `MODE_Sensorless_AN` | Hall+NLFO combined speed loop | Hall + NLFO | Full-speed fusion control |
| **14** | `MODE_Debug_HFI` | HFI test speed loop | Encoder (external observation) | HFI algorithm debugging |
| **15** | `MODE_Debug_SMO` | SMO test speed loop | Encoder (external observation) | SMO algorithm debugging |
| **16** | `MODE_Debug_NLFO` | NLFO test speed loop | Encoder (external observation) | NLFO algorithm debugging |
| **17** | `MODE_Debug_HS` | HFI to SMO speed loop | Encoder (fusion test) | Observer switching debugging |
| **18** | `MODE_Debug_HN` | HFI to NLFO speed loop | Encoder (fusion test) | Observer switching debugging |

Modify the macro definition in `UserData_Config.h`:

```c
// Select the mode you need (0-18)
#define Define_Run_Mode 11   // Example: Full-speed HFI+NLFO sensorless control
```

| Your Need | Recommended Mode |
|:---------|:---------|
| Just got the motor, want to spin it quickly | `0` or `1` (Open-loop forced start) |
| Winding machine torque control | `3` (Single current loop) |
| Self-balancing vehicle, hub motor | `4` (Speed-current double loop) |
| Gimbal, robotic arm, CNC, 3D printer | `5` (Position-velocity-current three loops) |
| Low-cost project (using Hall sensor) | `6` (Sensored Hall) |
| Drone, high-speed fan (sensorless) | `11` (Full-speed HFI+NLFO) |
| Algorithm research, observer tuning | `14` - `18` (Debug modes) |

---

## 6. Open-Source Hardware by Shangguan

> *"Schematic. PCB. Your hardware runs out of the box."*

![PCB Board Diagram](https://cdn.jsdelivr.net/gh/Sguan-ZhouQing/SguanFOC_Library@main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Image/PCB_CH32.jpg)

---

## 7. 📁 Repository Structure

```
SguanFOC_Library/
├── SguanFOC.c/h                # Core framework
├── Sguan_math.c/h              # Mathematical operations + coordinate transforms
├── Sguan_IQmath.c/h            # Fixed-point library
├── Sguan_PID.c/h               # PID controller
├── Sguan_Ladrc.c/h             # LADRC active disturbance rejection
├── Sguan_SMC.c/h               # Sliding mode control
├── Sguan_STA.c/h               # Super-twisting sliding mode
├── Sguan_PLL.c/h               # Phase-locked loop
├── Sguan_Filter.c/h            # Butterworth/Chebyshev/Bessel filters
├── Sguan_SVPWM.c/h             # SVPWM modulation
├── Sguan_SPWM.c/h              # SPWM + third harmonic injection
├── Sguan_HFI.c/h               # High-frequency injection
├── Sguan_SMO.c/h               # Sliding mode observer
├── Sguan_NLFO.c/h              # Nonlinear flux linkage observer
├── Sguan_NSD.c/h               # Polarity identification
├── Sguan_DOB.c/h               # Disturbance observer
├── Sguan_Optimize.c/h          # MTPA/Field Weakening/Dead-time/Angle compensation
├── Sguan_Cogging.c/h           # Anti-cogging calibration
├── Sguan_Identify.c/h          # Parameter identification
├── Sguan_printf.c/h            # JustFloat communication
├── Sguan_MotorStatus.c/h       # State machine
├── Sguan_Feedforward.c/h       # Feedforward stage
├── Sguan_Hall.c/h              # Triple Hall signal processing
├── UserData_Config.h           # (User) configuration switches
├── UserData_Function.h         # (User) hardware interface
├── UserData_Motor.h            # (User) motor parameters
├── UserData_Parameter.h        # (User) controller parameters
├── UserData_Status.h           # (User) state machine callbacks
└── UserData_UserControl.h      # (User) command processing
```

👉 [**Complete API Operation Documentation**](https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Sguan%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf)

---

## 8. License

This project is open-sourced under the **MIT License**.

You are free to **use, modify, and distribute** this codebase, **including for commercial purposes** — internal company use, customer project delivery, selling as a paid product — all without restrictions. No prior authorization required, no fees.

The MIT license only requires retaining the copyright notice. It does not mandate attribution, though we welcome being informed of your use case.

> 📄 See [LICENSE](LICENSE) for details

---

## 9. Connect with the Source Code Author

Xingbichen Sguan is an embedded algorithm engineer, open-source hardware enthusiast, and independent developer. Representative work: SguanFOC open-source project (GitHub FOC algorithm library). Shares motor control, embedded development, and open-source hardware technology across platforms.

| Platform | Account | Link |
|---|---|---|
| GitHub | Sguan-ZhouQing | https://github.com/Sguan-ZhouQing |
| Bilibili | Xingbichen Sguan | [https://space.bilibili.com/564956515](https://space.bilibili.com/564956515) |
| TikTok | Xingbichen Sguan | TikTok ID: dy9q15ail4xc |
| Fan Group | [Sguan(^^)] Water Friends Happy House | Group ID: 716279199 |
| Email | Technical Communication | 3464647102@qq.com |

Technical inquiries, project collaboration → Contact via platforms above or email.

<p align="center">
  <b>SguanFOC - Making Motor Control Simpler</b> 🚀
</p>

<p align="center">
  <img src="https://img.shields.io/badge/⭐-Whether%20you%20are%20an%20expert%20or%20beginner%2C%20if%20this%20motor%20control%20code%20project%20helps%20you%2C%20please%20give%20us%20a%20Star!-brightgreen">
</p>
