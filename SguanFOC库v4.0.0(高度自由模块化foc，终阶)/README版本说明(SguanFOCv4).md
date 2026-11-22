# SguanFOC库v4代码说明(Define系列)
[![Version](https://img.shields.io/badge/Version-4.0.0-blue)](https://github.com/Sguan-ZhouQing/SguanFOC_Library)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Language](https://img.shields.io/badge/Language-C-00599C)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Platform](https://img.shields.io/badge/Platform-ARM%20%7C%20DSP%20%7C%20任何C语言MCU-orange)](https://github.com/Sguan-ZhouQing/SguanFOC_Library)
[![完整FOC](https://img.shields.io/badge/🔄_完整FOC-开箱即用-red)]
## 1.UnitLib_Define0.h应用库
```c
/* (宏定义uint8_t，传递函数模块，数量设计0-n)
DEFINE0_NUM_TRANSFER1  	典型一阶传递函数
DEFINE0_NUM_TRANSFER2  	典型二阶传递函数
DEFINE0_NUM_TRANSFER3  	典型三阶传递函数
DEFINE0_NUM_TRANSFER4  	典型四阶传递函数
DEFINE0_NUM_TRANSFER5  	典型五阶传递函数
DEFINE0_NUM_INTEGRATOR 	积分器
DEFINE0_NUM_DERIVATIVE 	微分器
DEFINE0_NUM_HALL       	霍尔编码器
DEFINE0_NUM_LADRC1     	一阶线性自适应抗干扰控制
DEFINE0_NUM_LADRC2     	二阶线性自适应抗干扰控制
DEFINE0_NUM_SMC        	传统指数型趋近率的滑模控制
DEFINE0_NUM_DPCC       	增量式电流预测控制
DEFINE0_NUM_PIR        	比例积分谐振调节器
DEFINE0_NUM_PID        	传统闭环控制器
DEFINE0_NUM_PLL        	开环锁相环
DEFINE0_NUM_LPF1       	一阶低通滤波器
DEFINE0_NUM_LPF2       	二阶低通滤波器
DEFINE0_NUM_HPF1       	一阶高通滤波器
DEFINE0_NUM_HPF2       	二阶高通滤波器
DEFINE0_NUM_BPF1       	带通滤波器"一阶低通和高通串联"
DEFINE0_NUM_BPF2       	带通滤波器"典型二阶系统改型"
DEFINE0_NUM_SOGI       	广义积分器"典型二阶系统改型"
DEFINE0_NUM_NF         	陷波滤波器"典型二阶系统改型"
DEFINE0_NUM_TPNF       	陷波滤波器"三参数陷波滤波器"
DEFINE0_NUM_DOB        	超螺旋滑模扰动观测器
DEFINE0_NUM_RLS        	电机参数在线辨识观测器
DEFINE0_NUM_SMO        	"无感"滑模观测器
DEFINE0_NUM_NFO        	"无感"非线性磁链观测器
DEFINE0_NUM_AFO        	"无感"有效磁链观测器
DEFINE0_NUM_HFI        	"无感"高频正弦波注入
DEFINE0_NUM_ROLO       	"无感"降阶龙伯格观测器
DEFINE0_NUM_MARS       	"无感"模型参考自适应观测器
DEFINE0_NUM_EKF        	"无感"扩展卡尔曼滤波
DEFINE0_NUM_DELAY1     	延时函数"延时一拍"
DEFINE0_NUM_DELAY2     	延时函数"延时二拍"
DEFINE0_NUM_DELAY3     	延时函数"延时三拍"

(宏定义uint8_t，扩展存储单元，数量设计0-n)
DEFINE0_NUM_FLOAT      	扩展单精度浮点数
DEFINE0_NUM_DOUBLE     	扩展双精度浮点数
DEFINE0_NUM_INT8       	扩展8位有符号数
DEFINE0_NUM_UINT8      	扩展8位无符号数
DEFINE0_NUM_INT16      	扩展16位有符号数
DEFINE0_NUM_UINT16     	扩展16位无符号数
DEFINE0_NUM_INT32      	扩展32位有符号数
DEFINE0_NUM_UINT32     	扩展32位无符号数

(宏定义uint8_t，电机核心选择，功能设计)
DEFINE0_MOTOR_COUNT    	电机数量（“1-6”数量）
DEFINE0_MOTOR_MODE     	电机模式（“n”不同模式）
DEFINE0_MOTOR_MATH     	电机运算（“0-3”->float,double,Q15,Q31）

(宏定义float，定点代码基值填写，计算设计)
DEFINE0_BASE_TIME      	基值BASE归一化设计
DEFINE0_BASE_VOLTAGE   	基值BASE归一化设计
DEFINE0_BASE_CURRENT   	基值BASE归一化设计
DEFINE0_BASE_OTHER     	基值BASE归一化设计
*/
```

## 2.UnitLib_Define1.h应用库
```c
/*(宏定义void，xxx，xxx设计)
不同应用场景下，会设计不同用途的全局宏定义，这个宏就放这  
DEFINE1_开头去命名
*/
```

## 3.UserMotor_Define2.h电参库 
```c
/*

*/
```

## 4.Zone_Define3.h用户驱动层 
```c
/*(宏定义void，xxx，xxx设计)
DEFINE3_RUN_T          
*/
```


---
## SguanFOC_v4代码(①Sguan固件库🚀)
> *「Sguan固件库，UnitLib应用库，UserMotor电参库，Zone用户驱动层」*
> *"（Sguan）Firmware Library Description."*
```c
/*
Sguan_Config.h          配置文件
Sguan_Math.c/.h         数学运算
Sguan_Method.c/.h       方式方法
Sguan_Modbus.c/.h       通信协议
Sguan_MotorStatus.c/.h  电机状态
Sguan_Printf.c/.h       串口调试
Sguan_Transfer.c/.h     传函模块
Sguan_Value.h           数值宏
SguanFOC.c/.h           (固件)算法核心
*/
```

## SguanFOC_v4代码(②UnitLib应用库📁)
> *「Sguan固件库，UnitLib应用库，UserMotor电参库，Zone用户驱动层」*
> *"（UnitLib）App Library Description."*
```c
/*
UnitLib_Define0.h     	宏定义0
UnitLib_Define1.h       宏定义1
UnitLib_Main.c/.h       (应用)算法核心
UnitLib_Patch.c/.h      定点补丁
UnitLib_Situation.c/.h  状态函数
UnitLib_UserControl.c/.h调试函数
UnitLib_UserRTU.c/.h    协议函数
*/
```

## SguanFOC_v4代码(③UserMotor电参库![无刷电机](https://cdn.jsdelivr.net/gh/Sguan-ZhouQing/SguanFOC_Library@main/%E9%85%8D%E5%A5%97QT%E4%B8%8A%E4%BD%8D%E6%9C%BA%E5%8F%8AFOC%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E2%91%A0%5BPDF%5D/Image/MOTOR.svg))
> *「Sguan固件库，UnitLib应用库，UserMotor电参库，Zone用户驱动层」*
> *"（UserMotor）Motor Parameter Library Description."*
```c
/*
UserMotor_Define2.h   	宏定义2
UserMotor_Parameter.c/.h控制参数
*/
```

## SguanFOC_v4代码(④Zone用户驱动层🎯)
> *「Sguan固件库，UnitLib应用库，UserMotor电参库，Zone用户驱动层」*
> *"（Zone）User-Driven Layer Description."*
```c
/*
Zone_Define3.h       	宏定义3
Zone_Driver.c/.h        用户驱动
*/
```

