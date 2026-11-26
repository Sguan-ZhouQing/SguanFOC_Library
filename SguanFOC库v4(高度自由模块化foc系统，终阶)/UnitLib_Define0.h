/* ===================================================================
 * 文件:     UnitLib_Define0.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     2.UnitLib应用库_宏定义0
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-APACHE 文件。
 * =================================================================== */
#ifndef __UNITLIB_DEFINE0_H
#define __UNITLIB_DEFINE0_H

#define DEFINE0_NUM0_TRANSFER1  5                   // (uint8_t)典型一阶传递函数
#define DEFINE0_NUM0_TRANSFER2  2                   // (uint8_t)典型二阶传递函数
#define DEFINE0_NUM0_TRANSFER3  1                   // (uint8_t)典型三阶传递函数
#define DEFINE0_NUM0_TRANSFER4  1                   // (uint8_t)典型四阶传递函数
#define DEFINE0_NUM0_TRANSFER5  1                   // (uint8_t)典型五阶传递函数
#define DEFINE0_NUM0_INTEGRATOR 1                   // (uint8_t)积分器
#define DEFINE0_NUM0_DERIVATIVE 1                   // (uint8_t)微分器
#define DEFINE0_NUM0_HALL       1                   // (uint8_t)霍尔编码器
#define DEFINE0_NUM0_LADRC1     1                   // (uint8_t)一阶线性自适应抗干扰控制
#define DEFINE0_NUM0_LADRC2     1                   // (uint8_t)二阶线性自适应抗干扰控制
#define DEFINE0_NUM0_SMC        1                   // (uint8_t)传统指数型趋近率的滑模控制
#define DEFINE0_NUM0_DPCC       1                   // (uint8_t)增量式电流预测控制
#define DEFINE0_NUM0_PIR        1                   // (uint8_t)比例积分谐振调节器
#define DEFINE0_NUM0_PID        1                   // (uint8_t)传统闭环控制器
#define DEFINE0_NUM0_PLL        1                   // (uint8_t)开环锁相环
#define DEFINE0_NUM0_LPF1       1                   // (uint8_t)一阶低通滤波器
#define DEFINE0_NUM0_LPF2       1                   // (uint8_t)二阶低通滤波器
#define DEFINE0_NUM0_HPF1       1                   // (uint8_t)一阶高通滤波器
#define DEFINE0_NUM0_HPF2       1                   // (uint8_t)二阶高通滤波器
#define DEFINE0_NUM0_BPF1       1                   // (uint8_t)带通滤波器"一阶低通和高通串联"
#define DEFINE0_NUM0_BPF2       1                   // (uint8_t)带通滤波器"典型二阶系统改型"
#define DEFINE0_NUM0_SOGI       1                   // (uint8_t)广义积分器"典型二阶系统改型"
#define DEFINE0_NUM0_NF         1                   // (uint8_t)陷波滤波器"典型二阶系统改型"
#define DEFINE0_NUM0_TPNF       1                   // (uint8_t)陷波滤波器"三参数陷波滤波器"
#define DEFINE0_NUM0_DOB        1                   // (uint8_t)超螺旋滑模扰动观测器
#define DEFINE0_NUM0_RLS        1                   // (uint8_t)电机参数在线辨识观测器
#define DEFINE0_NUM0_SMO        1                   // (uint8_t)"无感"滑模观测器
#define DEFINE0_NUM0_NFO        1                   // (uint8_t)"无感"非线性磁链观测器
#define DEFINE0_NUM0_AFO        1                   // (uint8_t)"无感"有效磁链观测器
#define DEFINE0_NUM0_HFI        1                   // (uint8_t)"无感"高频正弦波注入
#define DEFINE0_NUM0_ROLO       1                   // (uint8_t)"无感"降阶龙伯格观测器
#define DEFINE0_NUM0_MARS       1                   // (uint8_t)"无感"模型参考自适应观测器
#define DEFINE0_NUM0_EKF        1                   // (uint8_t)"无感"扩展卡尔曼滤波
#define DEFINE0_NUM0_DELAY1     1                   // (uint8_t)延时函数"延时一拍"
#define DEFINE0_NUM0_DELAY2     1                   // (uint8_t)延时函数"延时二拍"
#define DEFINE0_NUM0_DELAY3     1                   // (uint8_t)延时函数"延时三拍"

#define DEFINE0_NUM1_SGUANQ      1                  // (uint8_t)扩展SguanQ浮点数
#define DEFINE0_NUM1_INT8       1                   // (uint8_t)扩展8位有符号数
#define DEFINE0_NUM1_UINT8      1                   // (uint8_t)扩展8位无符号数
#define DEFINE0_NUM1_INT16      1                   // (uint8_t)扩展16位有符号数
#define DEFINE0_NUM1_UINT16     1                   // (uint8_t)扩展16位无符号数
#define DEFINE0_NUM1_INT32      1                   // (uint8_t)扩展32位有符号数
#define DEFINE0_NUM1_UINT32     1                   // (uint8_t)扩展32位无符号数

// 1-6
// 0-n
// 宏定义(0指float，1指double，2指Q15，3指Q31)
#define DEFINE0_MOTOR_COUNT     1                   // (uint8_t)
#define DEFINE0_MOTOR_MODE      0                   // (uint8_t)
#define DEFINE0_MOTOR_MATH      0                   // (uint8_t)

#define DEFINE0_BASE_TIME       0.001953125         // (double)
#define DEFINE0_BASE_VOLTAGE    1024.0              // (double)
#define DEFINE0_BASE_CURRENT    128.0               // (double)
#define DEFINE0_BASE_OTHER      512.0               // (double)


#endif // UNITLIB_DEFINE0_H
