#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0-3决定“电机速度环”的控制方式(默认使用PI控制)
 * @reminder: 0->电流环“PI控制”，转速环“PI控制”
 * @reminder: 1->电流环“PI控制”，转速环“Ladrc线性自抗扰控制”
 * @reminder: 2->电流环“PI控制”，转速环“SMC传统滑模控制”
 * @reminder: 3->电流环“PI控制”，转速环“STA超螺旋二阶滑模控制”
 * @return {*}
 */
#define Switch_Control_Velocity 0

/**
 * @description: 宏定义0-3决定“电机位置环”的控制方式(默认使用PD控制)
 * @reminder: 0->位置环“PD控制” 
 * @reminder: 1->位置环“Ladrc线性自抗扰控制”
 * @reminder: 2->位置环“SMC传统滑模控制”
 * @reminder: 3->位置环“STA超螺旋二阶滑模控制”
 * @return {*}
 */
#define Switch_Control_Position 0

/**
 * @description: 宏定义0或1决定“电机矢量控制底层算法”(默认使用SVPWM)
 * @reminder: 0->使用七段式的SVPWM空间矢量合成的电机控制技术
 * @reminder: 1->使用带“三次谐波注入”优化后的SPWM脉宽调制技术
 * @return {*}
 */
#define Switch_PWM_Calculate 0

/**
 * @description: 宏定义0或1决定“电机参数辨识”额外数据的测量(默认关闭，首次上电需要1)
 * @reminder: 0->不开启电机的参数辨识
 * @reminder: 1->开启辨识，只执行基础的Rs、Ld、Lq和Sguan.identify.Encoder_Dir
 * @reminder: 2->开启辨识，执行所有辨识，包括额外的B、J和Flux...需调好电机闭环回路才可开启
 * @reminder: （所有任务之前，可提前填入粗略的参数，让电机勉强运行起来）
 * @reminder: （选择2号宏定义，需调好电机闭环回路才可开启...会占用电机运行时间）
 * @return {*}
 */
#define Switch_MOTOR_Identify 0

/**
 * @description: 宏定义0或1决定“电流前馈”是否开启(开启最优)
 * @reminder: 0->不开启电流的前馈解耦
 * @reminder: 1->开启电流的前馈解耦
 * @return {*}
 */
#define Open_Current_Feedforward 1

/**
 * @description: 宏定义0或1决定“速度前馈”是否开启(开启最优)
 * @reminder: 0->不开启速度的有功阻尼“速度解耦”
 * @reminder: 1->开启速度的有功阻尼“速度解耦”
 * @return {*}
 */
#define Open_Velocity_Feedforward 1

/**
 * @description: 宏定义0或1决定“超螺旋滑模扰动观测器”是否开启(开启最优)
 * @reminder: 0->不开启STA_SMDO
 * @reminder: 1->开启STA_SMDO
 * @return {*}
 */
#define Open_DOB_Calculate 1

/**
 * @description: 宏定义0或1决定“DeadZone死区补偿”是否开启(开启最优)
 * @reminder: 0->不开启MOS或者IGBT的死区补偿
 * @reminder: 1->开启DeadZone死区补偿算法(电流方向的前馈补偿)
 * @return {*}
 */
#define Open_DeadZone_Calculate 1

/**
 * @description: 宏定义0或1决定“MTPA最大转矩控制控制”是否开启(默认关闭)
 * @reminder: 0->不开启IPMSM的MTPA最大转矩控制
 * @reminder: 1->开启最大转矩控制控制(凸极电机需要)
 * @return {*}
 */
#define Open_MTPA_Calculate 0

/**
 * @description: 宏定义0或1决定“FW弱磁控制”是否开启(默认关闭)
 * @reminder: 0->不开启IPMSM的弱磁控制
 * @reminder: 1->开启弱磁控制(凸极电机需要)
 * @reminder: （弱磁控制想要稳定性高些，请提前开启MTPA控制）
 * @reminder: （MTPA控制：Open_MTPA_Calculate 1）
 * @return {*}
 */
#define Open_FW_Calculate 1

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Open_Printf_Debug)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Open_Printf_Debug 0


// 定时器中断参数设计
#define TIM_T 1e-4                          // 最大频率的控制周期


#endif // USERDATA_CONFIG_H
