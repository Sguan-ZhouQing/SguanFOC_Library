#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算Config */

/**
 * @description: 宏定义0-10决定“电机的控制模式”(默认使用“速度-电流串级闭环控制”模式)
 * @reminder: 0->MODE_VF_OPENLOOP       VF压频比开环        (开环强拖)
 * @reminder: 1->MODE_IF_OPENLOOP       IF流频比开环        (开环强拖)
 * @reminder: 2->MODE_Voltag_OPEN       电压开环            (高精度编码器提供Rad)
 * @reminder: 3->MODE_Current_SINGLE    电流单闭环          (高精度编码器提供Rad)
 * @reminder: 4->MODE_VelCur_DOUBLE     速度-电流串级闭环   (高精度编码器提供Rad)
 * @reminder: 5->MODE_PosVelCur_THREE   位置-速度-电流三环  (高精度编码器提供Rad)
 * @reminder: 6->MODE_Sensor_Hall       有感霍尔_转速环     (三霍尔编码器提供Rad)
 * @reminder: 7->MODE_Sensorless_HFI    高频注入_转速环     (低速域，速度有上限)
 * @reminder: 8->MODE_Sensorless_SMO    滑模观测_转速环     (高速域，IF切SMO)
 * @reminder: 9->MODE_Sensorless_HS     前两结合_转速环     (全速域，HFI切SMO)
 * @reminder: 10->MODE_Sensorless_AS    霍尔滑模结合_转速环  (全速域，霍尔切SMO)
 * @return {*}
 */
#define Define_Run_Mode 8

/**
 * @description: 宏定义0-3决定“无感算法的debug模式”是否开启(默认关闭)
 * @reminder: 0->Debug_NULL             关闭无感算法的dubug跟随，纯有感控制
 * @reminder: (开启debug...支持有感模式下，旁接无感模块观测调试)
 * @reminder: 1->Debug_HFI              低速域，纯HFI高频注入算法
 * @reminder: 2->Debug_SMO              高速域，纯SMO滑模观测器
 * @reminder: 3->Debug_HS               全速域，“高频注入”切“滑模观测”
 * @return {*}
 */
#define Define_Run_Debug 0

/**
 * @description: 宏定义0-3决定“电机速度环”的控制方式(默认使用PI控制)
 * @reminder: 0->Control_PID            电流环“PI控制”，转速环“PI控制”
 * @reminder: 1->Control_LADRC          电流环“PI控制”，转速环“Ladrc线性自抗扰控制”
 * @reminder: 2->Control_SMC            电流环“PI控制”，转速环“SMC传统滑模控制”
 * @reminder: 3->Control_STA            电流环“PI控制”，转速环“STA超螺旋滑模控制”
 * @return {*}
 */
#define Switch_MOTOR_Control_Vel 0

/**
 * @description: 宏定义0-3决定“电机位置环”的控制方式(默认使用PD控制)
 * @reminder: 0->Control_PID            位置环采用“PD控制” 
 * @reminder: 1->Control_LADRC          位置环采用“Ladrc线性自抗扰控制”
 * @reminder: 2->Control_SMC            位置环采用“SMC传统滑模控制”
 * @reminder: 3->Control_STA            位置环采用“STA超螺旋滑模控制”
 * @return {*}
 */
#define Switch_MOTOR_Control_Pos 0

/**
 * @description: 宏定义0-1决定“电机矢量控制底层算法”(默认使用SVPWM)
 * @reminder: 0->使用七段式的SVPWM空间矢量合成的电机控制技术
 * @reminder: 1->使用带“三次谐波注入”优化后的SPWM脉宽调制技术
 * @return {*}
 */
#define Switch_MOTOR_PWM 0

/**
 * @description: 宏定义0-2决定“滤波算法使用上的选择”(默认使用二阶巴特沃斯滤波器)
 * @reminder: 0->Filter_ButterWorth     Butter二阶巴特沃斯滤波器
 * @reminder: 1->Fitler_ChebyShev       ChebyShev切比雪夫二阶I型
 * @reminder: 2->Fitler_Bessel          Bessel二阶贝塞尔滤波器
 * @return {*}
 */
#define Switch_MOTOR_Filter 0

/**
 * @description: 宏定义0-2决定“电机参数辨识”额外数据的测量(默认关闭，首次上电需要1)
 * @reminder: 0->不开启电机的参数辨识
 * @reminder: 1->开启辨识，只执行基础的Rs、Ld、Lq和Sguan.identify.Encoder_Dir
 * @reminder: 2->开启辨识，执行所有辨识，包括额外的B、J和Flux...需调好电机闭环回路才可开启
 * @reminder: （所有任务之前，可提前填入粗略的参数，让电机勉强运行起来）
 * @reminder: （选择2号宏定义，需调好电机闭环回路才可开启...会占用电机运行时间）
 * @return {*}
 */
#define Switch_MOTOR_Identify 0

/**
 * @description: 宏定义0-2决定“电机启动定位方式的选择”(默认关闭)
 * @reminder: 0->不开启电机定位，有感绝对坐标值已经填写好了“定位信息”
 * @reminder: 1->开启电机定位，强拖D轴定位，电机会动一下
 * @reminder: 2->开启电机定位，高频注入定位，电机可以免晃动
 * @return {*}
 */
#define Switch_MOTOR_Start 0

/**
 * @description: 宏定义0或1决定“AngleComp相位延迟补偿”是否开启(默认关闭)
 * @reminder: 0->不开启系统的相位延迟补偿
 * @reminder: 1->开启相位延迟补偿，恒定延迟补偿，固定Td补偿算法
 * @return {*}
 */
#define Open_AngleComp_Calculate 0

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
#define Open_DOB_Calculate 0

/**
 * @description: 宏定义0或1决定“谐波抑制算法”是否开启(开启最优)
 * @reminder: 0->不开启电机谐波抑制
 * @reminder: 1->开启谐波抑制(陷波滤波器)，滤除电机5、7次主要谐波
 * @return {*}
 */
#define Open_Inhibit_Calculate 1

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
#define Open_FW_Calculate 0

/**
 * @description: 宏定义0或1决定“DeadZone死区补偿”是否开启(开启最优)
 * @reminder: 0->不开启MOS或者IGBT的死区补偿
 * @reminder: 1->开启DeadZone死区补偿算法(电流方向的前馈补偿)
 * @return {*}
 */
#define Open_DeadZone_Calculate 0

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Open_Printf_Debug)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Open_Printf_Debug 0

/**
 * @description: 宏定义0或1决定“抗齿槽算法(离线标定补偿)”是否开启(默认关闭)
 * @reminder: 0->不开启抗齿槽算法的离线标定补偿
 * @reminder: 1->开启抗齿槽算法，基于绝对位置下的iq输入
 * @reminder: （补偿标定需用到调好控制器的位置环）
 * @reminder: （过程中十分耗时...想要等待一定时间）
 * @return {*}
 */
#define Open_Cogging_Calculate 0

/**
 * @description: 宏定义数值决定“抗齿槽标定的16位Q15定点的标幺化基值”(默认8安培)
 * @reminder: （标定1800个点位，使用float转16位的Q15存储，占用3.6kb内存）
 * @reminder: （采用位置环定位，采集不同位置下的速度环输出Target_iq的数值）
 * @return {*}
 */
#define BASE_Cogging_Num 8.0f


// 定时器中断参数设计
#define TIM_T 5e-5                          // 最大频率的控制周期


#endif // USERDATA_CONFIG_H
