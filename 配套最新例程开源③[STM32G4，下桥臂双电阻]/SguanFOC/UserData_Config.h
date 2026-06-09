#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算Config */

/**
 * @description: 宏定义0-10决定“电机的控制模式”(默认使用“速度-电流串级闭环控制”模式)
 * @reminder: 0->MODE_VF_OPENLOOP       VF压频比开环        (开环强拖)
 * @reminder: 1->MODE_IF_OPENLOOP       IF流频比开环        (开环强拖)
 * @reminder: 2->MODE_Voltag_OPEN       电压开环            (高精度编码器提供Rad)
 * @reminder: 3->MODE_Current_SINGLE    电流单闭环          (高精度编码器提供Rad)
 * @reminder: 4->MODE_VelCur_DOUBLE     速度-电流串级闭环    (高精度编码器提供Rad)
 * @reminder: 5->MODE_PosVelCur_THREE   位置-速度-电流三环   (高精度编码器提供Rad)
 * @reminder: 6->MODE_Sensor_Hall       有感霍尔_转速环      (三霍尔编码器提供Rad)
 * @reminder: 7->MODE_Sensorless_HFI    高频注入_转速环      (低速域，速度有上限)
 * @reminder: 8->MODE_Sensorless_SMO    滑模观测_转速环      (高速域，IF切SMO)
 * @reminder: 9->MODE_Sensorless_NLFO   非线性磁链_转速环    (高速域，IF切NLFO)
 * @reminder: 10->MODE_Sensorless_HS    高频滑模结合_转速环  (全速域，HFI切SMO)
 * @reminder: 11->MODE_Sensorless_HN    高频磁链结合_转速环  (全速域，HFI切NLFO)
 * @reminder: 12->MODE_Sensorless_AS    霍尔滑模结合_转速环  (全速域，霍尔切SMO)
 * @reminder: 13->MODE_Sensorless_AN    霍尔磁链结合_转速环  (全速域，霍尔切NLFO)
 * @word: (在此以上是电机正常运行模式，在此以下是有感速度环，外载“无感观测器”测试的模式)
 * @reminder: 14->MODE_Debug_HFI        HFI测试_转速环      (高精度编码器提供Rad)
 * @reminder: 15->MODE_Debug_SMO        SMO测试_转速环      (高精度编码器提供Rad)
 * @reminder: 16->MODE_Debug_NLFO       NLFO测试_转速环     (高精度编码器提供Rad)
 * @reminder: 17->MODE_Debug_HS         HFI切SMO_转速环     (高精度编码器提供Rad)
 * @reminder: 18->MODE_Debug_HN         HFI切NLFO_转速环    (高精度编码器提供Rad)
 * @return {*}
 */
#define Define_Run_Mode 4

/**
 * @description: 宏定义0-3决定“电机速度环”的控制方式(默认使用PI控制)
 * @reminder: 0->Control_PID            电流环“PI控制”，转速环“PI控制”
 * @reminder: 1->Control_LADRC          电流环“PI控制”，转速环“Ladrc线性自抗扰控制”
 * @reminder: 2->Control_SMC            电流环“PI控制”，转速环“SMC传统滑模控制”
 * @reminder: 3->Control_STA            电流环“PI控制”，转速环“STA超螺旋滑模控制”
 * @return {*}
 */
#define Switch_MOTOR_Control_Vel 1

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
 * @reminder: 1->开启辨识，只执行基础的Rs、Ls（适用于SPMSM表贴电机，实际测Ld）
 * @reminder: 2->开启辨识，只执行基础的Rs、Ld、Lq（需要电机与定位0度后，锁定电机测电感）
 * @reminder: 3->开启辨识，单独的Flux。J和B暂时还无法实现...
 * @reminder: （所有任务之前，可提前填入粗略的参数，让电机勉强运行起来）
 * @reminder: （之所以单独测，是因为Flux想要电机空载旋转，而和锁定电机测电感相悖）
 * @reminder: （选择3号宏定义，需调好角度闭环回路才可开启...会占用电机运行时间）
 * @return {*}
 */
#define Switch_MOTOR_Identify 0

/**
 * @description: 宏定义0-2决定“电机启动定位方式的选择”(默认关闭)
 * @reminder: 0->不开启电机定位，有感绝对坐标值已经填写好了“定位信息”
 * @reminder: 1->开启电机定位，强拖D轴定位，电机会动一下
 * @reminder: （这个是有感模式的启动方式...无感默认对应的启动方式）
 * @reminder: （有感包含正常的闭环模式和三霍尔信号的有感控制）
 * @reminder: （比如SMO是IF启动，含高频注入HFI则是注入辨识启动）
 * @return {*}
 */
#define Switch_MOTOR_Start 0

/**
 * @description: 宏定义0-2决定UART或者CAN发送数据的模式
 * @reminder: 0->发送正常数据(数据格式是JustFloat)
 * @reminder: 1->仅发送Status数据，电机状态变换会记录下来
 * @reminder: 2->仅发送Cogging的离线标定抗齿槽数据
 * @reminder: （宏定义2号这样做，可以把数据记录下来）
 * @reminder: （平常抗齿槽的iq_tab是放置在单片机的RAW中的...）
 * @reminder: （把记录下的数据自己填入，并在iq_tab前加入const）
 * @reminder: （那么iq_tab则会存入Flash，RAW的内存占用就少了）
 * @return {*}
 */
#define Switch_Printf_Debug 0

/**
 * @description: 宏定义0或1决定“抗齿槽算法(离线标定补偿)”是否开启(默认关闭)
 * @reminder: 0->不开启抗齿槽算法的离线标定补偿
 * @reminder: 1->开启抗齿槽算法的离线标定补偿->仅开启使用
 * @reminder: 2->开启抗齿槽算法的离线标定补偿->测量并使用
 * @reminder: （基于绝对位置下的iq输入）
 * @reminder: （补偿标定需用到调好控制器的位置环）
 * @reminder: （过程中十分耗时...想要等待一定时间）
 * @return {*}
 */
#define Switch_Cogging_Calculate 0

/**
 * @description: 宏定义0或1决定“AngleComp相位延迟补偿”是否开启(默认关闭)
 * @reminder: 0->不开启系统的相位延迟补偿
 * @reminder: 1->开启相位延迟补偿，恒定延迟补偿，固定Td和Offset补偿算法
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
#define Open_Velocity_Feedforward 0

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
 * @description: 宏定义数值决定“抗齿槽标定的16位Q15定点的标幺化基值”(默认8安培)
 * @reminder: （标定1800个点位，使用float转16位的Q15存储，占用3.6kb内存）
 * @reminder: （采用位置环定位，采集不同位置下的速度环输出Target_iq的数值）
 * @return {*}
 */
#define BASE_Cogging_Num 8.0f


// 定时器中断参数设计
#define TIM_T 5e-5                          // 最大频率的控制周期


#endif // USERDATA_CONFIG_H
