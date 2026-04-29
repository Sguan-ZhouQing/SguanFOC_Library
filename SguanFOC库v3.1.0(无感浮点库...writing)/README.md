SguanFOC核心控制参数就是"已被声明为全局变量的Sguan结构体"，用户可以打印Sguan后面的数据
查看电机运行参数，或者修改里面内容控制电机。以下是SguanFOCv3.1.0版本的结构体含义->
(结构体中分为三种内容:结构体，变量，函数)
(结构体:不能单独使用，本身是被嵌套的)
(变量:单一数据参数，可以查看或者修改)
(函数:单一控制函数，可被随时调用使用)
// ================================== 分割线 ===================================
①第一层级(全部):
Sguan                                   (结构体) SguanFOC核心结构体，包含所有电控参数、状态和函数接口
// ================================== 分割线 ===================================
②第二层级(全部):
Sguan.status                            (变量) 电机当前运行状态，参考MOTOR_STATUS_xxx枚举值
Sguan.mode                              (变量) 电机当前控制模式，参考VF_OPENLOOP_MODE等宏定义

Sguan.transfer                          (结构体) 传递函数控制器，包含PID、LADRC、SMC、滤波器、PLL等算法
Sguan.encoder                           (结构体) 编码器数据处理，存储角度、角速度、电角度等实时数据
Sguan.current                           (结构体) 电流采样数据处理，存储Id/Iq/Ia/Ib/Ic及ADC偏置校准数据
Sguan.foc                               (结构体) FOC核心数据，目标值、电压给定、PWM占空比、正余弦值等
Sguan.motor                             (结构体) 电机实体参数，存储极对数、母线电压、采样电阻、电流方向等
Sguan.safe                              (结构体) 安全保护参数，存储过压、欠压、过温、过流等保护阈值
Sguan.flag                              (结构体) 运行标志位，存储PWM计算状态、看门狗计数、中断互斥锁等
Sguan.txdata                            (结构体) 数据发送缓冲区，用于串口/CAN向上位机发送调试数据

Sguan.Func_Start                        (函数) 启动电机，将状态机切换到未初始化状态
Sguan.Func_Stop                         (函数) 停止电机，将状态机切换到待机状态
Sguan.Func_Set_Mode                     (函数) 设置电机控制模式，参数为模式宏定义(0x00-0x09)
Sguan.Func_Set_Uq                       (函数) 设置Q轴电压给定值，用于VF开环控制
Sguan.Func_Set_Iq                       (函数) 设置Q轴电流给定值，用于电流闭环或力矩控制
Sguan.Func_Set_Velocity                 (函数) 设置目标转速(机械角速度rad/s)，用于速度闭环控制
Sguan.Func_Set_Position                 (函数) 设置目标位置(机械角度rad)，用于位置伺服控制
Sguan.Func_Set_TXdata                   (函数) 设置要发送的调试数据，通道0-11对应12个float数据
// ================================== 分割线 ===================================
③第三层级(全部):
Sguan.transfer.Current_D                (结构体) D轴电流环PID控制器参数，用于控制励磁电流
Sguan.transfer.Current_Q                (结构体) Q轴电流环PID控制器参数，用于控制转矩电流
Sguan.transfer.Velocity                 (结构体) 速度环控制器参数，支持PID/LADRC/SMC/STA四种算法
Sguan.transfer.Position                 (结构体) 位置环控制器参数，支持PID/LADRC/SMC/STA四种算法
Sguan.transfer.Response                 (变量) 响应带宽倍数，用于调整速度环和位置环的执行频率比例
Sguan.transfer.LPF_D                    (结构体) D轴电流低通滤波器，用于滤除Id采样噪声
Sguan.transfer.LPF_Q                    (结构体) Q轴电流低通滤波器，用于滤除Iq采样噪声
Sguan.transfer.LPF_encoder              (结构体) 速度信号低通滤波器，用于滤除转速反馈噪声
Sguan.transfer.Hall                     (结构体) 霍尔传感器信号处理，将三路霍尔信号转换为角度值
Sguan.transfer.PLL                      (结构体) 锁相环角度跟踪器，用于平滑编码器角度和计算角速度
Sguan.transfer.DOB                      (结构体) 超螺旋滑模扰动观测器，用于估计负载扰动并前馈补偿
Sguan.transfer.FW                       (结构体) 弱磁控制PI控制器，用于电压饱和时的弱磁电流调节
Sguan.transfer.BaseSpeed_fw             (变量) 弱磁基速设计点，用于MTPA到弱磁区的过渡切换
Sguan.transfer.Percentage_fw            (变量) 弱磁调制线占比，一般设计0.92，用于判断电压饱和阈值
Sguan.transfer.Beta_ff                  (变量) 转速环有功阻尼系数，用于速度前馈补偿提高稳定性
Sguan.transfer.DeadTime                 (变量) 死区时间参数，用于死区补偿算法计算补偿量
Sguan.transfer.Dead_CurMin              (变量) 死区补偿最小电流阈值，低于此值不进行补偿

Sguan.encoder.Real_Speed                (变量) 实际机械角速度(rad/s)，经过滤波后的转速反馈值
Sguan.encoder.Real_Pos                  (变量) 实际机械角度(rad)，多圈累加角度值
Sguan.encoder.Real_Rad                  (变量) 实际机械角度(rad)，单圈原始角度值(0-2π)
Sguan.encoder.Real_Erad                 (变量) 实际电角度(rad)，机械角度乘以极对数后归一化
Sguan.encoder.Real_We                   (变量) 实际电角速度(rad/s)，机械角速度乘以极对数
Sguan.encoder.Pos_offset                (变量) 编码器角度偏置，用于校准电机零位

Sguan.current.Real_Id                   (变量) 实际D轴电流(A)，经Park变换和滤波后的励磁电流
Sguan.current.Real_Iq                   (变量) 实际Q轴电流(A)，经Park变换和滤波后的转矩电流
Sguan.current.Real_Ia                   (变量) A相相电流(A)，ADC采样并校准后的原始相电流
Sguan.current.Real_Ib                   (变量) B相相电流(A)，ADC采样并校准后的原始相电流
Sguan.current.Real_Ic                   (变量) C相相电流(A)，通过Ia+Ib+Ic=0计算得出
Sguan.current.Real_Ialpha               (变量) Clark变换后的α轴电流(A)
Sguan.current.Real_Ibeta                (变量) Clark变换后的β轴电流(A)
Sguan.current.Final_Gain                (变量) ADC电流采样最终增益，用于将ADC原始值转换为电流值
Sguan.current.Current_offset0           (变量) 电流采样通道0的ADC偏置值，初始化时读取
Sguan.current.Current_offset1           (变量) 电流采样通道1的ADC偏置值，初始化时读取

Sguan.foc.Target_Speed                  (变量) 目标机械角速度(rad/s)，速度闭环的期望值
Sguan.foc.Target_Pos                    (变量) 目标机械角度(rad)，位置伺服的期望值
Sguan.foc.Target_Id                     (变量) 目标D轴电流(A)，通常设为0或MTPA计算值
Sguan.foc.Target_Iq                     (变量) 目标Q轴电流(A)，速度环或力矩环的输出
Sguan.foc.Ud_in                         (变量) D轴电压给定值(V)，经电流环PI计算后输出
Sguan.foc.Uq_in                         (变量) Q轴电压给定值(V)，经电流环PI计算后输出
Sguan.foc.Du                            (变量) U相占空比归一化值(0-1)
Sguan.foc.Dv                            (变量) V相占空比归一化值(0-1)
Sguan.foc.Dw                            (变量) W相占空比归一化值(0-1)
Sguan.foc.sine                          (变量) 当前电角度的正弦值，用于Park/Clarke变换
Sguan.foc.cosine                        (变量) 当前电角度的余弦值，用于Park/Clarke变换
Sguan.foc.Real_VBUS                     (变量) 实际母线电压(V)，用于电压限幅和弱磁判断
Sguan.foc.Real_Temp                     (变量) 实际驱动器温度(℃)，用于过温保护判断

Sguan.motor.identify                    (结构体) 电机参数辨识结果，存储Rs、Ld、Lq、Flux、B、J等
Sguan.motor.Poles                       (变量) 电机极对数，用于机械角度与电角度的转换
Sguan.motor.VBUS                        (变量) 母线电压额定值(V)，当无法读取实际电压时使用
Sguan.motor.Motor_Dir                   (变量) 电机运行方向，1为正向，-1为反向(通过交换AB相实现)
Sguan.motor.PWM_Dir                     (变量) PWM占空比极性，1为正逻辑，-1为反逻辑
Sguan.motor.Duty                        (变量) PWM满占空比计数值，取决于定时器周期
Sguan.motor.Current_Dir0                (变量) 电流采样通道0方向，1或-1用于校正电流方向
Sguan.motor.Current_Dir1                (变量) 电流采样通道1方向，1或-1用于校正电流方向
Sguan.motor.Current_Num                 (变量) 电流采样通道选择，0:AB相，1:AC相，2:BC相
Sguan.motor.ADC_Precision               (变量) ADC采样精度，12位ADC为4096
Sguan.motor.Amplifier                   (变量) 电流采样运放放大倍数
Sguan.motor.MCU_Voltage                 (变量) ADC基准电压(V)，通常为3.3V或5V
Sguan.motor.Sampling_Rs                 (变量) 电流采样电阻阻值(Ω)

Sguan.safe.VBUS_MAX                     (变量) 母线电压过压保护阈值(V)，超过则进入过压保护
Sguan.safe.VBUS_MIM                     (变量) 母线电压欠压保护阈值(V)，低于则进入欠压保护
Sguan.safe.VBUS_watchdog_limit          (变量) 电压异常看门狗周期，连续检测次数
Sguan.safe.Temp_MAX                     (变量) 驱动器过温保护阈值(℃)，超过则进入过温保护
Sguan.safe.Temp_MIN                     (变量) 驱动器低温保护阈值(℃)，低于则进入低温保护
Sguan.safe.Temp_watchdog_limit          (变量) 温度异常看门狗周期，连续检测次数
Sguan.safe.Dcur_MAX                     (变量) D轴电流过流保护阈值(A)，超过则进入过流保护
Sguan.safe.Qcur_MAX                     (变量) Q轴电流过流保护阈值(A)，超过则进入过流保护
Sguan.safe.DQcur_watchdog_limit         (变量) 过流保护看门狗周期，连续检测次数
Sguan.safe.Current_limit                (变量) 电流状态机判断阈值(A)，用于判定力矩增减状态
Sguan.safe.Speed_limit                  (变量) 速度状态机判断阈值(rad/s)，用于判定加减速状态
Sguan.safe.Position_limit               (变量) 位置状态机判断阈值(rad)，用于判定位置增减状态
Sguan.safe.DISABLED_watchdog_limit      (变量) 失能状态看门狗周期，超时后自动进入待机

Sguan.flag.PWM_Calc                     (变量) PWM计算标志位，1表示正在计算PWM
Sguan.flag.PWM_watchdog_limit           (变量) PWM计算看门狗上限，超时则报PWM_CALC_FAULT错误
Sguan.flag.in_PWM_Calc_ISR              (变量) PWM计算中断互斥锁，1表示正在执行高优先级中断

Sguan.txdata.fdata                      (变量) 发送数据数组，最多12个float，用于上位机调试
Sguan.txdata.tail                       (变量) JustFloat协议帧尾，固定为0x00,0x00,0x80,0x7F
// ================================== 分割线 ===================================
④第四层级(部分省略):
Sguan.transfer.Current_D.run            (结构体) PID控制器运行时数据，包含误差、积分、微分等中间变量
Sguan.transfer.Current_D.Wc             (变量) PID微分环节一阶低通滤波截止频率(rad/s)
Sguan.transfer.Current_D.T              (变量) PID离散运行周期(s)，电流环通常为PMSM_RUN_T
Sguan.transfer.Current_D.Kp             (变量) PID比例增益
Sguan.transfer.Current_D.Ki             (变量) PID积分增益
Sguan.transfer.Current_D.Kd             (变量) PID微分增益
Sguan.transfer.Current_D.OutMax         (变量) PID输出上限限幅
Sguan.transfer.Current_D.OutMin         (变量) PID输出下限限幅
Sguan.transfer.Current_D.IntMax         (变量) PID积分项上限限幅，用于积分抗饱和
Sguan.transfer.Current_D.IntMin         (变量) PID积分项下限限幅，用于积分抗饱和

Sguan.transfer.Current_Q.run            (结构体) PID控制器运行时数据，包含误差、积分、微分等中间变量
Sguan.transfer.Current_Q.Wc             (变量) PID微分环节一阶低通滤波截止频率(rad/s)
Sguan.transfer.Current_Q.T              (变量) PID离散运行周期(s)
Sguan.transfer.Current_Q.Kp             (变量) PID比例增益
Sguan.transfer.Current_Q.Ki             (变量) PID积分增益
Sguan.transfer.Current_Q.Kd             (变量) PID微分增益
Sguan.transfer.Current_Q.OutMax         (变量) PID输出上限限幅
Sguan.transfer.Current_Q.OutMin         (变量) PID输出下限限幅
Sguan.transfer.Current_Q.IntMax         (变量) PID积分项上限限幅
Sguan.transfer.Current_Q.IntMin         (变量) PID积分项下限限幅

Sguan.transfer.Velocity.run             (结构体) 速度环控制器运行时数据(PID/LADRC/SMC/STA共用接口)
Sguan.transfer.Velocity.T               (变量) 速度环离散运行周期(s)，等于PMSM_RUN_T乘以Response
Sguan.transfer.Position.run             (结构体) 位置环控制器运行时数据
Sguan.transfer.Position.T               (变量) 位置环离散运行周期(s)，等于PMSM_RUN_T乘以Response平方
............

Sguan.transfer.LPF_D.filter             (结构体) 二阶巴特沃斯滤波器运行时数据，包含历史输入输出
Sguan.transfer.LPF_D.Wc                 (变量) 低通滤波器截止频率(rad/s)
Sguan.transfer.LPF_D.T                  (变量) 滤波器离散运行周期(s)

Sguan.transfer.PLL.go                   (结构体) 锁相环运行时数据，包含误差、角速度、角度等
Sguan.transfer.PLL.T                    (变量) 锁相环离散运行周期(s)
Sguan.transfer.PLL.Kp                   (变量) 锁相环比增益
Sguan.transfer.PLL.Ki                   (变量) 锁相环积分增益
Sguan.transfer.PLL.is_position_mode     (变量) 位置环模式标志，1时角度可以累加超过2π

Sguan.motor.identify.pmsm               (结构体) 参数辨识状态机，存储当前辨识步骤
Sguan.motor.identify.Encoder_Dir        (变量) 编码器方向，1或-1，用于校正编码器极性
Sguan.motor.identify.Rs                 (变量) 电机相电阻(Ω)，参数辨识或手动填入
Sguan.motor.identify.Ld                 (变量) D轴电感(H)，参数辨识或手动填入
Sguan.motor.identify.Lq                 (变量) Q轴电感(H)，参数辨识或手动填入
Sguan.motor.identify.Flux               (变量) 电机磁链(Wb)，参数辨识或手动填入
Sguan.motor.identify.B                  (变量) 粘性阻尼系数(N·m·s/rad)，参数辨识或手动填入
Sguan.motor.identify.J                  (变量) 转动惯量(kg·m²)，参数辨识或手动填入
// ================================== 分割线 ===================================
⑤第五层级(部分省略):
Sguan.transfer.Current_D.run.Ref        (变量) PID期望值输入，电流环为目标电流
Sguan.transfer.Current_D.run.Fbk        (变量) PID反馈值输入，电流环为实际电流
Sguan.transfer.Current_D.run.Output     (变量) PID控制器输出值，电流环为电压给定
............
Sguan.transfer.Current_D.run.IntegralFrozen_flag (变量) 积分抗饱和标志，1表示积分已冻结
............
Sguan.transfer.Velocity.run.Ref         (变量) 速度环期望值，目标转速(rad/s)
Sguan.transfer.Velocity.run.Fbk         (变量) 速度环反馈值，实际转速(rad/s)
Sguan.transfer.Velocity.run.Output      (变量) 速度环输出值，为目标Q轴电流(A)
............
Sguan.transfer.Position.run.Ref         (变量) 位置环期望值，目标位置(rad)
Sguan.transfer.Position.run.Fbk         (变量) 位置环反馈值，实际位置(rad)
Sguan.transfer.Position.run.Output      (变量) 位置环输出值，为目标转速(rad/s)
............
Sguan.motor.identify.pmsm.Status        (变量) 参数辨识状态机当前步骤，参考MOTOR_IDENTIFY_xxx宏定义
............
