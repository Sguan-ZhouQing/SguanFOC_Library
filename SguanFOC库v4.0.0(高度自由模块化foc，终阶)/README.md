总结REAME文档->

Sguan_Transfer.c/.h提供以下工程模块(用户可随便使用):

(以下的yes是仅对于float运算，对于IQmath后面再讨论)
Transfer1   典型一阶传递函数 yes
Transfer2   典型二阶传递函数 yes
Transfer3   典型三阶传递函数 yes
Transfer4   典型四阶传递函数 yes
Transfer5   典型五阶传递函数 yes
Integrator  积分器 yes
Derivative  微分器 yes
Curve       S型曲线加减速函数
Dft         快速傅里叶变换
Hall        霍尔编码器
Ladrc1      一阶线性自适应抗干扰控制
Ladrc2      二阶线性自适应抗干扰控制
Smc         传统指数型趋近率的滑模控制
Dpcc        增量式电流预测控制
Pir         比例积分谐振调节器
Pid         传统闭环控制器
Pll         开环锁相环
Lpf1        一阶低通滤波器 yes
Lpf2        二阶低通滤波器 yes
Hpf1        一阶高通滤波器 yes
Hpf2        二阶高通滤波器 yes
Bpf1        带通滤波器(一阶低通和高通串联) yes
Bpf2        带通滤波器(典型二阶系统改型) yes
Sogi        广义积分器(典型二阶系统改型) 
Nf          陷波滤波器(典型二阶系统改型) yes
Tpnf        陷波滤波器(三参数陷波滤波器) yes
Dob         超螺旋滑模扰动观测器
Rls         电机参数在线辨识观测器
Smo         (无感)滑模观测器
Nlfo        (无感)非线性磁链观测器
Vcfo        (无感)电压电流互补磁链观测器
Hfi         (无感)高频正弦波注入
Rolo        (无感)降阶龙伯格观测器
Mars        (无感)模型参考自适应观测器
Ekf         (无感)扩展卡尔曼滤波
Delay1      延时函数(延时一拍) yes
Delay2      延时函数(延时两拍) yes
Delay3      延时函数(延时三拍) yes

Sine        正弦发生器
Cosine      余弦发生器
SinCos      正余弦发生器
Tan         正切求解器
Atan        反正切求解器
Limit       限幅函数
Sign        符号函数
clarke      克拉克变换
park        帕克变换
ipark       帕克逆变换
Spwm0       零序注入的SPWM模块
Spwm        普通SPWM模块
Svpwm       七段式SVPWM模块
Swpwm       电调PWM无感方波
SingleRs    单电阻采样函数
