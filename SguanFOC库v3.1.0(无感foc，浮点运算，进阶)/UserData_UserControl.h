#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */

/**
 * @description: 1.用户的通信发送接口
 * @reminder: (此方函数->填写你需要修改的串口发送的数据)
 * @return {*}
 */
static inline void User_UserTX(void){
    // 传入串口printf要发送的数据，如txdata.fdata[0]，最多默认16个数值
    // 如需传入更多数值，请修改Sguan_printf中的参数
    // ========== 组1: 状态与核心指令 (0-3) ==========
    Sguan.txdata.fdata[0] = (float)Sguan.status;        // 状态机当前状态码
    Sguan.txdata.fdata[1] = Sguan.foc.Target_Speed;     // 目标机械转速 (rad/s)
    Sguan.txdata.fdata[2] = Sguan.foc.Target_Iq;        // 目标Q轴电流 (A)
    Sguan.txdata.fdata[3] = Sguan.foc.Target_Id;        // 目标D轴电流 (A)

    // ========== 组2: 实际反馈 (4-7) ==========
    Sguan.txdata.fdata[4] = Sguan.encoder.Real_Speed;   // 实际机械转速 (rad/s)
    Sguan.txdata.fdata[5] = Sguan.current.Real_Iq;      // 实际Q轴电流 (A)
    Sguan.txdata.fdata[6] = Sguan.current.Real_Id;      // 实际D轴电流 (A)
    Sguan.txdata.fdata[7] = Sguan.encoder.Real_Pos;     // 实际机械角度 (rad)

    // ========== 组3: 控制器输出与中间变量 (8-14) ==========
    Sguan.txdata.fdata[8]  = Sguan.foc.Uq_in;           // Q轴电压指令 (V)
    Sguan.txdata.fdata[9]  = Sguan.foc.Ud_in;           // D轴电压指令 (V)
    Sguan.txdata.fdata[10] = Sguan.foc.Real_VBUS;       // 母线电压实际值 (V)
    Sguan.txdata.fdata[11] = Sguan.foc.Du;              // U相占空比 (0~1)
    Sguan.txdata.fdata[12] = Sguan.foc.Dv;              // V相占空比 (0~1)
    Sguan.txdata.fdata[13] = Sguan.foc.Dw;              // W相占空比 (0~1)

    // Sguan.txdata.fdata[12] = Sguan.encoder.Sensorless_Speed;
    // Sguan.txdata.fdata[13] = Sguan.encoder.Sensorless_Pos;
    // Sguan.txdata.fdata[14] = Sguan.encoder.Sensorless_We;
    // Sguan.txdata.fdata[15] = Sguan.encoder.Sensorless_Re;

    Sguan.txdata.fdata[14] = Sguan.current.Real_Ia;     // A相瞬时电流值 (A)

    // ========== 组4: 系统配置与辅助信息 (15) ==========
    Sguan.txdata.fdata[15] = (float)CONFIG_MODE;        // 当前控制模式 (0-10)
}

/**
 * @description: 2.用户的通信接收接口
 * @reminder: (此方函数->填写浮点数赋值对象)
 * @reminder: (如果需要其他指令，可在Sguan_printf.c中添加)
 * @param {float} data
 * @return {*}
 */
static inline void Handle_User0_Adjust(float data){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是User0=xx?
    // 收到指令后，会把数据赋值到data
}

/**
 * @description: 3.用户的通信接收接口
 * @reminder: (此方函数->填写浮点数赋值对象)
 * @param {float} data
 * @return {*}
 */
static inline void Handle_User1_Adjust(float data){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是User1=xx?
    // 收到指令后，会把数据赋值到data
}

/**
 * @description: 4.用户的通信接收接口
 * @reminder: (此方函数->填写浮点数赋值对象)
 * @param {float} data
 * @return {*}
 */
static inline void Handle_User2_Adjust(float data){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是User2=xx?
    // 收到指令后，会把数据赋值到data
}


#endif // USERDATA_USERCONTROL_H
