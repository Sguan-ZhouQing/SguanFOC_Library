#include "Sguan_MotorStatus.h"











// // static void (*const status_handlers[])(void) = {
// //     0
// // }

// // void MotorStatus_Loop(uint8_t *status){
// //     // 运行状态机任务指示函数
// //     // 带“输入参数”数值限定
// //     status_handlers[Value_set(*status,
// //         MOTOR_STATUS_DISABLED,0)]();
// // }

// #include <stdio.h>
// #include <stdint.h>
// #include <stdbool.h>

// // ============================================
// // 1. 定义状态和事件（枚举）
// // ============================================

// // 电机状态
// typedef enum {
//     STATE_IDLE = 0,           // 空闲
//     STATE_STARTING,           // 启动中
//     STATE_RUNNING,            // 运行中
//     STATE_STOPPING,           // 停止中
//     STATE_FAULT,              // 故障
//     STATE_EMERGENCY_STOP,     // 急停
//     STATE_CALIBRATING         // 校准中
// } MotorState;

// // 触发事件
// typedef enum {
//     EVENT_NONE = 0,
//     EVENT_START,              // 启动命令
//     EVENT_STOP,               // 停止命令
//     EVENT_EMERGENCY,          // 急停
//     EVENT_FAULT_OCCURRED,     // 发生故障
//     EVENT_FAULT_CLEARED,      // 故障清除
//     EVENT_CALIBRATE,          // 校准命令
//     EVENT_CALIB_DONE,         // 校准完成
//     EVENT_SPEED_REACHED,      // 达到目标速度
//     EVENT_SPEED_ZERO          // 速度归零
// } MotorEvent;

// // ============================================
// // 2. 定义动作函数类型（函数指针）
// // ============================================

// // 动作函数：接收上下文，返回 bool 表示动作是否成功
// typedef bool (*ActionFunc)(void *context);

// // ============================================
// // 3. 定义状态表条目
// // ============================================

// typedef struct {
//     MotorState current_state;    // 当前状态
//     // 持续运行的状态
//     MotorEvent event;            // 触发事件
//     ActionFunc action;           // 要执行的动作（可为 NULL）
//     MotorState next_state;       // 下一个状态
//     const char *description;     // 描述信息（调试用）
// } StateTransition;

// // ============================================
// // 4. 电机上下文结构体（存储运行时数据）
// // ============================================

// typedef struct {
//     MotorState state;            // 当前状态
//     uint32_t speed;              // 当前速度
//     uint32_t target_speed;       // 目标速度
//     uint32_t fault_code;         // 故障码
//     uint32_t run_time;           // 运行时间
//     // ... 其他参数
// } MotorContext;

// // ============================================
// // 5. 实现动作函数
// // ============================================

// // 动作：启动电机
// bool action_start_motor(void *ctx) {
//     MotorContext *motor = (MotorContext *)ctx;
//     printf("  🔧 执行动作: 启动电机 (目标速度: %d RPM)\n", motor->target_speed);
//     motor->speed = 100;  // 模拟启动
//     return true;
// }

// // 动作：停止电机
// bool action_stop_motor(void *ctx) {
//     MotorContext *motor = (MotorContext *)ctx;
//     printf("  🔧 执行动作: 停止电机\n");
//     motor->speed = 0;
//     return true;
// }

// // 动作：紧急停止
// bool action_emergency_stop(void *ctx) {
//     MotorContext *motor = (MotorContext *)ctx;
//     printf("  🚨 执行动作: 紧急停止！\n");
//     motor->speed = 0;
//     motor->state = STATE_EMERGENCY_STOP;  // 立即切换
//     return true;
// }

// // 动作：清除故障
// bool action_clear_fault(void *ctx) {
//     MotorContext *motor = (MotorContext *)ctx;
//     printf("  🔧 执行动作: 清除故障 (故障码: 0x%X)\n", motor->fault_code);
//     motor->fault_code = 0;
//     return true;
// }

// // 动作：开始校准
// bool action_start_calibrate(void *ctx) {
//     MotorContext *motor = (MotorContext *)ctx;
//     printf("  🔧 执行动作: 开始校准...\n");
//     motor->speed = 0;
//     return true;
// }

// // 动作：完成校准
// bool action_calib_done(void *ctx) {
//     printf("  🔧 执行动作: 校准完成 ✅\n");
//     return true;
// }

// // 动作：空动作（什么都不做）
// bool action_none(void *ctx) {
//     return true;
// }

// // ============================================
// // 6. 定义状态表（核心！）
// // ============================================

// // 1.立即类（事件触发，事件立即发送并切换）
// // 启动初始化，清除故障........手动切事件触发
// // 故障类，电机加减速状态等........自动计算事件触发
// // 2.延时类（计时，如果未达预期值；或者达到预期值，则触发事件）
// // 故障类..........自动切事件触发
// // (每个状态机都有持续的任务)

// static const StateTransition motor_state_table[] = {
//     // { 当前状态,     触发事件,            动作,              下一个状态,       描述 }
    
//     // ---- 空闲状态 ----
//     { STATE_IDLE,       EVENT_START,         action_start_motor,   STATE_STARTING,   "空闲 → 启动" },
//     { STATE_IDLE,       EVENT_CALIBRATE,     action_start_calibrate, STATE_CALIBRATING, "空闲 → 校准" },
//     { STATE_IDLE,       EVENT_FAULT_OCCURRED, action_none,         STATE_FAULT,      "空闲 → 故障" },
    
//     // ---- 启动状态 ----
//     { STATE_STARTING,   EVENT_SPEED_REACHED, action_none,         STATE_RUNNING,    "启动 → 运行" },
//     { STATE_STARTING,   EVENT_FAULT_OCCURRED, action_none,         STATE_FAULT,      "启动 → 故障" },
//     { STATE_STARTING,   EVENT_EMERGENCY,     action_emergency_stop, STATE_EMERGENCY_STOP, "启动 → 急停" },
    
//     // ---- 运行状态 ----
//     { STATE_RUNNING,    EVENT_STOP,          action_stop_motor,    STATE_STOPPING,   "运行 → 停止" },
//     { STATE_RUNNING,    EVENT_FAULT_OCCURRED, action_none,         STATE_FAULT,      "运行 → 故障" },
//     { STATE_RUNNING,    EVENT_EMERGENCY,     action_emergency_stop, STATE_EMERGENCY_STOP, "运行 → 急停" },
    
//     // ---- 停止状态 ----
//     { STATE_STOPPING,   EVENT_SPEED_ZERO,    action_none,         STATE_IDLE,       "停止 → 空闲" },
//     { STATE_STOPPING,   EVENT_FAULT_OCCURRED, action_none,         STATE_FAULT,      "停止 → 故障" },
    
//     // ---- 故障状态 ----
//     { STATE_FAULT,      EVENT_FAULT_CLEARED, action_clear_fault,  STATE_IDLE,       "故障 → 空闲" },
    
//     // ---- 急停状态 ----
//     { STATE_EMERGENCY_STOP, EVENT_FAULT_CLEARED, action_clear_fault, STATE_IDLE,   "急停 → 空闲" },
    
//     // ---- 校准状态 ----
//     { STATE_CALIBRATING, EVENT_CALIB_DONE,   action_calib_done,   STATE_IDLE,       "校准 → 空闲" },
//     { STATE_CALIBRATING, EVENT_FAULT_OCCURRED, action_none,         STATE_FAULT,      "校准 → 故障" },
    
//     // 结束标记（用于遍历）
//     { STATE_IDLE,       EVENT_NONE,          NULL,                STATE_IDLE,       "END" }
// };

// // 获取状态表大小
// #define STATE_TABLE_SIZE (sizeof(motor_state_table) / sizeof(motor_state_table[0]))

// // ============================================
// // 7. 状态机核心函数
// // ============================================

// // 处理事件
// bool motor_handle_event(MotorContext *motor, MotorEvent event) {
//     printf("\n📌 当前状态: %d, 事件: %d\n", motor->state, event);
    
//     // 遍历状态表，查找匹配的条目
//     for (int i = 0; i < STATE_TABLE_SIZE; i++) {
//         const StateTransition *trans = &motor_state_table[i];
        
//         // 检查是否匹配
//         if (trans->current_state == motor->state && trans->event == event) {
//             // 执行动作
//             if (trans->action != NULL) {
//                 if (!trans->action(motor)) {
//                     printf("  ❌ 动作执行失败！\n");
//                     return false;
//                 }
//             }
            
//             // 状态转移
//             printf("  📍 状态转移: %s\n", trans->description);
//             motor->state = trans->next_state;
//             return true;
//         }
//     }
    
//     // 没有找到匹配的转移
//     printf("  ⚠️ 未找到匹配的转移: 状态=%d, 事件=%d\n", motor->state, event);
//     return false;
// }

// // ============================================
// // 8. 演示主程序
// // ============================================

// int main() {
//     // 初始化电机上下文
//     MotorContext motor = {
//         .state = STATE_IDLE,
//         .speed = 0,
//         .target_speed = 3000,
//         .fault_code = 0,
//         .run_time = 0
//     };
    
//     printf("========== 电机状态机演示 ==========\n");
//     printf("初始状态: %d (空闲)\n\n", motor.state);
    
//     // 模拟一系列事件
//     motor_handle_event(&motor, EVENT_START);          // 启动
//     motor_handle_event(&motor, EVENT_SPEED_REACHED);  // 达到速度
//     motor_handle_event(&motor, EVENT_FAULT_OCCURRED); // 发生故障
//     motor_handle_event(&motor, EVENT_FAULT_CLEARED);  // 清除故障
//     motor_handle_event(&motor, EVENT_START);          // 再次启动
//     motor_handle_event(&motor, EVENT_EMERGENCY);      // 急停
//     motor_handle_event(&motor, EVENT_FAULT_CLEARED);  // 清除故障
    
//     printf("\n✅ 最终状态: %d\n", motor.state);
    
//     return 0;
// }
