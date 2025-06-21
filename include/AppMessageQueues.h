//=======================================================================================
// @file AppMessageQueues.h
// @brief 此文件用于统一管理APP层的消息队列。
// @author Trae自动生成
// @date 2025/5/3 9:00
//=======================================================================================

#ifndef APP_MESSAGE_QUEUES_H
#define APP_MESSAGE_QUEUES_H

#include "BoardCom.h"
#include "chassis.h"
#include "feeder.h"
#include "msgQueue.h"

namespace os {
// 拨盘状态速度队列
inline MsgQueue<feeder::FeederData, 5> feeder_ref_queue{};
// 底盘模式队列
inline MsgQueue<chassis::ChassisMode, 5> chassis_mode_queue{};
// 底盘速度目标队列
inline MsgQueue<chassis::Chassis_Spd_t, 5> gimbal2chassis_ref_queue{};
// 小陀螺方向队列
inline MsgQueue<int8_t, 5> gyro_direction_queue{};
// 摩擦轮速度队列
inline MsgQueue<float, 5> shooter_spd_queue{};
// 摩擦轮模式队列
inline MsgQueue<shooter::FeederMode, 5> shoot_mode_queue{};
// 自定义功率目标队列
inline MsgQueue<float, 5> customized_power_target_queue{};
// 功率模式队列
inline MsgQueue<powerCtrl::power_mode_t, 5> power_mode_queue{};
// 底盘速度目标队列
inline MsgQueue<chassis::Chassis_Spd_t, 5> chassis_ref_queue{};
// 功率目标队列
inline MsgQueue<float, 5> power_target_queue{};
// 裁判系统功率限制队列
inline MsgQueue<float, 5> referee_power_limit_queue{};
// 底盘当前功率队列
inline MsgQueue<chassis::ControlMode, 5> control_mode_queue{};  // 傻逼
// 一键下坡队列
inline MsgQueue<bool, 5> step_down_queue{}; // 傻逼说的是我
}  // namespace os

#endif  // APP_MESSAGE_QUEUES_H