/**
 * @file           : feeder.h
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-4-12 下午11:01
 * @lastEditor     :
 * @lastEditTime   : 25-4-12 下午11:01
 */
#pragma once
#include <cstdint>

namespace feeder {
struct FeederData {
    enum class FeederMode {
        kAngle = 0,
        kSpeed = 1,
    } mode;
    float speed;
    float angle;
};

// ------- 卡拨盘参数
constexpr float locked_current = 9600.0F * 10.0F / 10000.F;  // 10A
constexpr float locked_speed = 10.0F;
constexpr uint8_t locked_time = 50;

// ------- 热量控制相关参数
constexpr float FEEDER_ANGLE_STEP = 45.0f;     // 一个弹丸的角度步进
constexpr float DEFAULT_COOLING_RATE = 20.0f;  // 默认冷却速率，每秒冷却值
constexpr float HEAT_PER_BULLET = 10.0f;       // 每发弹丸热量增加值

float GetNowAngle();
bool LockJudge();
bool ifFinishSingleShoot();
void OfflineUpdateHeat();
float GetOfflineHeat();
}  // namespace feeder