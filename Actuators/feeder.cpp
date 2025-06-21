/**
 * @file           : feeder.cpp
 * @author         : Star_Plucking
 * @brief          : 拨盘电机控制
 * @date           : 25-4-11 上午1:40
 * @lastEditor     :
 * @lastEditTime   : 25-4-11 上午1:40
 */

#include "feeder.h"
#include "AppMessageQueues.h"
#include "DjiMotors.h"
#include "referee.h"
#include <algorithm/filter.h>
#include <algorithm/pid.h>
#include <mutex>
#include <FSM.h>

extern dji::DjiReferee referee_data;       // 裁判系统数据

#define FEEDER_CAN_ID 0x201
namespace device::dji_motors {
Motor feeder_motor(FEEDER_CAN_ID, 36.0F, 0.0F, DJIMotorType::kM2006);
MotorManager feeder_group(M2006TxID_1, M2006RxID_1, &os::Can::Instance(FDCAN3), {feeder_motor});
}  // namespace device::dji_motors

namespace feeder {
PIDController feeder_speed_pid({200.0f, 100.0f, 0.0f, 5000.0F, 20000.0f, 20000.0f});
PIDController feeder_angle_pid({4.f, 50.f, 0.0f, 2.5F, 250.0f, 20000.0f});

os::filter::LowPassFilter feeder_spd_filter(0.0f);

}  // namespace feeder

// 注册电机回调函数
InitLate(SetFeederMotorCallback) {
    os::Can::Instance(FDCAN3).on_receive.connect([&](FDCAN_RxHeaderTypeDef const* rxHeader, uint8_t const* rxBuffer) {
        device::dji_motors::feeder_group.DecodeData(rxHeader->Identifier, rxBuffer);
    });
}

namespace feeder {
float feeder_angle_ref = 0.0F;  // 拨盘角度期望
float feeder_speed_ref = 0.0F;  // 拨盘速度期望
float feeder_angle_fdb = 0.0F;  // 拨盘角度反馈
float feeder_speed_fdb = 0.0F;  // 拨盘速度反馈
FeederData feeder_data;

bool GetRef() { return os::feeder_ref_queue.ReceiveTo(&feeder_data, 0_ms); }

void CalClosePID() {
    float output{};
    if (feeder_data.mode == FeederData::FeederMode::kAngle) {
        // 角度速度串级PID
        feeder_angle_pid.setReference(feeder_angle_ref);
        feeder_angle_pid.setFeedback(device::dji_motors::feeder_motor.m_Encoder.GetShaftContinuousAngle());
        feeder_speed_pid.setReference(feeder_angle_pid.calculate());
        feeder_speed_pid.setFeedback(
            feeder_spd_filter.calculate(device::dji_motors::feeder_motor.m_Encoder.GetShaftSpeed()));
        output = feeder_speed_pid.calculate();
    } else if (feeder_data.mode == FeederData::FeederMode::kSpeed) {
        // 速度PID
        feeder_speed_pid.setReference(feeder_speed_ref);
        feeder_speed_pid.setFeedback(
            feeder_spd_filter.calculate(device::dji_motors::feeder_motor.m_Encoder.GetShaftSpeed()));
        output = feeder_speed_pid.calculate();
    }
    device::dji_motors::feeder_motor.SetOutput(static_cast<int16_t>(output));
}

void Send_output() { device::dji_motors::feeder_group.TransmitData(); }

float GetNowAngle() {
    return device::dji_motors::feeder_motor.m_Encoder.GetShaftContinuousAngle();  // 获取当前角度
}
}  // namespace feeder

float ang_ref{};
float spd_ref{};
[[noreturn]] CREATE_THREAD_STATIC(feeder_thread, 512, NULL, 5) {
    using namespace feeder;
    feeder_data.mode = FeederData::FeederMode::kSpeed;  // 设置当前模式为速度模式
    for (;;) {
        if (GetRef()) {
            feeder_angle_ref = feeder_data.angle;
            feeder_speed_ref = feeder_data.speed;
        }

        CalClosePID();
        OfflineUpdateHeat();
        Send_output();
        osDelay(1);
    }
}

bool feeder::LockJudge() {
    static uint8_t lock_count = 0;
    if ((std::abs(device::dji_motors::feeder_motor.m_Encoder.GetCurrent()) >= locked_current) and
        (std::abs(device::dji_motors::feeder_motor.m_Encoder.GetShaftSpeed()) <= locked_speed)) {
        lock_count++;
        if (lock_count >= locked_time) return true;
    }
    lock_count = 0;
    return false;
}

bool feeder::ifFinishSingleShoot() {
    if (std::abs(feeder_angle_pid.getFeedback() - feeder_angle_pid.getRef()) <= 5.0f) {
        return true;
    }
    return false;
}

namespace feeder::logic {
extern os::StateMachine Feeder_FSM;
}

namespace feeder {
etl::atomic<float> sum_heat = 0.0F;  // 当前热量
float last_angle{};                  // 上次角度
time::second_t last_update_time{};   // 上次更新时间
bool is_shooter_locked = false;      // 是否锁定
}  // namespace feeder

void feeder::OfflineUpdateHeat() {

    // 获取裁判系统数据中的热量上限和冷却速率
    // float const heat_limit = referee_data.robotPerf_.HeatLimit; // 暂时弃用
    float cooling_rate{};

    cooling_rate = referee_data.robotPerf_.CoolingRate;

    if (cooling_rate <= 0.0f) {
        cooling_rate = DEFAULT_COOLING_RATE;  // 使用默认冷却速率
    }

    // 计算当前角度和上次角度的差值，判断是否发射了弹丸
    float const current_angle = GetNowAngle();

    // 如果角度差大于一定值，则认为转过了一发弹丸的距离
    if (float const angle_diff = current_angle - last_angle; fabs(angle_diff) >= FEEDER_ANGLE_STEP) {
        // 计算发射了多少发弹丸（可能一次更新转过多个弹丸角度）
        int const bullet_count = static_cast<int>(fabs(angle_diff) / FEEDER_ANGLE_STEP);
        // 增加热量
        sum_heat = sum_heat + static_cast<float>(bullet_count) * HEAT_PER_BULLET;
        // 更新上次角度为当前角度减去多余的角度（不足一个弹丸角度的部分）
        last_angle = current_angle - angle_diff + static_cast<float>(bullet_count) * FEEDER_ANGLE_STEP;
    }

    // 按照10Hz的频率结算冷却
    auto const current_time = os::GetTime();

    if (auto const time_diff = current_time - last_update_time; time_diff >= 100_ms) {  // 每100ms结算一次，即10Hz
        // 计算冷却值 = 每秒冷却值 / 10
        float const cooling_value = cooling_rate / 10.0f;
        // 减少热量
        sum_heat = math::max(0.0f, sum_heat - cooling_value);
        last_update_time = current_time;
    }

    // 三个热量常数计算（UNUSED）
    // Q0 = 热量上限
    // Q1 = 当前热量(sum_heat)
    // Q2 = Q0 + 100 (17mm发射机构)
    // float const Q0 = heat_limit;
    // float const Q1 = sum_heat;
    // float const Q2 = Q0 + 100.0f;
}

/**
 * @brief 获取离线热控当前热量
 * @return 离线热控当前热量
 */
float feeder::GetOfflineHeat() { return sum_heat; }