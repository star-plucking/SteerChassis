t/**
 * @file           : feederLogic.cpp
 * @author         : Star_Plucking
 * @brief          : 拨盘电机逻辑
 * @date           : 25-4-12 下午10:59
 * @lastEditor     :
 * @lastEditTime   : 25-4-12 下午10:59
 */

/***
 *  ________  _______    _______    ________   _______    ________
 * |\  _____\|\  ___ \  |\  ___ \  |\   ___ \ |\  ___ \  |\   __  \
 * \ \  \__/ \ \   __/| \ \   __/| \ \  \_|\ \\ \   __/| \ \  \|\  \
 *  \ \   __\ \ \  \_|/__\ \  \_|/__\ \  \ \\ \\ \  \_|/__\ \   _  _\
 *   \ \  \_|  \ \  \_|\ \\ \  \_|\ \\ \  \_\\ \\ \  \_|\ \\ \  \\  \|
 *    \ \__\    \ \_______\\ \_______\\ \_______\\ \_______\\ \__\\ _\
 *     \|__|     \|_______| \|_______| \|_______| \|_______| \|__|\|__|
 */

#include "AppMessageQueues.h"
#include "BoardCom.h"
#include "DjiMotors.h"
#include "algorithm/math.h"
#include "clock.h"
#include "units.h"
#include <FSM.h>
#include <feeder.h>
#include <referee.h>

//---- 线程间的消息队列

// os::MsgQueue<float, 5> shooter_spd_queue;  // 摩擦轮速度队列
// os::MsgQueue<shooter::FeederMode, 5> shoot_mode_queue;

namespace device::dji_motors {
extern Motor feeder_motor;
}

// ---- 裁判系统数据
extern dji::DjiReferee referee_data;

namespace feeder {
// ---- 事件定义
namespace event {
os::EventType INIT_DONE;
os::EventType LOCKING;
os::EventType REVERSE_DONE;
os::EventType SINGLE_SHOOT;
os::EventType CONTINUOUS_SHOOT;
os::EventType STOP_SHOOT;
}  // namespace event

namespace logic {
// ---- 拨盘状态机
os::StateMachine Feeder_FSM;

FeederData feederData{};                           // 拨盘数据
float init_angle = 0.0F;                           // 初始化角度
bool shooter_done = false;                         // 摩擦轮状态
bool single_shooter_done = false;                  // 单发状态
time::second_t begin_time = 0_ms;                  // 倒转开始时间
#define FEEDER_ANGLE_STEP 45.F                     // 拨盘旋转步长
constexpr float max_frq = 20.0F;                   // 最大频率
constexpr float max_speed = max_frq * 60.F / 8.F;  // 最大速度（单位：rpm）  // 8.F为每一圈的弹丸数
constexpr float reverse_speed = -30.0F;            // 反转速度（单位：rpm）
constexpr time::second_t reverse_time = 100_ms;    // 反转时间（单位：ms）
constexpr float detected_shooter_speed = 9.1F;      // 检测正常摩擦轮速度（单位：rpm）9.4-9.8
constexpr float detected_shooter_launch = 9.F;    // 检测发弹丸时候摩擦轮的速度（单位：rpm）>=8.7
constexpr uint16_t max_remaining_heat = 25;        // 最大剩余热量（单位：交）(对于17mm其实就是两发弹丸)
constexpr time::second_t Interval_time = 30_ms;    // 单发的间隔时间

static time::second_t last_Single_time = 0_ms;  // 上一次单发时间
float shooter_spd = 0;

// ---- 拨盘初始化
class : public os::StateBase {
    void onEnter() override {
        // 清空发送队列
        UNUSED() os::feeder_ref_queue.Reset();
        feederData.angle = GetNowAngle();                  // 获取当前角度
        feederData.speed = 10.0F;                          // 设置当前速度为10.0F
        feederData.mode = FeederData::FeederMode::kSpeed;  // 设置当前模式为速度模式
    }

    void onExecute() override {
        feederData.speed = 10.0F;
        os::feeder_ref_queue.Send(&feederData, os::MsgQueue<FeederData, 5>::waitForever);  // 发送当前状态到消息队列
        // 初始化完成
        if (shooter_done and shooter_spd <= detected_shooter_launch) {
            init_angle = os::math::normalize_angle(GetNowAngle() + 15.F, 0.F, 45.F);  // 设置初始化角度为当前角度+25.0F
            feederData.angle = GetNowAngle() + 15.F;                                  // 设置当前角度为初始化角度
            feederData.mode = FeederData::FeederMode::kAngle;                         // 设置当前模式为角度模式
            feederData.speed = 0.0F;                                                  // 设置当前速度为0.0F
            os::feeder_ref_queue.Send(&feederData, os::MsgQueue<FeederData, 5>::waitForever);  // 发送当前状态到消息队列

            board_com::chassis2gimbal_pkg::Instance().FeederInitDone();  // 发送初始化完成事件

            Feeder_FSM.react(event::INIT_DONE);  // 发送初始化完成事件
        }
    }

    void onExit() override {
        // shooter_done = false;
    }

    using StateBase::StateBase;
} INIT("init");

// ----- 停止状态
class : public os::StateBase {
    void onEnter() override {
        // 清空发送队列
        UNUSED() os::feeder_ref_queue.Reset();
        feederData.angle = init_angle + std::floor((GetNowAngle() - init_angle + 40.F) / 45.F) * 45.F;  // 设置当前角度
        feederData.mode = FeederData::FeederMode::kAngle;
        feederData.speed = 0.0F;

        single_shooter_done = false;  // 设置单发状态为假（只允许停止状态清理单发状态位）
    }

    void onExecute() override {
        os::feeder_ref_queue.Send(&feederData, 0_ms);
         board_com::chassis2gimbal_pkg::Instance().FeederInitReset();
    }  // 保持当前角度并发送到消息队列

    void onExit() override {
        // shooter_done = false;
    }
    using StateBase::StateBase;
} STOP("stop");

// ----- 单发模式
class : public os::StateBase {
    void onEnter() override {
        // 清空发送队列
        UNUSED() os::feeder_ref_queue.Reset();

        // ----- 单发热控
        if (auto const current_heat = MAX(referee_data.powerAndHeat_.Shooter17mmHeat[0], feeder::GetOfflineHeat());
            static_cast<float>(referee_data.robotPerf_.HeatLimit) - current_heat <= max_remaining_heat) {
            Feeder_FSM.checkoutTo(STOP);  // 如果剩余热量小于等于最大剩余热量，则强制进入停止状态
        }
        bool const if_shoter_output = referee_data.robotPerf_.IsShooterOutput;

        // 设置拨盘旋转45度（一个弹丸的距离）
        if (shooter_done and if_shoter_output) {
            feederData.angle += FEEDER_ANGLE_STEP;
            feederData.mode = FeederData::FeederMode::kAngle;  // 设置当前模式为角度模式
        }
    }

    void onExecute() override {
        if (LockJudge()) {
            // 判断是否锁定
            Feeder_FSM.react(event::LOCKING);  // 发送锁定事件
        } else {
            if (shooter_done and referee_data.robotPerf_.IsShooterOutput == 1) {
                os::feeder_ref_queue.Send(&feederData, 0_ms);  // 发送当前状态到消息队列
            }
            // 单发控制逻辑
            if (fabs(device::dji_motors::feeder_motor.m_Encoder.GetShaftContinuousAngle() - feederData.angle) <= 0.1F) {
                // 判断是否完成单发
                Feeder_FSM.react(event::STOP_SHOOT);
            }
        }
    }  // 保持当前速度并发送到消息队列

    void onExit() override {
        last_Single_time = os::GetTime();
        // shooter_done = false;
    }
    using StateBase::StateBase;
} SINGLE("single");

// ---- 连续发射状态
class : public os::StateBase {
    void onEnter() override {
        // 清空发送队列
        UNUSED() os::feeder_ref_queue.Reset();

        feederData.mode = FeederData::FeederMode::kSpeed;
        feederData.speed = 0.0F;
    }

    void onExecute() override {
        if (LockJudge()) {
            // 判断是否锁定
            Feeder_FSM.react(event::LOCKING);  // 发送锁定事件
        } else {
            if (shooter_done and referee_data.robotPerf_.IsShooterOutput == 1) {
                // 在线离线综合热控系统

                if (auto const current_heat =
                        MAX(referee_data.powerAndHeat_.Shooter17mmHeat[0], feeder::GetOfflineHeat());
                    static_cast<float>(referee_data.robotPerf_.HeatLimit) - current_heat <= max_remaining_heat) {
                    feederData.speed = 0;
                } else {
                    feederData.speed = max_speed;
                }


                single_shooter_done = false;                       // 设置单发状态为假
                feederData.mode = FeederData::FeederMode::kSpeed;  // 设置当前模式为速度模式
                os::feeder_ref_queue.Send(&feederData, 0_ms);      // 发送当前状态到消息队列
            }
        }
    }

    void onExit() override {
        single_shooter_done = false;  // 设置单发状态为假
        feederData.speed = 0;
        // shooter_done = false;
    }

    using StateBase::StateBase;
} CONTINUOUS("continuous");

// -----  卡弹处理状态（防止炸膛）
class : public os::StateBase {
    void onEnter() override {
        // 清空发送队列
        UNUSED() os::feeder_ref_queue.Reset();

        begin_time = os::GetTime();                        // 获取当前时间
        feederData.speed = 0.0F;                           // 设置当前速度为0.0F
        feederData.mode = FeederData::FeederMode::kSpeed;  // 设置当前模式为速度模式
    }

    void onExecute() override {
        if (os::GetTime() - begin_time <= reverse_time) {
            feederData.speed = reverse_speed;  // 设置拨盘反转速度
            os::feeder_ref_queue.Send(&feederData, 0_ms);
        } else {
            Feeder_FSM.react(event::REVERSE_DONE);  // 发送反转完成事件
        }
    }

    void onExit() override {
        // 清理状态
        feederData.speed = 0;
        // shooter_done = false;
    }

    using StateBase::StateBase;
} LOCKING("locking");
}  // namespace logic

InitLate(add_feeder_logic) {
    using namespace feeder::logic;
    // 状态转换逻辑
    INIT + event::INIT_DONE = STOP;

    STOP + event::LOCKING = LOCKING;
    STOP + event::SINGLE_SHOOT = SINGLE;
    STOP + event::CONTINUOUS_SHOOT = CONTINUOUS;

    LOCKING + event::REVERSE_DONE = STOP;

    SINGLE + event::LOCKING = LOCKING;
    SINGLE + event::STOP_SHOOT = STOP;
    SINGLE + event::CONTINUOUS_SHOOT = CONTINUOUS;

    CONTINUOUS + event::LOCKING = LOCKING;
    CONTINUOUS + event::STOP_SHOOT = STOP;
    CONTINUOUS + event::SINGLE_SHOOT = SINGLE;
}
}  // namespace feeder

uint8_t fsy_test = 0;
uint8_t fsy_test2__ = 0;
[[noreturn]] CREATE_THREAD_STATIC(feeder_logic_thread, 1024, NULL, 5) {
    using namespace feeder::logic;
    for (;;) {
        shooter::FeederMode shoot_mode;
        os::shooter_spd_queue.ReceiveTo(&shooter_spd, 0_ms);

        if (shooter_spd > detected_shooter_launch)
            fsy_test2__ = 1;
        else fsy_test2__ = 0;

        if (shooter_spd < 8.F || referee_data.robotPerf_.IsShooterOutput == 0) {
            shooter_done = false;
            Feeder_FSM.checkoutTo(STOP);
        } else {
            if (shooter_spd >= detected_shooter_speed)
                shooter_done = true;

            if (os::shoot_mode_queue.ReceiveTo(&shoot_mode, 0_ms)) {
                switch (shoot_mode) {
                    case shooter::FeederMode::Init:
                        Feeder_FSM.checkoutTo(INIT);
                        fsy_test = 0;
                        break;
                    case shooter::FeederMode::CONTINUOUS:
                        Feeder_FSM.react(feeder::event::CONTINUOUS_SHOOT);
                        fsy_test = 1;
                        break;
                    case shooter::FeederMode::STOP:
                        Feeder_FSM.react(feeder::event::STOP_SHOOT);
                        fsy_test = 2;
                        break;
                    case shooter::FeederMode::SINGLE: {
                        if (os::GetTime() - last_Single_time < Interval_time) {
                            // 如果没有达到间隔时间，强制禁止下一次单发
                            break;
                        }
                        Feeder_FSM.react(feeder::event::SINGLE_SHOOT);
                        fsy_test = 3;
                        break;
                    }
                    default:
                        break;
                }
            }
        }
        Feeder_FSM.execute();
        os::Sleep(1_ms);  // 延时1ms
    }
}