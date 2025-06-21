/**
 * @file           : chassisLogic.cpp
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-4-7 下午10:14
 * @lastEditor     :
 * @lastEditTime   : 25-4-7 下午10:14
 */

/***
 *       ___          ___          ___          ___          ___                     ___
 *      /  /\        /__/\        /  /\        /  /\        /  /\       ___         /  /\
 *     /  /:/        \  \:\      /  /::\      /  /:/_      /  /:/_     /  /\       /  /:/_
 *    /  /:/          \__\:\    /  /:/\:\    /  /:/ /\    /  /:/ /\   /  /:/      /  /:/ /\
 *   /  /:/  ___  ___ /  /::\  /  /:/~/::\  /  /:/ /::\  /  /:/ /::\ /__/::\     /  /:/ /::\
 *  /__/:/  /  /\/__/\  /:/\:\/__/:/ /:/\:\/__/:/ /:/\:\/__/:/ /:/\:\\__\/\:\__ /__/:/ /:/\:\
 *  \  \:\ /  /:/\  \:\/:/__\/\  \:\/:/__\/\  \:\/:/~/:/\  \:\/:/~/:/   \  \:\/\\  \:\/:/~/:/
 *   \  \:\  /:/  \  \::/      \  \::/      \  \::/ /:/  \  \::/ /:/     \__\::/ \  \::/ /:/
 *    \  \:\/:/    \  \:\       \  \:\       \__\/ /:/    \__\/ /:/      /__/:/   \__\/ /:/
 *     \  \::/      \  \:\       \  \:\        /__/:/       /__/:/       \__\/      /__/:/
 *      \__\/        \__\/        \__\/        \__\/        \__\/                   \__\/
 */

#include "AppMessageQueues.h"
#include "algorithm/math.h"
#include "cap.h"
#include "chassis.h"
#include "units.h"
#include <etl/deque.h>
#include <FSM.h>
#include <referee.h>

dji::DjiReferee referee_data;

// -------- 底盘状态机--------
/**
 * 底盘状态机有如下状态
 * INIT，STOP，FOLLOW，GYRO，NORMAL
 *
 * 底盘功率状态机有如下状态
 * INIT，ECO，CUSTOMIZED，MAX_OUT
 *
 */

// ---------- 功率控制状态机---------
// 面向牢框架：
// POWER_LIMIT + CAP_NORMAL =  ECO
// POWER_LIMIT + CAP_SPEEDUP =  CUSTOMIZED
// POWER_UNLIMITED = MAX_OUTPUT

namespace chassis_power::event {
// --- 功率控制事件
os::EventType INIT_DONE;

}  // namespace chassis_power::event

namespace chassis::logic {
// ---- 底盘状态机
os::StateMachine Chassis_FSM;
}  // namespace chassis::logic

namespace chassis_power::logic {
// ---- 功率控制状态机
os::StateMachine PowerCtrl_FSM;
}  // namespace chassis_power::logic

namespace chassis_power {
namespace logic {
// ---- 功率控制状态机
// os::StateMachine PowerCtrl_FSM;

// ----状态机处理中变量
powerCtrl::power_mode_t power_mode;  // 功率模式
uint8_t _cap_status;                 // 电容状态
float referee_power_limit = 0.F;     // 裁判系统功率限制
float _power_limit = 0.F;            // 功率限制值

// ----状态机计算使用的常量
constexpr float _power_limit_max = 350.F;  // 功率限制最大值

// ---- 功率控制初始化
class : public os::StateBase {
    void onEnter() override {}
    void onExecute() override {
        // 初始化完成后默认进入ECO模式
        if (chassis::logic::Chassis_FSM.getCurrentState() != "init") {
            PowerCtrl_FSM.react(event::INIT_DONE);
        }
    }
    void onExit() override {}
    using StateBase::StateBase;
} INIT("init");

// ---- ECO模式 (对应限功率模式下的普通状态)
class : public os::StateBase {
    void onEnter() override {
        // 清空功率目标队列
        UNUSED() os::power_target_queue.Reset();
    }
    void onExecute() override {
        // 计算功率限制
        float temp_power_limit = 0;
        if (os::referee_power_limit_queue.ReceiveTo(&temp_power_limit, 0_ms)) {
            referee_power_limit = temp_power_limit;
        }
        // 否则保持原有值
        _power_limit = MIN(referee_power_limit, cap::Max_Power_Cal());
        // _power_limit = referee_power_limit;
        // 发送功率期望到功率期望队列
        os::power_target_queue.Send(&_power_limit, 0_ms);
    }
    void onExit() override {}
    using StateBase::StateBase;
} ECO("eco");

// ---- CUSTOMIZED模式 (对应限功率模式下的加速状态)
class : public os::StateBase {
    void onEnter() override {
        // 清空功率目标队列
        UNUSED() os::power_target_queue.Reset();
    }
    void onExecute() override {
        // 计算功率限制
        float temp_power_limit{};

        // if (os::customized_power_target_queue.ReceiveTo(&temp_power_limit, 0_ms)) {
        //     _power_limit = temp_power_limit;
        // }
        if (os::referee_power_limit_queue.ReceiveTo(&temp_power_limit, 0_ms)) {
            referee_power_limit = temp_power_limit;  // 赋值但是不调用
        }
        auto level = referee_data.robotPerf_.Level;
        // 根据机器人等级(1-10)线性映射功率(110-140W)
        temp_power_limit = 110.0F + (level - 1) * 3.0F;
        // 确保功率在范围内
        temp_power_limit = etl::clamp(temp_power_limit, 110.0F, 140.0F);
        _power_limit = MIN(temp_power_limit, cap::Max_Power_Cal());

        // 发送功率期望到功率期望队列
        os::power_target_queue.Send(&_power_limit, 0_ms);
    }
    void onExit() override {
        // 清空自定义功率目标队列
        UNUSED() os::customized_power_target_queue.Reset();
    }
    using StateBase::StateBase;
} CUSTOMIZED("customized");

// ---- MAX_OUT模式 (对应不限功率模式)
class : public os::StateBase {
    void onEnter() override {
        // 清空功率目标队列
        UNUSED() os::power_target_queue.Reset();
    }
    void onExecute() override {
        // 计算功率限制 (最大功率模式)
        _power_limit = MIN(_power_limit_max, cap::Max_Power_Cal());
        // 发送功率期望到功率期望队列
        os::power_target_queue.Send(&_power_limit, 0_ms);
    }
    void onExit() override {}
    using StateBase::StateBase;
} MAX_OUT("max_out");

class : public os::StateBase {
    void onEnter() override {
        // 清空功率目标队列
        UNUSED() os::power_target_queue.Reset();
    }
    void onExecute() override {
        // 裁判系统功率限制和电容允许最小功率取小值
        _power_limit = MIN(33, cap::Max_Power_Cal());
        // 发送功率期望到功率期望队列
        os::power_target_queue.Send(&_power_limit, 0_ms);
    }
    void onExit() override {}
    using StateBase::StateBase;
} EMERGENCY("emergency");

}  // namespace logic

namespace logic {
// 我的评价是直接checkout就行（
InitLate(add_power_ctrl_logic) {
    // 状态转换逻辑
    INIT + event::INIT_DONE = ECO;
    PowerCtrl_FSM.checkoutTo(INIT);
}

}  // namespace logic
}  // namespace chassis_power

namespace chassis::event {
// --- 板间通讯事件
os::EventType INIT_DONE;

}  // namespace chassis::event

namespace chassis::logic {

// ----- 状态机处理中变量
Chassis_Spd_t _ref;
ChassisMode _mode;
auto _stop_on_time = 0_ms;

// ---- 底盘初始化
class : public os::StateBase {
    void onEnter() override { UNUSED() os::chassis_ref_queue.Reset(); }
    void onExecute() override {
        // do nothing(笑了)
    }
    void onExit() override {}
    using StateBase::StateBase;
} INIT("init");

// ---- 随动状态
class : public os::StateBase {
    void onEnter() override {
        // 清空底盘期望值队列
        UNUSED() os::chassis_ref_queue.Reset();
        // 关力控
        auto control_mode = ControlMode::SPEED_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);  // 发送控制模式到队列
    }
    void onExecute() override {
        // 读取云台发送过来的期望值
        Chassis_Spd_t gimbal_ref{};
        os::gimbal2chassis_ref_queue.ReceiveTo(&gimbal_ref, 0_ms);
        _ref = Follow(gimbal_ref);                // 计算底盘期望值
        os::chassis_ref_queue.Send(&_ref, 0_ms);  // 发送底盘期望值到队列
    }
    void onExit() override {
        auto control_mode = ControlMode::FORCE_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);
    }
    using StateBase::StateBase;
} FOLLOW("follow");

// ---- 小陀螺状态
class : public os::StateBase {
    void onEnter() override {
        // 清空底盘期望值队列
        UNUSED() os::chassis_ref_queue.Reset();
        // 关力控
        auto control_mode = ControlMode::SPEED_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);  // 发送控制模式到队列
    }
    void onExecute() override {
        // 读取云台发送过来的期望值
        Chassis_Spd_t gimbal_ref{};
        os::gimbal2chassis_ref_queue.ReceiveTo(&gimbal_ref, 0_ms);  // 读取云台发送过来的期望值
        _ref = GYRO(gimbal_ref);                                    // 计算底盘期望值
        os::chassis_ref_queue.Send(&_ref, 0_ms);                    // 发送底盘期望值到队列
    }
    void onExit() override {
        auto control_mode = ControlMode::FORCE_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);
    }
    using StateBase::StateBase;
} GYRO("gyro");

// ---- “正常”模式
class : public os::StateBase {
    void onEnter() override {
        // 清空底盘期望值队列
        UNUSED() os::chassis_ref_queue.Reset();
        auto control_mode = ControlMode::FORCE_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);
    }
    void onExecute() override {
        // 读取云台发送过来的期望值
        Chassis_Spd_t gimbal_ref{};
        os::gimbal2chassis_ref_queue.ReceiveTo(&gimbal_ref, 0_ms);  // 读取云台发送过来的期望值
        _ref = Normal(gimbal_ref);                                  // 计算底盘期望值
        os::chassis_ref_queue.Send(&_ref, 0_ms);                    // 发送底盘期望值到队列
    }
    void onExit() override {}
    using StateBase::StateBase;
} NORMAL("normal");

// ---- 下台阶模式
constexpr auto _step_down_time_limit = 750_ms;     // 下台阶时间限制
time::microsecond_t _step_down_begin_time = 0_ms;  // 下台阶时间
class : public os::StateBase {
    void onEnter() override {
        // 清空底盘期望值队列
        UNUSED() os::chassis_ref_queue.Reset();
        UNUSED() os::control_mode_queue.Reset();
        _step_down_begin_time = os::GetTime();  // 记录下台阶时间
        // 关力控
        auto control_mode = ControlMode::SPEED_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);  // 发送控制模式到队列
    }
    void onExecute() override {
        if (os::GetTime() - _step_down_begin_time > _step_down_time_limit) {
            // 超过时间限制，退出下台阶模式
            Chassis_FSM.checkoutTo(FOLLOW);
            return;
        }
        // 读取云台发送过来的期望值
        Chassis_Spd_t gimbal_ref{};
        chassis_power::logic::PowerCtrl_FSM.checkoutTo(chassis_power::logic::MAX_OUT);
        // 给固定速度
        gimbal_ref.vx = max_chassis_spd;
        gimbal_ref.vy = 0_mps;
        // 计算底盘期望值
        _ref = Follow(gimbal_ref);
        os::chassis_ref_queue.Send(&_ref, 0_ms);  // 发送底盘期望值到队列
    }
    void onExit() override {
        chassis_power::logic::PowerCtrl_FSM.checkoutTo(chassis_power::logic::ECO);
        UNUSED() os::control_mode_queue.Reset();
        // 发送控制模式到队列
        auto control_mode = ControlMode::FORCE_CTRL;
        os::control_mode_queue.Send(&control_mode, 0_ms);
    }
    using StateBase::StateBase;
} STEP_DOWN("step_down");

// ---- 停止模式
class : public os::StateBase {
    void onEnter() override {
        // 清空底盘期望值队列
        UNUSED() os::chassis_ref_queue.Reset();
        _stop_on_time = os::GetTime();  // 记录停止时间
    }
    void onExecute() override {
        _ref = chassis::STOP();                   // 计算底盘期望值
        os::chassis_ref_queue.Send(&_ref, 0_ms);  // 发送底盘期望值到队列
    }
    void onExit() override {}
    using StateBase::StateBase;
} STOP("stop");

}  // namespace chassis::logic

namespace chassis::logic {
InitLate(add_chassis_ctrl_logic) {
    // 这是什么唐氏逻辑啊
    INIT + event::INIT_DONE = FOLLOW;

    Chassis_FSM.checkoutTo(INIT);
}
}  // namespace chassis::logic

CREATE_THREAD_STATIC(chassis_logic, 512, NULL, 5) {
    using namespace chassis_power::logic;
    using namespace chassis::logic;

    // ----- 初始化逻辑检查 -----
    // 不写在for(;;)中可以在每次进行状态切换时候少进行一次判断（你说是不是吧（皮））
    while (Chassis_FSM.getCurrentState() == "init") {
        Chassis_FSM.execute();
        PowerCtrl_FSM.execute();
        if (chassis::ifAllMotorEnable()) {
            Chassis_FSM.react(chassis::event::INIT_DONE);
            PowerCtrl_FSM.react(chassis_power::event::INIT_DONE);
        }
        os::Sleep(1_ms);
    }
    // ----- 进入主循环 -----
    for (;;) {
        Chassis_FSM.execute();
        PowerCtrl_FSM.execute();
        // 处理功率模式消息队列
        if (os::chassis_mode_queue.ReceiveTo(&_mode, 0_ms)) {
            switch (_mode) {
                case chassis::ChassisMode::STOP:
                    Chassis_FSM.checkoutTo(STOP);
                    break;
                case chassis::ChassisMode::FOLLOW:
                    Chassis_FSM.checkoutTo(FOLLOW);
                    break;
                case chassis::ChassisMode::GYRO:
                    Chassis_FSM.checkoutTo(GYRO);
                    break;
                case chassis::ChassisMode::NORMAL:
                    Chassis_FSM.checkoutTo(NORMAL);
                    break;
                default:
                    break;
            }
        }
        // 处理功率模式消息队列，添加超时处理
        if (os::power_mode_queue.ReceiveTo(&power_mode, 0_ms)) {
            switch (power_mode) {
                case powerCtrl::power_mode_t::ECO:
                    PowerCtrl_FSM.checkoutTo(ECO);
                    break;
                case powerCtrl::power_mode_t::CUSTOMIZED:
                    PowerCtrl_FSM.checkoutTo(CUSTOMIZED);
                    break;
                case powerCtrl::power_mode_t::MAX_OUT:
                    PowerCtrl_FSM.checkoutTo(MAX_OUT);
                    break;
                case powerCtrl::power_mode_t::EMERGENCY:
                    PowerCtrl_FSM.checkoutTo(EMERGENCY);
                default:
                    break;
            }
        }
        // os::Sleep(1_ms);
        osDelay(1);
    }
}
