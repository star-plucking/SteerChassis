/**
 * @file           : BoardCom.h
 * @author         : Foundary
 * @brief          : None
 * @date           : 2025/4/5 19:48
 * @lastEditor     : Foundary
 * @lastEditTime   : 2025/4/5 19:48
 */

#pragma once

#include "chassis.h"
#include "clock.h"
#include "connectivity/can.h"
#include "units.h"
#include <cstring>
#include <singleton.h>
#include <stm32g4xx_hal_fdcan.h>

namespace gimbal {
enum class YawMode : uint8_t {
    CONTROL = 0,
    ARMOR_AIM = 1,
    BIGBUFF_AIM,
    SMALLBUFF_AIM,
};
}  // namespace gimbal

namespace shooter {
enum class ShooterState : uint8_t {
    STOP = 0,      // 关摩擦轮
    SHOOTING = 1,  // 开摩擦轮
};

enum class AutoState : uint8_t {
    OFF_AUTO = 0,  // 无自动扳机
    IN_AUTO = 1,   // 有自动扳机
};

enum class FeederMode : uint8_t { STOP = 0, SINGLE = 1, CONTINUOUS, Init };
}  // namespace shooter

namespace board_com {

enum class GyroDirMode : uint8_t {
    NO_GYRO = 0,
    GYRO_CW = 1,   // 逆时针正转
    GYRO_CCW = 2,  // 顺时针反转
};

enum class FoldState : uint8_t {
    Fold = 0,
    Unfold = 1,
};

// ----- 接收数据包 ----- 底盘 ==> 云台 -----

enum boardComState {
    kConnected = 0,
    kLost,
};

constexpr uint32_t chassis2gimbal_id = 0x333;

struct __attribute__((packed)) chassis2gimbal_data {
    // byte 1-2
    uint8_t SOF1{0xa5}, SOF2{0x5a};
    // byte 3
    uint8_t robot_id;
    // byte 4
    uint8_t GameProcess;  // 比赛进程
    // byte 5-6
    uint16_t RemainSecond;  // 剩余时间
    // byte 7-10
    float shoot_spd_referee;
    // byte 11
    uint8_t init_finished;  // 拨盘初始化完成
    // byte 12
    uint8_t ShooterOutput;  // 电管armo口电压数据
    // byte 13
    uint8_t GimbalOutput;  // 电管gimbal口电压数据

    uint8_t _[3];
};  // 总共 16字节

class chassis2gimbal_pkg;
// 使用一个const指针用来在调试的时候访问单例
inline chassis2gimbal_pkg const* chassis2gimbal_pkg_debug_ptr = nullptr;

class __attribute__((packed)) chassis2gimbal_pkg : public Singleton<chassis2gimbal_pkg> {
    friend Singleton;

   private:
    FDCAN_TxHeaderTypeDef _txHeader{
        .Identifier = chassis2gimbal_id,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_16,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_FD_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0,
    };

    chassis2gimbal_data _data{};

    bool _data_updated = false;
    time::millisecond_t _last_update_time = 0_ms;
    boardComState state_ = kLost;

    // 私有构造函数
    chassis2gimbal_pkg() { chassis2gimbal_pkg_debug_ptr = this; }

   public:
    uint8_t const data_len = 16;  // 底盘发送数据16字节

    void Pack();

    // 获取数据
    [[nodiscard]] chassis2gimbal_data const& GetData() const { return _data; }

    // 新数据更新
    bool IsUpdated() {
        bool const temp = _data_updated;
        _data_updated = false;
        return temp;
    }

    void TestLost() {  // 传给connection guard做断联检测
        if (os::GetTime() - _last_update_time > 200_ms) {
            state_ = kLost;
        } else {
            state_ = kConnected;
        }
    }

    [[nodiscard]] bool isLost() const { return state_ == kLost; }

    // 接受并解析数据
    void Receive(uint8_t const* buffer, uint8_t length) {
        if (length != data_len) return;

        memcpy(&_data, buffer, sizeof(_data));
        _data_updated = true;
        _last_update_time = os::GetTime();
    }

    void FeederInitDone();  // 设置拨盘初始化完成
    void FeederInitReset();

    void SendData() { os::Can::Instance(FDCAN2).transmit(_txHeader, reinterpret_cast<uint8_t*>(&_data)); }
};

// ----- 发送数据包 ----- 底盘 <== 云台 -----
constexpr uint32_t gimbal2chassis_id = 0x430;
struct __attribute__((packed)) gimbal2chassis_data {
    // byte 1-2
    uint8_t SOF1, SOF2;
    // byte 3
    gimbal::YawMode yaw_mode : 2;           // 0为手调 1为装甲板自瞄 2为开大符 3为开小符
    chassis::ChassisMode chassis_mode : 2;  // 0为停止 1为不随动 2为小陀螺 3为随动
    powerCtrl::power_mode_t
        power_limit_mode : 2;  // 0为最大功率(无限制) 1为正常限制功控 2为自调功率上限(有功控) 3为低功率
    shooter::FeederMode feeder_mode : 2;
    // byte 4
    GyroDirMode gyro_dir : 2;  // 0为不陀螺 1为正 2为负
    uint8_t is_get_target : 1;
    uint8_t reserved2 : 2;
    uint8_t ui_cmd : 1;
    shooter::ShooterState shooter_state : 1;  // 摩擦轮状态
    shooter::AutoState auto_shoot_state : 1;  // 自动扳机状态

    // byte 5-6
    int16_t yaw_motor_output;
    // byte 7-10
    float yaw_pos_fdb;
    // byte 11-14
    float chassis_x_ref;
    // byte 15-18
    float chassis_y_ref;
    // byte 19-22
    float pitch_angle;

    // byte 23
    int8_t shooter_spd;  // 摩擦轮转速 * 10 (最大转速25m/s, *10 < 256)

    // byte 24
    FoldState fold_flag : 1;         // 是否折叠,0为折叠 1为展开
    uint8_t force_control : 1;       // 是否开启力控,0为关闭力控 1为开启力控
    uint8_t step_down : 1;           // 一键下台阶
    uint8_t fly_slope : 1;           // 一键飞坡
    uint8_t force_control_mode : 1;  // 力控模式,1为力控 0为速度控制
    uint8_t vtm_com_state : 1;       // 图传链路状态
    uint8_t reserved : 2;            // 预留

    // byte 25-32
    uint8_t ___[8];  ///> 预留
};  // 总共 32字节

enum class gimbal2chassisState { kLost = 0, kConnected = 1 };

class gimbal2chassis_pkg;
// 使用一个const指针用来在调试的时候访问单例
inline gimbal2chassis_pkg const* gimbal2chassis_pkg_debug_ptr = nullptr;

class __attribute__((packed)) gimbal2chassis_pkg : public Singleton<gimbal2chassis_pkg> {
    friend Singleton;

   private:
    FDCAN_TxHeaderTypeDef _txHeader{
        .Identifier = gimbal2chassis_id,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_32,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_FD_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0,
    };
    units::time::millisecond_t _last_update_time = 0_ms;
    boardComState state_ = kLost;

    gimbal2chassis_data _data{};

    // 私有构造函数
    gimbal2chassis_pkg() { gimbal2chassis_pkg_debug_ptr = this; }

   public:
    uint32_t const data_len = FDCAN_DLC_BYTES_32;  // 发送数据32字节

    // 获取发送头
    [[nodiscard]] FDCAN_TxHeaderTypeDef const& GetTxHeader() const { return _txHeader; }

    // 获取数据
    [[nodiscard]] gimbal2chassis_data const& GetData() const { return _data; }

    void SendData() const {
        uint8_t tx_data[32]{};
        memcpy(tx_data, &_data, sizeof(_data));
        os::Can::Instance(FDCAN2).transmit(_txHeader, tx_data);
    }

    void Pack();  // 打包数据，准备发送

    void ToggleUI_State() {  // 翻转UI状态，以做到刷新作用
        _data.ui_cmd = !_data.ui_cmd;
    }

    void Set_AutoShootState(shooter::AutoState _state) {  // 设置自动扳机状态
        _data.auto_shoot_state = _state;
    }

    void Set_FeederMode(shooter::FeederMode __mode) {  // 设置拨盘状态
        _data.feeder_mode = __mode;
    }

    void TestLost() {  // 传给connection guard做断联检测
        if (os::GetTime() - _last_update_time > 20_ms) {
            state_ = kLost;
        } else {
            state_ = kConnected;
        }
    }

    void Receive(uint8_t const* buffer, uint8_t const length) {
        if (length != data_len and buffer[0] != 0xA5) return;
        memcpy(&_data, buffer, sizeof(_data));
        _last_update_time = os::GetOsTime();
    }

    void Update_time() { _last_update_time = units::make_unit<units::time::second_t>(HAL_GetTick()); }  // 更新时间戳

    void SetState(boardComState const state) { state_ = state; }

    [[nodiscard]] bool isLost() const { return state_ == kLost; }
};

}  // namespace board_com
