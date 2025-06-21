/**
 * @file           : cap.h
 * @author         : Star_Plucking
 * @brief          : 电容协议文件之类的吧（大概）
 * @date           : 25-4-11 上午2:16
 * @lastEditor     :
 * @lastEditTime   : 25-4-11 上午2:16
 */
#pragma once

#include "sheriff_os.h"
#include "units.h"

namespace cap {
// 又是依托陈年老shit
uint8_t const level_max_power_table[10][2] = {  // 等级-功率对照表,用于校验裁判系统max power是否正确
    {60, 45}, {65, 50}, {70, 55}, {75, 60},  {80, 65},
    {85, 70}, {90, 75}, {95, 80}, {100, 90}, {100, 100}};  // 两行分别对应功率优先和血量优先

constexpr float Cap_Version_Table[4][3] = {
    {11.5f, 11.5f, 11.5f},  // buck_input_current_max
    {11.5f, 11.5f, 11.5f},  // buck_output_current_max
    {11.5f, 23.0f, 1.0f},   // boost_input_current_max
    {11.5f, 23.0f, 1.0f}    // boost_output_current_max
};
}  // namespace cap

namespace cap {

// ----- 电容 => 底盘数据包 -----
struct __attribute__((__packed__)) cap_data {
    float cap_power;            // 电容电量
    uint8_t cap_rest_energy;    // 电容剩余能量
    uint8_t buck_version : 4;   // buck版本(尚未知道有什么用)
    uint8_t boost_version : 4;  // boost版本(尚未知道有什么用)
};




enum cap_state {
    kConnected = 0,
    kLost = 1,
};
struct cap2chassis_community {
   private:
    cap_data _data{};
    cap_state _state = kLost;
    units::time::millisecond_t last_update_time = 0_ms;

    // 私有构造函数
    cap2chassis_community() = default;
    cap2chassis_community(cap2chassis_community const&) = default;

    float buck_input_current_max{};
    float buck_output_current_max{};
    float boost_input_current_max{};
    float boost_output_current_max{};

   public:
    uint8_t const data_len = 8;

    // 写成单例形式
    static cap2chassis_community& Instance() {
        static cap2chassis_community instance;
        return instance;
    }
    // 禁止赋值
    cap2chassis_community& operator=(cap2chassis_community const&) = delete;

    // 获取数据
    cap_data* GetData() { return &_data; }

    void TestLost() {
        if (os::GetTime() - last_update_time > 200_ms) {
            _state = kLost;
        } else {
            _state = kConnected;
        }
    }

    void Decode(uint8_t const* buffer, uint16_t const len) {
        if (len != data_len) return;
        memcpy(&_data, buffer, sizeof(_data));
        buck_input_current_max = Cap_Version_Table[0][_data.buck_version - 1];
        buck_output_current_max = Cap_Version_Table[1][_data.buck_version - 1];
        boost_input_current_max = Cap_Version_Table[2][_data.boost_version - 1];
        boost_output_current_max = Cap_Version_Table[3][_data.boost_version - 1];
    }

    [[nodiscard]] float GetBuckInputCurrentMax() const { return buck_input_current_max; }
    [[nodiscard]] float GetBuckOutputCurrentMax() const { return buck_output_current_max; }
    [[nodiscard]] float GetBoostInputCurrentMax() const { return boost_input_current_max; }
    [[nodiscard]] float GetBoostOutputCurrentMax() const { return boost_output_current_max; }

    [[nodiscard]] bool isLost() const { return _state == kLost; }
};

// ----- 底盘 => 电容数据包 -----

struct __attribute__((__packed__)) chassis2cap_data {
    uint8_t cap_mode_flag;      // 电容模式标志位
    uint8_t power_level_limit;  // 功率等级限制
    uint8_t power_limit;        // 功率限制
    uint8_t power_buffer;         // 功率缓冲
};

struct chassis2cap_community;
inline chassis2cap_community * chassis2cap_community_debug_ptr{};
struct chassis2cap_community {
   private:
    chassis2cap_data _data{};
    // 私有构造函数
    chassis2cap_community() = default;
    chassis2cap_community(chassis2cap_community const&) = default;

    FDCAN_TxHeaderTypeDef _txHeader{
        .Identifier = 0x98,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0,
    };

   public:
    uint8_t const data_len = 8;
    static chassis2cap_community& Instance() {
        static chassis2cap_community instance;
        return instance;
    }

    chassis2cap_community& operator=(chassis2cap_community const&) = delete;
    chassis2cap_data* GetData() { return &_data; }
    [[nodiscard]] FDCAN_TxHeaderTypeDef GetTxHeader() const { return _txHeader; }

    void Pack();
};

float Max_Power_Cal();

}  // namespace cap