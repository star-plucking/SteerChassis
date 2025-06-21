/**
 * @file           : cap.cpp
 * @author         : Star_Plucking
 * @brief          : 电容通信线程
 * @date           : 25-4-11 上午12:59
 * @lastEditor     :
 * @lastEditTime   : 25-4-11 上午12:59
 */

#include "cap.h"
#include <referee.h>
#include <sheriff_os.h>

cap::cap_data TestData{};
#define CAP_ID 0x299
void cap_rx_cb(FDCAN_RxHeaderTypeDef const* rxHeader, uint8_t const* rxBuffer) {
    if (rxHeader->Identifier == CAP_ID) {
        cap::cap2chassis_community::Instance().Decode(rxBuffer, rxHeader->DataLength);
        memcpy(&TestData, rxBuffer, 8);
    }
}

InitLate(init_supercap_Callback) { os::Can::Instance(FDCAN3).on_receive.connect(cap_rx_cb); }

// ---- 裁判系统数据
extern dji::DjiReferee referee_data;
float chassis_power;

[[noreturn]] CREATE_THREAD_STATIC(send_cap_, 512, NULL, 5) {
    for (;;) {
        using namespace cap;
        chassis2cap_community::Instance().Pack();
        os::Can::Instance(FDCAN3).transmit(chassis2cap_community::Instance().GetTxHeader(),
                                           reinterpret_cast<uint8_t*>(chassis2cap_community::Instance().GetData()));
        os::Sleep(10_ms);
        // osDelay(10);
    }
}

namespace cap {
void chassis2cap_community::Pack() {
    _data.cap_mode_flag = 0x77 << 1;  // 别问，问就是老代码就是这么写的
    _data.power_limit = referee_data.robotPerf_.PowerLimit;
    // 与血量功率对照表比对,确定功率上限
    for (int i = 0; i < 2; i++) {
        if (referee_data.robotPerf_.PowerLimit == level_max_power_table[referee_data.robotPerf_.Level - 1][i]) {
            _data.power_level_limit = referee_data.robotPerf_.PowerLimit;
        }

        _data.power_buffer = referee_data.powerAndHeat_.ChassisEnergyBuffer;
    }
}

float Max_Power_Cal() {
    static bool power_limited = false;
    uint16_t const remain_energy = cap2chassis_community::Instance().GetData()->cap_rest_energy;

    // 滞环控制：当能量低于18限制功率，只有当能量恢复到25以上时才解除限制
    if (remain_energy < 15.0F) {
        power_limited = true;
        return 40.0F;
    } else if (power_limited && remain_energy < 25.0F) {
        // 仍处于限制状态，等待能量恢复到阈值以上
        return 40.0F;
    } else {
        // 電力充足，解除限制
        power_limited = false;
        return 400.0F;
    }
}

// 新超电板掉压保护
// float Max_Power_Cal() {
//     uint16_t const remain_energy = cap2chassis_community::Instance().GetData()->cap_rest_energy;
//     float VCC_Cap = os::math::sqrt((double)((remain_energy / 100.0f) * 27.0f * 27.0f));
//     float Power_Input_Max = cap2chassis_community::Instance().GetBoostInputCurrentMax() * VCC_Cap;
//     float VCC_Motor = MIN(24.0f, (Power_Input_Max / cap2chassis_community::Instance().GetBoostOutputCurrentMax()));
//     float Power_Output_Max = cap2chassis_community::Instance().GetBoostOutputCurrentMax() * VCC_Motor;
//     float Cap_Charging_Power_Max =
//         MIN(VCC_Cap * cap2chassis_community::Instance().GetBuckOutputCurrentMax(), referee_data.robotPerf_.PowerLimit ) *
//         0.75f;
//     if (remain_energy >= 15) {
//         return Power_Output_Max;
//     } else {
//         float Low_Power_Output_Max = Power_Output_Max * (remain_energy * remain_energy) / 255.0f;
//         return MAX(Low_Power_Output_Max, Cap_Charging_Power_Max);
//     }
// }

}  // namespace cap