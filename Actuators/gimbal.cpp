/**
 * @file           : gimbal.cpp
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-4-9 上午4:09
 * @lastEditor     :
 * @lastEditTime   : 25-4-9 上午4:09
 */

#include "BoardCom.h"
#include "DjiMotors.h"
#include "algorithm/pid.h"
#include "sheriff_os.h"
#include "stm32g4xx_hal_iwdg.h"

extern IWDG_HandleTypeDef hiwdg;
namespace iwdg {
void my_IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

void my_IWDG_Refresh(void) { HAL_IWDG_Refresh(&hiwdg); }
}  // namespace iwdg

namespace gimbal {
#define GIMBAL_CAN_ID 0x205
}  // namespace gimbal

namespace device::dji_motors {
Motor GimbalMotor(GIMBAL_CAN_ID, 1.0F, 65.4453125f, DJIMotorType::kGM6020v);
MotorManager GimbalGroup(GM6020TxID_1, GM6020RxID_1, &os::Can::Instance(FDCAN1), {GimbalMotor});

}  // namespace device::dji_motors

// 注册电机回调函数
InitLate(SetGimbalMotorCallback) {
    os::Can::Instance(FDCAN1).on_receive.connect([&](FDCAN_RxHeaderTypeDef const* rxHeader, uint8_t const* rxBuffer) {
        device::dji_motors::GimbalGroup.DecodeData(rxHeader->Identifier, rxBuffer);
    });
}

[[noreturn]] CREATE_THREAD_STATIC(gimbal_thread, 512, NULL, 5) {
    using namespace gimbal;
    // iwdg::my_IWDG_Init();  // 初始化独立看门狗
    // 初始化
    for (;;) {
        iwdg::my_IWDG_Refresh();  // 刷新独立看门狗
        if (not board_com::gimbal2chassis_pkg::Instance().isLost()) {
            device::dji_motors::GimbalMotor.SetOutput(
                board_com::gimbal2chassis_pkg::Instance().GetData().yaw_motor_output);
        } else {
            // 如果没有接收到数据，关闭电机
            device::dji_motors::GimbalMotor.SetOutput(0);
        }
        // 发送
        device::dji_motors::GimbalGroup.TransmitData();
        os::Sleep(1_ms);
    }
}
