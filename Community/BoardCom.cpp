/**
 * @file           : BoardCom.cpp
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-4-4 下午9:08
 * @lastEditor     :
 * @lastEditTime   : 25-4-4 下午9:08
 */

#include "BoardCom.h"
#include "AppMessageQueues.h"
#include "chassis.h"
#include <ConnectionGuard.h>
#include <referee.h>
#include <sheriff_os.h>
#include <units.h>

extern dji::DjiReferee referee_data;
// ---- 板通接收互斥锁
os::Semaphore board_com_binary(os::Semaphore::TYPE::BINARY);
// ----- 板通接收数据包回调初始化
InitLate(connect_boardcom_callback) {
    os::Can::Instance(FDCAN2).on_receive.connect([&](FDCAN_RxHeaderTypeDef const* rxHeader, uint8_t const* rxBuffer) {
        if (rxHeader->Identifier == board_com::gimbal2chassis_id) {
            if (rxBuffer[0] != 0xa5 || rxBuffer[1] != 0x5a) return;
            board_com::gimbal2chassis_pkg::Instance().Receive(rxBuffer, rxHeader->DataLength);
            UNUSED() board_com_binary.GiveFromISR();
        }
    });
    // 连接丢失回调
    os::ConnectionGuard::guard_().connect([&] { board_com::gimbal2chassis_pkg::Instance().TestLost(); });
}

// ----- 板通发送数据包线程
[[noreturn]] CREATE_THREAD_STATIC(board_com_send_thread, 1024, NULL, 5) {
    using namespace board_com;
    for (;;) {
        chassis2gimbal_pkg::Instance().Pack();
        chassis2gimbal_pkg::Instance().SendData();
         os::Sleep(1_ms);
        // osDelay(1);
    }
}

// ----- 板通接收数据包线程

[[noreturn]] CREATE_THREAD_STATIC(board_com_receive_thread, 1024, NULL, 5) {
    using namespace board_com;
    for (;;) {
        // 使用互斥锁保护数据接收
        if (board_com_binary.Take(200_ms)) {
            // 发送底盘模式到消息队列
            auto chassis_mode = gimbal2chassis_pkg::Instance().GetData().chassis_mode;
            os::chassis_mode_queue.Send(&chassis_mode, 0_ms);
            // 发送底盘期望速度到消息队列
            chassis::Chassis_Spd_t chassis_spd{};
            chassis_spd.vx =
                units::make_unit<velocity::meters_per_second_t>(gimbal2chassis_pkg::Instance().GetData().chassis_x_ref);
            chassis_spd.vy =
                units::make_unit<velocity::meters_per_second_t>(gimbal2chassis_pkg::Instance().GetData().chassis_y_ref);
            chassis_spd.w = 0_rad_per_s;
            os::gimbal2chassis_ref_queue.Send(&chassis_spd, 0_ms);

            // 发送骚陀螺方向到消息队列
            if (gimbal2chassis_pkg::Instance().GetData().gyro_dir == GyroDirMode::GYRO_CCW) {
                int8_t direction = 1;
                os::gyro_direction_queue.Send(&direction, 0_ms);
            } else if (gimbal2chassis_pkg::Instance().GetData().gyro_dir == GyroDirMode::GYRO_CW) {
                int8_t direction = -1;
                os::gyro_direction_queue.Send(&direction, 0_ms);
            }

            // 发送shooter速度到消息队列
            auto shooter_true_spd = gimbal2chassis_pkg::Instance().GetData().shooter_spd / 10.0F;
            os::shooter_spd_queue.Send(&shooter_true_spd, 0_ms);

            // 发送feeder模式到消息队列
            auto shoot_mode = gimbal2chassis_pkg::Instance().GetData().feeder_mode;
            os::shoot_mode_queue.Send(&shoot_mode, 0_ms);

            // 发送功率模式到消息队列
            auto power_mode = gimbal2chassis_pkg::Instance().GetData().power_limit_mode;
            os::power_mode_queue.Send(&power_mode, 0_ms);

            // 发送底盘控制模式到队列（temp unused）
            // static auto last_ctrl_mode = chassis::ControlMode::FORCE_CTRL;
            // auto chassis_ctrl_mode =
            //     static_cast<chassis::ControlMode>(gimbal2chassis_pkg::Instance().GetData().force_control);
            // if (chassis_ctrl_mode != last_ctrl_mode) {
            //     os::control_mode_queue.Send(&chassis_ctrl_mode, 0_ms);
            // }
            // last_ctrl_mode = chassis_ctrl_mode;


            // TODO : 其他期望值
        }
        if (gimbal2chassis_pkg::Instance().isLost()) {
            // 转到STOP模式底盘
            // 发送底盘模式到消息队列
            chassis::ChassisMode mode = chassis::ChassisMode::STOP;
            os::chassis_mode_queue.Send(&mode, 0_ms);

            // 发送feeder模式到消息队列
            shooter::FeederMode shoot_mode = shooter::FeederMode::STOP;
            os::shoot_mode_queue.Send(&shoot_mode, 0_ms);
        }
    }

}

// ------ 板通发送数据包实现
namespace board_com {
void chassis2gimbal_pkg::Pack() {
    _data.SOF1 = 0xa5, _data.SOF2 = 0x5a;
    _data.robot_id = referee_data.robotPerf_.ID;
    _data.shoot_spd_referee = referee_data.shootStatus_.initial_speed;
    _data.ShooterOutput = referee_data.robotPerf_.IsShooterOutput;
    _data.GimbalOutput = referee_data.robotPerf_.IsGimbalOutput;
    _data.GameProcess = referee_data.gameStatus_.GameProcess;
    _data.RemainSecond = referee_data.gameStatus_.StageRemainTimeSecond;
}

void chassis2gimbal_pkg::FeederInitDone() { _data.init_finished = 1; }

void chassis2gimbal_pkg::FeederInitReset() { _data.init_finished = 0; }
}  // namespace board_com