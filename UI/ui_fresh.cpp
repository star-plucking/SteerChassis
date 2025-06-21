/**
 * @file           : ui_fresh.cpp
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-5-7 下午2:50
 * @lastEditor     :
 * @lastEditTime   : 25-5-7 下午2:50
 */
#include "BoardCom.h"
#include "DjiMotors.h"
#include "FSM.h"
#include "cap.h"
#include "clock.h"
#include "sheriff_os.h"
#include "thread.hpp"
#include "ui.h"

extern etl::atomic<float> sum_power_chassis;
extern etl::atomic<float> power_target_ref;
namespace chassis {
extern etl::atomic<units::velocity::meters_per_second_t> chassis_now_speed;
}

namespace chassis::logic {
// ---- 底盘状态机
extern os::StateMachine Chassis_FSM;
}  // namespace chassis::logic

namespace chassis_power::logic {
extern os::StateMachine PowerCtrl_FSM;
}

namespace device::dji_motors {
extern Motor GimbalMotor;
}

namespace ui {
constexpr float cap_sum_length = 1202 - 720;
constexpr float power_length = 77;

// 大风车吱呀吱呀滴转
constexpr float center_x = 960;                              // (910 + 1010) / 2
constexpr float center_y = 150;                              // (100 + 200) / 2
constexpr float right_line_dx1 = -50, right_line_dy1 = -50;  // 右线起点相对中心点(-50, -50)
constexpr float right_line_dx2 = 50, right_line_dy2 = 50;    // 右线终点相对中心点(50, 50)
constexpr float left_line_dx1 = 50, left_line_dy1 = -50;     // 左线起点相对中心点(50, -50)
constexpr float left_line_dx2 = -50, left_line_dy2 = 50;     // 左线终点相对中心点(-50, 50)

// 右侧小车
constexpr float pitch_height = 593.0F - 537.0F;                // pitch角度
constexpr float pitch_width = 1620.0F - 1552.0F;               // pitch宽度
constexpr float pitch_width_right_length = 1620.0F - 1568.0F;  // pitch宽度右侧长度
constexpr float pitch_width_left_length = 1568.0F - 1552.0F;   // pitch宽度左侧长度

constexpr auto pitch_fold_angle = 60_deg;  // pitch折叠角度

// 各种矩形的初始值
constexpr float GYRORec_start_x = 482;
constexpr float GYRORec_start_y = 739;

constexpr float FollowRec_start_x = 482;
constexpr float FollowRec_start_y = 571;

constexpr float ShooterRec_start_x = 482;
constexpr float ShooterRec_start_y = 695;

constexpr float ForceRec_start_x = 482;
constexpr float ForceRec_start_y = 780;

constexpr float BC_start_x = 482;
constexpr float BC_start_y = 657;

constexpr float VTM_start_x = 482;
constexpr float VTM_start_y = 614;

enum class UI_Color : uint8_t {
    MAIN = 0,
    Yellow = 1,
    Green = 2,
    Orange = 3,
    Purple = 4,
    Pink = 5,
    Cyan = 6,
    Black = 7,
    White = 8,
};

#define eco_str "ECO   "
#define custom_str "CUSTOM"
#define maxout_str "MAXOUT"

// 自瞄字符串(注意对齐空格)
#define no_aim "NO_AIM    "
#define armor "ARMOR     "
#define small_buffer "SM_BUFFER "
#define big_buffer "BIG_BUFFER"

void Draw_car() {
    // 动画
    // 更新小车车的两个pitch && 更新折叠状态
    static float pitch_width_center_point_x = 1568.0F,  //  pitch宽度中心点x
        pitch_width_center_point_y = 593.0F;            // pitch宽度中心点y
    static float current_fold_angle = 0.0F;             // 当前折叠角度
    static bool is_folding = false;                     // 是否正在折叠
    static bool was_folded = false;                     // 之前的折叠状态
    static auto fold_start_time = 0_ms;                 // 折叠开始时间
    // 检查折叠状态是否改变
    bool const is_folded =
        board_com::gimbal2chassis_pkg::Instance().GetData().fold_flag != board_com::FoldState::Unfold;
    if (is_folded != was_folded) {
        // 折叠状态发生变化，开始动画
        is_folding = true;
        fold_start_time = os::GetOsTime();
        was_folded = is_folded;
    }
    // 如果正在折叠过程中，计算当前角度
    if (is_folding) {
        auto const current_time = os::GetOsTime();

        if (auto const elapsed_time = current_time - fold_start_time; elapsed_time >= 500_ms) {
            // 动画结束
            is_folding = false;
            current_fold_angle = is_folded ? pitch_fold_angle.value() : 0.0F;
        } else {
            // 按时间比例计算当前角度 (0-500ms)
            float const progress = static_cast<float>(elapsed_time) / 500.0F;  // 0.0-1.0的进度

            if (is_folded) {
                // 从0到pitch_fold_angle
                current_fold_angle = progress * pitch_fold_angle.value();
            } else {
                // 从pitch_fold_angle到0
                current_fold_angle = (1.0F - progress) * pitch_fold_angle.value();
            }
        }
    } else {
        // 不在动画中，直接使用目标角度
        current_fold_angle = is_folded ? pitch_fold_angle.value() : 0.0F;
    }

    // 使用当前计算出的角度更新UI
    if (current_fold_angle < 0.01F) {
        // 完全展开状态
        ui_g_Ungroup_pitchLow->end_x = ui_g_Ungroup_pitchLow->start_x;
        ui_g_Ungroup_pitchLow->end_y = ui_g_Ungroup_pitchLow->start_y + pitch_height;
    } else {
        // 部分或完全折叠状态
        ui_g_Ungroup_pitchLow->end_x =
            ui_g_Ungroup_pitchLow->start_x -
            os::math::sin(units::make_unit<angle::degree_t>(current_fold_angle)) * pitch_height;
        ui_g_Ungroup_pitchLow->end_y =
            ui_g_Ungroup_pitchLow->start_y +
            os::math::cos(units::make_unit<angle::degree_t>(current_fold_angle)) * pitch_height;
    }

    // 小车高俩pitch的角度
    pitch_width_center_point_x = ui_g_Ungroup_pitchLow->end_x;  // pitch宽度中心点x
    pitch_width_center_point_y = ui_g_Ungroup_pitchLow->end_y;  // pitch宽度中心点y
    float pitch_angle = board_com::gimbal2chassis_pkg::Instance().GetData().pitch_angle;
    ui_g_Ungroup_PitchHigh->start_x =
        pitch_width_center_point_x -
        pitch_width_left_length * os::math::cos(units::make_unit<angle::degree_t>(pitch_angle));
    ui_g_Ungroup_PitchHigh->start_y =
        pitch_width_center_point_y -
        pitch_width_left_length * os::math::sin(units::make_unit<angle::degree_t>(pitch_angle));
    ui_g_Ungroup_PitchHigh->end_x =
        pitch_width_center_point_x +
        pitch_width_right_length * os::math::cos(units::make_unit<angle::degree_t>(pitch_angle));
    ui_g_Ungroup_PitchHigh->end_y =
        pitch_width_center_point_y +
        pitch_width_right_length * os::math::sin(units::make_unit<angle::degree_t>(pitch_angle));
    ui_g_Ungroup_GM->start_x = ui_g_Ungroup_PitchHigh->start_x;
    ui_g_Ungroup_GM->start_y = ui_g_Ungroup_PitchHigh->start_y;
}
class UI {
   private:
    UART_HandleTypeDef* huart{};

   public:
    explicit UI(UART_HandleTypeDef* huart) : huart(huart) {}
    ~UI() = default;

    static void init() { ui_init_g(); }
    static void update() {
        float cap_length =
            cap_sum_length *
            (static_cast<float>(cap::cap2chassis_community::Instance().GetData()->cap_rest_energy) / 100.0F);
        if (cap_length <= 0) {
            cap_length = 0;
        }
        ui_g_Ungroup_capLine->end_x = ui_g_Ungroup_capLine->start_x + cap_length;
        ui_g_Ungroup_RestCap->number = cap::cap2chassis_community::Instance().GetData()->cap_rest_energy;

        // 功率条
        float power_scale = sum_power_chassis.load() / 390.0F * 70.0F;
        if (power_scale >= 70.0F or power_scale < 1.2F) {
            power_scale = 1.2F;
        }
        ui_g_Ungroup_RightArc->end_angle = ui_g_Ungroup_RightArc->start_angle + power_scale;
        // 功率数显
        ui_g_Ungroup_Power->number = sum_power_chassis;

        // 速度条
        float speed_scale = chassis::chassis_now_speed.load().value() / 4.0 * 70.0F;
        if (speed_scale >= 70.0F or speed_scale < 1.2F) {
            speed_scale = 1.2F;
        }
        ui_g_Ungroup_LeftArc->end_angle = ui_g_Ungroup_LeftArc->start_angle + speed_scale;
        // 速度数显
        ui_g_Ungroup_speed->number = static_cast<int32_t>(chassis::chassis_now_speed.load() * 10);

        float const gimbal_angle = device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle();

        // 计算旋转角度（将角度转换为弧度）
        float const rotation_angle = gimbal_angle * static_cast<float>(M_PI) / 180.0f;
        float const cos_angle = cos(rotation_angle);
        float const sin_angle = sin(rotation_angle);

        // 应用旋转变换 - 右线起点
        float const new_right_x1 = center_x + (right_line_dx1 * cos_angle - right_line_dy1 * sin_angle);
        float const new_right_y1 = center_y + (right_line_dx1 * sin_angle + right_line_dy1 * cos_angle);
        // 应用旋转变换 - 右线终点
        float const new_right_x2 = center_x + (right_line_dx2 * cos_angle - right_line_dy2 * sin_angle);
        float const new_right_y2 = center_y + (right_line_dx2 * sin_angle + right_line_dy2 * cos_angle);

        // 应用旋转变换 - 左线起点
        float const new_left_x1 = center_x + (left_line_dx1 * cos_angle - left_line_dy1 * sin_angle);
        float const new_left_y1 = center_y + (left_line_dx1 * sin_angle + left_line_dy1 * cos_angle);
        // 应用旋转变换 - 左线终点
        float const new_left_x2 = center_x + (left_line_dx2 * cos_angle - left_line_dy2 * sin_angle);
        float const new_left_y2 = center_y + (left_line_dx2 * sin_angle + left_line_dy2 * cos_angle);

        // 更新右线位置
        ui_g_Ungroup_RIghtLine->start_x = static_cast<uint32_t>(new_right_x1);
        ui_g_Ungroup_RIghtLine->start_y = static_cast<uint32_t>(new_right_y1);
        ui_g_Ungroup_RIghtLine->end_x = static_cast<uint32_t>(new_right_x2);
        ui_g_Ungroup_RIghtLine->end_y = static_cast<uint32_t>(new_right_y2);

        // 更新左线位置
        ui_g_Ungroup_LeftLine->start_x = static_cast<uint32_t>(new_left_x1);
        ui_g_Ungroup_LeftLine->start_y = static_cast<uint32_t>(new_left_y1);
        ui_g_Ungroup_LeftLine->end_x = static_cast<uint32_t>(new_left_x2);
        ui_g_Ungroup_LeftLine->end_y = static_cast<uint32_t>(new_left_y2);

        // 更新LED位置
        ui_g_Ungroup_LED->start_x = static_cast<uint32_t>(new_right_x2);
        ui_g_Ungroup_LED->start_y = static_cast<uint32_t>(new_right_y2);
        ui_g_Ungroup_LED->end_x = static_cast<uint32_t>(new_left_x1);
        ui_g_Ungroup_LED->end_y = static_cast<uint32_t>(new_left_y1);

        // 优化自瞄字符串和自瞄框逻辑
        bool is_aiming = false;
        switch (board_com::gimbal2chassis_pkg::Instance().GetData().yaw_mode) {
            case gimbal::YawMode::BIGBUFF_AIM:
                strcpy(ui_g_Ungroup_aimmode->string, big_buffer);
                is_aiming = true;
                break;
            case gimbal::YawMode::SMALLBUFF_AIM:
                strcpy(ui_g_Ungroup_aimmode->string, small_buffer);
                is_aiming = true;
                break;
            case gimbal::YawMode::ARMOR_AIM:
                strcpy(ui_g_Ungroup_aimmode->string, armor);
                is_aiming = true;
                break;
            case gimbal::YawMode::CONTROL:
                strcpy(ui_g_Ungroup_aimmode->string, no_aim);
                is_aiming = false;
                break;
            default:
                break;
        }

        // 自瞄框
        if (is_aiming) {
            ui_g_Ungroup_aim_rec->color = static_cast<uint8_t>(
                board_com::gimbal2chassis_pkg::Instance().GetData().is_get_target == 1 ? UI_Color::Purple
                                                                                       : UI_Color::Yellow);
            ui_g_Ungroup_aim_rec->width = 3;
        } else {
            ui_g_Ungroup_aim_rec->color = static_cast<uint8_t>(UI_Color::Cyan);
            ui_g_Ungroup_aim_rec->width = 1;
        }

        // 功率目标更新
        ui_g_Ungroup_Target_P->number = static_cast<int32_t>(power_target_ref);

        // 更新功率字符串
        if (chassis_power::logic::PowerCtrl_FSM.getCurrentState() == "eco") {
            strcpy(ui_g_Ungroup_Power_mode_out->string, eco_str);
        } else if (chassis_power::logic::PowerCtrl_FSM.getCurrentState() == "customized") {
            strcpy(ui_g_Ungroup_Power_mode_out->string, custom_str);
        } else if (chassis_power::logic::PowerCtrl_FSM.getCurrentState() == "max_out") {
            strcpy(ui_g_Ungroup_Power_mode_out->string, maxout_str);
        } else {
            strcpy(ui_g_Ungroup_Power_mode_out->string, eco_str);
        }

        // 更新pitch角度
        ui_g_Ungroup_pitch->number =
            static_cast<int32_t>(board_com::gimbal2chassis_pkg::Instance().GetData().pitch_angle * 1000);

        // 更新力控状态
        if (chassis::GetCtrlMode() == chassis::ControlMode::FORCE_CTRL) {
            ui_g_Ungroup_ForceRec->start_x = ForceRec_start_x + 8;
            ui_g_Ungroup_ForceRec->start_y = ForceRec_start_y + 8;
            ui_g_Ungroup_ForceRec->end_x = ui_g_Ungroup_ForceRec->start_x + 16;
            ui_g_Ungroup_ForceRec->end_y = ui_g_Ungroup_ForceRec->start_y + 16;
            ui_g_Ungroup_ForceRec->width = 16;
        } else {
            ui_g_Ungroup_ForceRec->start_x = ForceRec_start_x;
            ui_g_Ungroup_ForceRec->start_y = ForceRec_start_y;
            ui_g_Ungroup_ForceRec->end_x = ui_g_Ungroup_ForceRec->start_x + 33;
            ui_g_Ungroup_ForceRec->end_y = ui_g_Ungroup_ForceRec->start_y + 33;
            ui_g_Ungroup_ForceRec->width = 1;
        }

        // 更新陀螺状态
        if (chassis::logic::Chassis_FSM.getCurrentState() == "gyro") {
            // 填充方块
            ui_g_Ungroup_GYRORec->start_x = GYRORec_start_x + 8;
            ui_g_Ungroup_GYRORec->start_y = GYRORec_start_y + 8;
            ui_g_Ungroup_GYRORec->end_x = ui_g_Ungroup_GYRORec->start_x + 16;
            ui_g_Ungroup_GYRORec->end_y = ui_g_Ungroup_GYRORec->start_y + 16;
            ui_g_Ungroup_GYRORec->width = 16;
        } else {
            // 填��空白
            ui_g_Ungroup_GYRORec->start_x = GYRORec_start_x;
            ui_g_Ungroup_GYRORec->start_y = GYRORec_start_y;
            ui_g_Ungroup_GYRORec->end_x = ui_g_Ungroup_GYRORec->start_x + 33;
            ui_g_Ungroup_GYRORec->end_y = ui_g_Ungroup_GYRORec->start_y + 33;
            ui_g_Ungroup_GYRORec->width = 1;
        }

        if (chassis::logic::Chassis_FSM.getCurrentState() == "follow") {
            // 填充方块
            ui_g_Ungroup_follow_rec->start_x = FollowRec_start_x + 8;
            ui_g_Ungroup_follow_rec->start_y = FollowRec_start_y + 8;
            ui_g_Ungroup_follow_rec->end_x = ui_g_Ungroup_follow_rec->start_x + 16;
            ui_g_Ungroup_follow_rec->end_y = ui_g_Ungroup_follow_rec->start_y + 16;
        } else {
            // 填充空白
            ui_g_Ungroup_follow_rec->start_x = FollowRec_start_x;
            ui_g_Ungroup_follow_rec->start_y = FollowRec_start_y;
            ui_g_Ungroup_follow_rec->end_x = ui_g_Ungroup_follow_rec->start_x + 33;
            ui_g_Ungroup_follow_rec->end_y = ui_g_Ungroup_follow_rec->start_y + 33;
        }

        // 更新发射器状态
        if (board_com::gimbal2chassis_pkg::Instance().GetData().shooter_spd > 10.0F) {
            // 填充方块
            ui_g_Ungroup_ShooterRec->start_x = ShooterRec_start_x + 8;
            ui_g_Ungroup_ShooterRec->start_y = ShooterRec_start_y + 8;
            ui_g_Ungroup_ShooterRec->end_x = ui_g_Ungroup_ShooterRec->start_x + 16;
            ui_g_Ungroup_ShooterRec->end_y = ui_g_Ungroup_ShooterRec->start_y + 16;
            ui_g_Ungroup_ShooterRec->width = 16;
        } else {
            // 填充空白
            ui_g_Ungroup_ShooterRec->start_x = ShooterRec_start_x;
            ui_g_Ungroup_ShooterRec->start_y = ShooterRec_start_y;
            ui_g_Ungroup_ShooterRec->end_x = ui_g_Ungroup_ShooterRec->start_x + 33;
            ui_g_Ungroup_ShooterRec->end_y = ui_g_Ungroup_ShooterRec->start_y + 33;
            ui_g_Ungroup_ShooterRec->width = 1;
        }

        if (not board_com::gimbal2chassis_pkg::Instance().isLost()) {
            ui_g_Ungroup_BC_Rec->start_x = BC_start_x + 8;
            ui_g_Ungroup_BC_Rec->start_y = BC_start_y + 8;
            ui_g_Ungroup_BC_Rec->end_x = ui_g_Ungroup_BC_Rec->start_x + 16;
            ui_g_Ungroup_BC_Rec->end_y = ui_g_Ungroup_BC_Rec->start_y + 16;
            ui_g_Ungroup_BC_Rec->width = 16;
        } else {
            ui_g_Ungroup_BC_Rec->start_x = BC_start_x;
            ui_g_Ungroup_BC_Rec->start_y = BC_start_y;
            ui_g_Ungroup_BC_Rec->end_x = ui_g_Ungroup_BC_Rec->start_x + 33;
            ui_g_Ungroup_BC_Rec->end_y = ui_g_Ungroup_BC_Rec->start_y + 33;
            ui_g_Ungroup_BC_Rec->width = 1;
        }

        if (board_com::gimbal2chassis_pkg::Instance().GetData().vtm_com_state) {
            ui_g_Ungroup_VTM_Rec->start_x = VTM_start_x + 8;
            ui_g_Ungroup_VTM_Rec->start_y = VTM_start_y + 8;
            ui_g_Ungroup_VTM_Rec->end_x = ui_g_Ungroup_VTM_Rec->start_x + 16;
            ui_g_Ungroup_VTM_Rec->end_y = ui_g_Ungroup_VTM_Rec->start_y + 16;
            ui_g_Ungroup_VTM_Rec->width = 16;
        } else {
            ui_g_Ungroup_VTM_Rec->start_x = VTM_start_x;
            ui_g_Ungroup_VTM_Rec->start_y = VTM_start_y;
            ui_g_Ungroup_VTM_Rec->end_x = ui_g_Ungroup_VTM_Rec->start_x + 33;
            ui_g_Ungroup_VTM_Rec->end_y = ui_g_Ungroup_VTM_Rec->start_y + 33;
            ui_g_Ungroup_VTM_Rec->width = 1;
        }

        Draw_car();

        ui_update_g();

        // TODO:
    }

    static void remove_ui() { ui_delete_layer(2, 0); }
};

}  // namespace ui

uint8_t last_UI_flag = 0;
CREATE_THREAD_STATIC(ui_thread, 1024, NULL, 5) {
    while (ui_self_id == 0) {
        os::Sleep(10_ms);
    }
    ui::UI::remove_ui();
    ui::UI::init();
    for (;;) {
        if (last_UI_flag != board_com::gimbal2chassis_pkg::Instance().GetData().ui_cmd) {
            ui::UI::remove_ui();
            ui::UI::init();
            os::Sleep(10_ms);
        } else {
            ui::UI::update();
        }
        last_UI_flag = board_com::gimbal2chassis_pkg::Instance().GetData().ui_cmd;
        // os::Sleep(1_ms);
        osDelay(1);
    }
}
