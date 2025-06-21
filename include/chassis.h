/**
 * @file           : chassis.h
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-4-4 下午2:27
 * @lastEditor     :
 * @lastEditTime   : 25-4-4 下午2:27
 */

#pragma once
#include <units.h>
using namespace units;
using namespace units::literals;

namespace chassis {

// 车身常数
constexpr auto wheel_radius = 0.06_m;     // 轮半径
constexpr auto chassis_radius = 0.225_m;  // 车半径
float constexpr sqrt2 = 1.4142135623730950488016887242097F;

#define WHEEL_FORWARD_LEFT_CAN_ID 0x203
#define WHEEL_FORWARD_RIGHT_CAN_ID 0x201
#define WHEEL_BACKWARD_RIGHT_CAN_ID 0x202
#define WHEEL_BACKWARD_LEFT_CAN_ID 0x204

#define RUDDER_FORWARD_LEFT_CAN_ID 0x208
#define RUDDER_FORWARD_RIGHT_CAN_ID 0x207
#define RUDDER_BACKWARD_RIGHT_CAN_ID 0x206
#define RUDDER_BACKWARD_LEFT_CAN_ID 0x205

constexpr float RUDDER_FORWARD_LEFT_INSTALL_ANGLE = 253.652344F;
constexpr float RUDDER_FORWARD_RIGHT_INSTALL_ANGLE = (98.3496094F);
constexpr float RUDDER_BACKWARD_RIGHT_INSTALL_ANGLE = 193.579102F;
constexpr float RUDDER_BACKWARD_LEFT_INSTALL_ANGLE = 224.25293F;

#define YAW_CAN_ID 0x205

constexpr float kWheelMotorDecRation = 268.0f / 17.0f;
constexpr float kTorqueCurrentConstant = 0.3F / (3591.F / 187.F) * kWheelMotorDecRation;
constexpr float kOutputCurrentConstant = 16384.0F / 20.0F;
constexpr auto kVehicleMass = 19.8_kg;             // 车身质量
constexpr auto kGravity = 9.81_mps_sq;             // 重力加速度
constexpr auto kGravityHeight = 0.184_m + 0.1_m;  // 重心高度
constexpr torque::newton_meter_t torque_fd_params[4] = {0.036462364_Nm, 0.0358546_Nm, 0.0507434_Nm,
                                                        0.0419317_Nm};  // 力矩前馈参数
constexpr int16_t current_fd_params[4] = {981, 1055, 443, 386};         // 电流前馈参数
// 小陀螺速度改为根据等级来变化，按照10个等级写10个速度
inline angular_velocity::radians_per_second_t Gyro_Speed_Set[10] = {
    7.5_rad_per_s, 7.5_rad_per_s, 7.5_rad_per_s, 7.5_rad_per_s,  8.0_rad_per_s,
    8.5_rad_per_s, 9.0_rad_per_s, 9.5_rad_per_s, 10.5_rad_per_s, 11.5_rad_per_s};  // 小陀螺速度
constexpr auto Gyro_Shift_scale = 1.25F;                                           // 小陀螺速度比例系数
constexpr auto Gyro_Speed = 8.0_rad_per_s;                                         // 小陀螺速度（都给你⑨完了）
constexpr auto Upper_Gyro_Speed = 12.5_rad_per_s;                                  // 加速小陀螺
constexpr auto follow_dead_zone = 2.5_deg;                                         // 跟随模式死区
constexpr auto lock_time = 2000_ms;

constexpr auto max_wheel_spd = 469.0_rpm * (19.0F / kWheelMotorDecRation) - 30.0_rpm;  // 电机最大转速-保守因子
constexpr auto max_chassis_spd = 3.45_mps;                                             // 底盘最大线速度 * 保守因子
constexpr auto constant_rolling_force = 0.2_N;                                         // 摩擦力常数
constexpr auto proportional_friction_constant = 0.1_N / 1_mps;                         // 比例摩擦系数
constexpr auto acc_scale_offset = 1.0F;                                                // 加速度比例偏置项



struct ChassisRef_t {
    velocity::meters_per_second_t vx, vy;
    angular_velocity::radians_per_second_t vw;
};

struct Chassis_Spd_t {
    velocity::meters_per_second_t vx, vy;
    angular_velocity::radians_per_second_t w;
};

// 底盘状态枚举值
enum class ZeroPointMode {
    KEEP = 0,         // 保持
    FIXED = 1,        // 锁定
    CHASSIS_GYRO = 2  //
};

// 底盘模式枚举值
enum class ChassisMode : uint8_t {
    STOP = 0,
    NORMAL = 1,
    GYRO = 2,
    FOLLOW = 3,
};

enum class ControlMode : uint8_t {
    SPEED_CTRL = 0,
    FORCE_CTRL = 1,
};

Chassis_Spd_t Follow(Chassis_Spd_t const& ref);
Chassis_Spd_t GYRO(Chassis_Spd_t const& ref);
Chassis_Spd_t Normal(Chassis_Spd_t const& ref);
Chassis_Spd_t STOP();

ControlMode GetCtrlMode();

bool ifAllMotorEnable();

class stribeck_friction {
   private:
    float tau_s{};  // 静摩擦
    float tau_c{};  // 动摩擦
    float vs = {};  // Stribeck 速度常数
    float b = {};   // 粘性阻尼系数
   public:
    explicit stribeck_friction(float const tau_s, float const tau_c, float const vs, float const b)
        : tau_s(tau_s), tau_c(tau_c), vs(vs), b(b) {}
    stribeck_friction() = default;

    [[nodiscard]] float get_friction(float const omega) const {
        float const abs_omega = std::abs(omega);
        float const exp_decay = std::exp(-std::pow(abs_omega / vs, 2));
        float const friction = (tau_s + (tau_c - tau_s) * exp_decay) * std::copysign(1.0, omega) + b * omega;

        return friction;
    }
};

}  // namespace chassis

namespace powerCtrl {

//
enum class power_mode_t : uint8_t {
    MAX_OUT = 0,     // 最大功率模式
    ECO = 1,         // 节能模式
    CUSTOMIZED = 2,  // 自定义功率模式
    EMERGENCY = 3,   // 紧急模式
};

void SetPowerMode(power_mode_t mode);

}  // namespace powerCtrl
