/**
 * @file           : chassis.cpp
 * @author         : Star_Plucking
 * @brief          : None
 * @date           : 25-4-4 下午2:36
 * @lastEditor     :
 * @lastEditTime   : 25-4-4 下午2:36
 */

/**
 * 底盘坐标系示意图
 * +---------+
 * | 0     3 |
 * |    ^    |       ↑ x
 * |    |    |       |
 * | 1     2 |  y<---+
 * +---------+
 *
 * 四个编号对应四个舵轮位置:
 * 0: 左前轮  1: 左后轮
 * 2: 右后轮  3: 右前轮
 * 底盘的设计思想是高度解耦的集成控制
 */

#include <algorithm/math.h>
// #include <algorithm/physicalVector.h>
#include "AppMessageQueues.h"
#include "units.h"
#include <algorithm/filter.h>
#include <algorithm/pid.h>
#include <algorithm/powerCtrl.h>
#include <etl/math.h>
#include <BoardCom.h>
#include <DjiMotors.h>
#include <FSM.h>
#include <bsp.h>
#include <chassis.h>
#include <referee.h>

using namespace units::literals;  // 这并非一种很安全的做法，可能会引入一些不必要的单位转换
using namespace units;

os::MsgQueue<int8_t, 5> gyro_direction_queue;
extern dji::DjiReferee referee_data;

//------ 构造底盘电机
namespace device::dji_motors {
extern Motor GimbalMotor;

Motor wheel_motor[4] = {Motor(WHEEL_FORWARD_LEFT_CAN_ID, chassis::kWheelMotorDecRation, 0.0F, DJIMotorType::kM3508),
                        Motor(WHEEL_BACKWARD_LEFT_CAN_ID, chassis::kWheelMotorDecRation, 0.0F, DJIMotorType::kM3508),
                        Motor(WHEEL_BACKWARD_RIGHT_CAN_ID, chassis::kWheelMotorDecRation, 0.0F, DJIMotorType::kM3508),
                        Motor(WHEEL_FORWARD_RIGHT_CAN_ID, chassis::kWheelMotorDecRation, 0.0F, DJIMotorType::kM3508)};
Motor rudder_motor[4] = {
    Motor(RUDDER_FORWARD_LEFT_CAN_ID, 1.0F, chassis::RUDDER_FORWARD_LEFT_INSTALL_ANGLE, DJIMotorType::kGM6020i),
    Motor(RUDDER_BACKWARD_LEFT_CAN_ID, 1.0F, chassis::RUDDER_BACKWARD_LEFT_INSTALL_ANGLE, DJIMotorType::kGM6020i),
    Motor(RUDDER_BACKWARD_RIGHT_CAN_ID, 1.0F, chassis::RUDDER_BACKWARD_RIGHT_INSTALL_ANGLE, DJIMotorType::kGM6020i),
    Motor(RUDDER_FORWARD_RIGHT_CAN_ID, 1.0F, chassis::RUDDER_FORWARD_RIGHT_INSTALL_ANGLE, DJIMotorType::kGM6020i)};

// 电机组
MotorManager wheel_group(M3508TxID_1, M3508RxID_1, &os::Can::Instance(FDCAN1),
                         {wheel_motor[0], wheel_motor[1], wheel_motor[2], wheel_motor[3]});
MotorManager rudder_group(GM6020iTxID_1, GM6020RxID_1, &os::Can::Instance(FDCAN3),
                          {rudder_motor[0], rudder_motor[1], rudder_motor[2], rudder_motor[3]});

// 底盘速度估计器

}  // namespace device::dji_motors

// 注册电机回调函数
InitLate(SetChassisMotorCallback) {
    os::Can::Instance(FDCAN1).on_receive.connect([&](FDCAN_RxHeaderTypeDef const* rxHeader, uint8_t const* rxBuffer) {
        device::dji_motors::wheel_group.DecodeData(rxHeader->Identifier, rxBuffer);
    });
    os::Can::Instance(FDCAN3).on_receive.connect([&](FDCAN_RxHeaderTypeDef const* rxHeader, uint8_t const* rxBuffer) {
        device::dji_motors::rudder_group.DecodeData(rxHeader->Identifier, rxBuffer);
    });
}

// 底盘PID参数初始化
namespace chassis {

PIDController rudder_angle_pid[4]{
    PIDController({6.5f, 0.35f, 0.35f, 60.0f, 120.0f, 90.0f}), PIDController({6.5f, 0.25f, 0.1f, 70.0f, 120.0f, 90.0f}),
    PIDController({7.0f, 0.35f, 0.1f, 80.0f, 120.0f, 90.0f}), PIDController({7.0F, 0.25F, 0, 80.0F, 120.0F, 90.0F})};

PIDFeedforwardController wheel_speed_pid_forceCtrl[4]{
    PIDFeedforwardController({65.0f, 0.0f, 0.0f, 0.0f, 16000.0f, 10000.0f}),
    PIDFeedforwardController({65.0f, 0.0f, 0.0f, 0.0f, 16000.0f, 10000.0f}),
    PIDFeedforwardController({65.0f, 0.0f, 0.0f, 0.0f, 16000.0f, 10000.0f}),
    PIDFeedforwardController({65.0f, 0.0f, 0.0f, 0.0F, 16000.0f, 10000.0f})};
PIDFeedforwardController wheel_speed_pid_spdCtrl[4]{
    PIDFeedforwardController({80.0f, 1.0f, 0.0f, 0.0f, 16000.0f, 10000.0f}),
    PIDFeedforwardController({80.0f, 1.0f, 0.0f, 0.0f, 16000.0f, 10000.0f}),
    PIDFeedforwardController({80.0f, 1.0f, 0.0f, 0.0f, 16000.0f, 10000.0f}),
    PIDFeedforwardController({80.0f, 1.0f, 0.0f, 0.0F, 16000.0f, 10000.0f})};
PIDFeedforwardController wheel_speed_pid_gyroCtrl[4]{
    PIDFeedforwardController({120.0f, 2000.0f, 0.0f, 1000.0f, 16383.0f, 10000.0f}),
    PIDFeedforwardController({120.0f, 2000.0f, 0.0f, 1000.0f, 16383.0f, 10000.0f}),
    PIDFeedforwardController({120.0f, 2000.0f, 0.0f, 1000.0f, 16383.0f, 10000.0f}),
    PIDFeedforwardController({120.0f, 2000.0f, 0.0f, 1000.0F, 16383.0f, 10000.0f})};

PIDController rudder_speed_pid[4]{
    PIDController({65.0f, 0.35f, 0.0f, 8000, 12000, 30000}), PIDController({65.0f, 0.35f, 0.0f, 8000, 12000, 30000}),
    PIDController({65.0f, 0.35f, 0.0f, 8000, 12000, 30000}), PIDController({65.0f, 0.35f, 0.0f, 8000, 12000, 30000})};

// ------- 底盘速度PID
PIDController chassis_speed_x_pid({24, 1, 0, 200, 300, 300});  // I太大容易打滑内耗
PIDController chassis_speed_y_pid({24, 1, 0, 200, 300, 300});
PIDController chassis_speed_w_pid({8, 0, 0, 200, 0, 300});  // 由于正解算多解问题，会造成假反馈破坏底盘运动
// ------- 速度斜坡
os::math::SlopeCtrl chassis_speed_x_slope(3600.0f / 1000.0F, 3600.0f / 1000.0F);
os::math::SlopeCtrl chassis_speed_y_slope(3600.0f / 1000.0F, 3600.0f / 1000.0F);
os::math::SlopeCtrl chassis_speed_w_slope(18000.0f / 1000.0F, 18000.0f / 1000.0F);
// 六百六十六
// ------- 随动模式PID
PIDController follow_angle_pid({0.2f, 0.0f, 0.1f, 1.0f, 8.0f, 200});
PIDController follow_spd_pid({0.4f, 0.0f, 0.3f, 1.0f, 12.0f, 200});
}  // namespace chassis

namespace chassis_power::logic {
extern os::StateMachine PowerCtrl_FSM;
}

namespace chassis::logic {
// ---- 底盘状态机
extern os::StateMachine Chassis_FSM;
}  // namespace chassis::logic

namespace chassis {

// ------ 舵电机
angle::degree_t rudder_angle_ref[4] = {0.0_deg, 0.0_deg, 0.0_deg, 0.0_deg};  // 舵电机角度期望
angle::degree_t rudder_angle[4]{};                                           // 舵电机角度反馈

// ----- 轮电机
torque::newton_meter_t wheel_torque_ref[4] = {0.0_Nm, 0.0_Nm, 0.0_Nm, 0.0_Nm};  // 轮电机力矩期望
torque::newton_meter_t wheel_torque_fdb[4]{};                                   // 轮电机力矩反馈
angular_velocity::revolutions_per_minute_t wheel_spd_fdb[4]{};                  // 轮电机速度反馈
angular_velocity::revolutions_per_minute_t wheel_spd_ref[4]{};                  // 轮电机速度期望

// ----- 底盘
etl::atomic<ControlMode> chassis_control_mode = ControlMode::FORCE_CTRL;       // 底盘控制模式
force::newton_t chassis_force_ref[2] = {0.0_N, 0.0_N};                         // 底盘力期望
force::newton_t chassis_force_fdb[2] = {0.0_N, 0.0_N};                         // 底盘力反馈
torque::newton_meter_t chassis_torque_ref = 0.0_Nm;                            // 底盘转矩期望
force::newton_t chassis_slope_compensation[2] = {0.0_N, 0.0_N};                // 底盘斜坡补偿
acceleration::meters_per_second_squared_t chassis_acceleration_estimate[2]{};  // 底盘期望加速度估计
acceleration::meters_per_second_squared_t chassis_acceleration_fdb[2]{};       // 底盘加速度反馈(差分近似)
acceleration::meters_per_second_squared_t max_allowed_acceleration{};          // 底盘容许最大加速度
angle::degree_t chassis_rolling_degree = 0.0_deg;                              // 底盘滚转角度
Chassis_Spd_t chassis_ref;                                                     // 底盘速度期望
Chassis_Spd_t chassis_fdb;                                                     // 底盘速度反馈

// ----- 其他
int8_t Gyro_Directions = 1;  // 小陀螺方向

/**
 * @brief 获取底盘电机所有反馈
 * @note 存储在wheel_spd和rudder_angle中
 */
void GetAllMotorFdb() {
    for (int i = 0; i < 4; ++i) {
        wheel_spd_fdb[i] = units::make_unit<angular_velocity::revolutions_per_minute_t>(
            device::dji_motors::wheel_motor[i].m_Encoder.GetShaftSpeed());
        wheel_torque_fdb[i] = units::make_unit<torque::newton_meter_t>(
            device::dji_motors::wheel_motor[i].m_Encoder.GetCurrent() * kTorqueCurrentConstant);
        rudder_angle[i] =
            units::make_unit<angle::degree_t>(device::dji_motors::rudder_motor[i].m_Encoder.GetInvLimitedAngle());
    }
    // 2,3号电机的转速是反向的
    wheel_spd_fdb[2] = -wheel_spd_fdb[2];
    wheel_spd_fdb[3] = -wheel_spd_fdb[3];
}

/**
 * @brief 获取底盘期望值
 */
void Get_chassis_ref() {
    Chassis_Spd_t chassis_ref_temp{};
    // 获取底盘期望值
    if (os::chassis_ref_queue.ReceiveTo(&chassis_ref_temp, 0_ms)) {
        velocity::meters_per_second_t const ref_speed =
            math::sqrt(chassis_ref_temp.vx * chassis_ref_temp.vx + chassis_ref_temp.vy * chassis_ref_temp.vy);
        if (ref_speed > max_chassis_spd) {
            float const scale = max_chassis_spd() / ref_speed();
            chassis_ref_temp.vx = make_unit<velocity::meters_per_second_t>(chassis_ref_temp.vx() * scale);
            chassis_ref_temp.vy = make_unit<velocity::meters_per_second_t>(chassis_ref_temp.vy() * scale);
        }
        chassis_ref.vx = make_unit<velocity::meters_per_second_t>(
            chassis_speed_x_slope.calcRef(chassis_ref.vx(), chassis_ref_temp.vx()));
        chassis_ref.vy = make_unit<velocity::meters_per_second_t>(
            chassis_speed_y_slope.calcRef(chassis_ref.vy(), chassis_ref_temp.vy()));
        chassis_ref.w = make_unit<angular_velocity::radians_per_second_t>(
            chassis_speed_w_slope.calcRef(chassis_ref.w(), chassis_ref_temp.w()));
    }

    ControlMode chassis_control_mode_temp{};
    // 获取底盘控制模式
    if (os::control_mode_queue.ReceiveTo(&chassis_control_mode_temp, 0_ms)) {
        chassis_control_mode = chassis_control_mode_temp;
    }

    int8_t gyro_Directions_tmp = 1;
    // 获取小陀螺方向
    if (os::gyro_direction_queue.ReceiveTo(&gyro_Directions_tmp, 0_ms)) {
        Gyro_Directions = gyro_Directions_tmp;
    }
}

void GetAccelerationEstimate() {}

/**
 * @brief 获取电机初始化状态
 */
bool ifAllMotorEnable() {
    bool flag = true;
    for (auto const& i : device::dji_motors::wheel_motor) {
        flag &= i.GetState() == device::dji_motors::DJIMotorState::kEnable;
    }
    for (auto const& i : device::dji_motors::rudder_motor) {
        flag &= i.GetState() == device::dji_motors::DJIMotorState::kEnable;
    }
    return flag;
}

void GetMaxAllowedAcceleration() {
    if (chassis_acceleration_fdb[0] > 0_mps_sq and chassis_acceleration_fdb[1] > 0_mps_sq) {
        // 加速度在第一象限，则最大加速度由三号电机决定
        angle::degree_t const acc_angle = math::atan2(chassis_acceleration_fdb[1], chassis_acceleration_fdb[0]);

        angle::degree_t delta_angle = math::fabs(45.0_deg - acc_angle);
        delta_angle = make_unit<angle::degree_t>(os::math::normalize_angle(delta_angle(), 0.0F, 45.0F));
        length::meter_t const roll_radius = chassis_radius * os::math::cos(delta_angle) * acc_scale_offset;

        angle::degree_t const GravityAngle = os::math::atan2(roll_radius(), kGravityHeight());

        max_allowed_acceleration = kGravity * os::math::tan(GravityAngle);
    } else if (chassis_acceleration_fdb[0] < 0_mps_sq and chassis_acceleration_fdb[1] > 0_mps_sq) {
        // 加速度在第二象限，则最大加速度由四号电机决定
        angle::degree_t const acc_angle = math::atan2(chassis_acceleration_fdb[1], chassis_acceleration_fdb[0]);

        angle::degree_t delta_angle = math::fabs(135_deg - acc_angle);
        delta_angle = make_unit<angle::degree_t>(os::math::normalize_angle(delta_angle(), 0.0F, 45.0F));

        length::meter_t const roll_radius = chassis_radius * os::math::cos(delta_angle) * acc_scale_offset;

        angle::degree_t const GravityAngle = os::math::atan2(roll_radius(), kGravityHeight());

        max_allowed_acceleration = kGravity * os::math::tan(GravityAngle);
    } else if (chassis_acceleration_fdb[0] < 0_mps_sq and chassis_acceleration_fdb[1] < 0_mps_sq) {
        // 加速度在第三象限，则最大加速度由一号电机决定
        angle::degree_t const acc_angle = math::atan2(chassis_acceleration_fdb[1], chassis_acceleration_fdb[0]);

        angle::degree_t delta_angle = math::fabs(-135.0_deg - acc_angle);
        delta_angle = make_unit<angle::degree_t>(os::math::normalize_angle(delta_angle(), 0.0F, 45.0F));
        length::meter_t const roll_radius = chassis_radius * os::math::cos(delta_angle) * acc_scale_offset;

        angle::degree_t const GravityAngle = os::math::atan2(roll_radius(), kGravityHeight());

        max_allowed_acceleration = kGravity * os::math::tan(GravityAngle);
    } else if (chassis_acceleration_fdb[0] > 0_mps_sq and chassis_acceleration_fdb[1] < 0_mps_sq) {
        // 加速度在第四象限，则最大加速度由一号电机决定
        angle::degree_t const acc_angle = math::atan2(chassis_acceleration_fdb[1], chassis_acceleration_fdb[0]);

        angle::degree_t delta_angle = math::fabs(-45.0_deg - acc_angle);
        delta_angle = make_unit<angle::degree_t>(os::math::normalize_angle(delta_angle(), 0.0F, 45.0F));
        length::meter_t const roll_radius = chassis_radius * os::math::cos(delta_angle) * acc_scale_offset;

        angle::degree_t const GravityAngle = os::math::atan2(roll_radius(), kGravityHeight());

        max_allowed_acceleration = kGravity * os::math::tan(GravityAngle);
    } else {
        angle::degree_t const GravityAngle =
            os::math::atan2(chassis_radius() / sqrt2 * acc_scale_offset, kGravityHeight());
        max_allowed_acceleration = kGravity * os::math::tan(GravityAngle);
    }
}

ControlMode GetCtrlMode() { return chassis_control_mode; }

etl::atomic<velocity::meters_per_second_t> chassis_now_speed;
// inverse process（逆向过程）vs. forward process（正向过程）。
/**
 * @brief 底盘运动学正向解算
 * @note 从四个底盘电机获取当前底盘状态
 */
void KinematicsForwardCal() {
    chassis_fdb.vx =
        wheel_radius *
        (wheel_spd_fdb[0] * os::math::cos(rudder_angle[0]) + wheel_spd_fdb[1] * os::math::cos(rudder_angle[1]) +
         wheel_spd_fdb[2] * os::math::cos(rudder_angle[2]) + wheel_spd_fdb[3] * os::math::cos(rudder_angle[3])) /
        4.0_rad;
    chassis_fdb.vy =
        wheel_radius *
        (wheel_spd_fdb[0] * os::math::sin(rudder_angle[0]) + wheel_spd_fdb[1] * os::math::sin(rudder_angle[1]) +
         wheel_spd_fdb[2] * os::math::sin(rudder_angle[2]) + wheel_spd_fdb[3] * os::math::sin(rudder_angle[3])) /
        4.0_rad;
    chassis_fdb.w =
        (-wheel_spd_fdb[0] * os::math::cos(rudder_angle[0]) + wheel_spd_fdb[0] * os::math::sin(rudder_angle[0]) -
         wheel_spd_fdb[1] * os::math::cos(rudder_angle[1]) - wheel_spd_fdb[1] * os::math::sin(rudder_angle[1]) +
         wheel_spd_fdb[2] * os::math::cos(rudder_angle[2]) - wheel_spd_fdb[2] * os::math::sin(rudder_angle[2]) +
         wheel_spd_fdb[3] * os::math::cos(rudder_angle[3]) + wheel_spd_fdb[3] * os::math::sin(rudder_angle[3])) *
        sqrt2 / 2 * wheel_radius / chassis_radius / 4.0F;

    // 使用前向差分近似计算加速度
    static Chassis_Spd_t chassis_last_fdb{};  // 上次底盘状态
    static auto last_time = 0_us;             // 上次时间戳,用于计算加速度
    chassis_acceleration_fdb[0] = (chassis_fdb.vx - chassis_last_fdb.vx) / (os::GetDWTTime() - last_time);
    chassis_acceleration_fdb[1] = (chassis_fdb.vy - chassis_last_fdb.vy) / (os::GetDWTTime() - last_time);
    last_time = os::GetDWTTime();
    chassis_last_fdb = chassis_fdb;

    chassis_now_speed = math::sqrt(chassis_fdb.vy * chassis_fdb.vy + chassis_fdb.vx * chassis_fdb.vx);
}

/**
 * @brief 底盘运动学逆向解算
 * @note 从底盘状态到四个底盘电机的输出
 */
void KinematicsInverseCal() {
    // 旋转矩阵修正小陀螺位置
    if (chassis::logic::Chassis_FSM.getCurrentState() == "gyro") {
        auto const rotate_angle = make_unit<angle::radian_t>(-chassis_ref.w() * 0.03);
        chassis_ref.vx = chassis_ref.vx * os::math::cos(rotate_angle) - chassis_ref.vy * os::math::sin(rotate_angle);
        chassis_ref.vy = chassis_ref.vx * os::math::sin(rotate_angle) + chassis_ref.vy * os::math::cos(rotate_angle);
    }

    // 若使用速控底盘或力位混合控制，计算舵机角度
    rudder_angle_ref[0] = os::math::atan2(chassis_ref.vy() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius(),
                                          chassis_ref.vx() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius());
    rudder_angle_ref[1] = os::math::atan2(chassis_ref.vy() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius(),
                                          chassis_ref.vx() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius());
    rudder_angle_ref[2] = os::math::atan2(chassis_ref.vy() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius(),
                                          chassis_ref.vx() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius());
    rudder_angle_ref[3] = os::math::atan2(chassis_ref.vy() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius(),
                                          chassis_ref.vx() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius());

    // 计算速度期望，基于速度的ref

    // 使用向量叉乘计算（beta）

    wheel_spd_ref[0] = 1_rad_per_s *
                       os::math::sqrt((chassis_ref.vx() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vx() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) +
                                      (chassis_ref.vy() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vy() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius())) /
                       wheel_radius();
    wheel_spd_ref[1] = 1_rad_per_s *
                       os::math::sqrt((chassis_ref.vx() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vx() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) +
                                      (chassis_ref.vy() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vy() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius())) /
                       wheel_radius();

    // 2号和3号电机装反了，需要加入负号补偿
    wheel_spd_ref[2] = -1_rad_per_s *
                       os::math::sqrt((chassis_ref.vx() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vx() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) +
                                      (chassis_ref.vy() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vy() - sqrt2 / 2.0F * chassis_ref.w() * chassis_radius())) /
                       wheel_radius();
    wheel_spd_ref[3] = -1_rad_per_s *
                       os::math::sqrt((chassis_ref.vx() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vx() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) +
                                      (chassis_ref.vy() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius()) *
                                          (chassis_ref.vy() + sqrt2 / 2.0F * chassis_ref.w() * chassis_radius())) /
                       wheel_radius();
    // ----- 防止走不直
    // 获取最大转速轮子的索引和值
    // float fastest_wheel_spd = std::fabs(wheel_spd_ref[0]());
    // for (int i = 1; i < 4; i++) {
    //     if (float const abs_speed = std::fabs(wheel_spd_ref[i]()); abs_speed > fastest_wheel_spd) {
    //         fastest_wheel_spd = abs_speed;
    //     }
    // }

    // if (fastest_wheel_spd > max_wheel_spd()) {
    //     if (float const scale = (max_wheel_spd() / fastest_wheel_spd) * 1.25F; scale != 0) {
    //         for (angular_velocity::revolutions_per_minute_t& i : wheel_spd_ref) {
    //             i = make_unit<angular_velocity::revolutions_per_minute_t>(i() * scale);
    //         }
    //         // 将底盘速度也缩放
    //         chassis_ref.vx = chassis_ref.vx * scale;
    //         chassis_ref.vy = chassis_ref.vy * scale;
    //         chassis_ref.w = chassis_ref.w * scale;
    //     }
    // }
}

/**
 * @brief 动力学逆解算
 * @note 从力矩期望到轮电机输出和舵机角度
 * @note <dynamics> n.动态；动力学，力学；
 */
void DynamicsInverseCal() {
    // 利用底盘的力矩期望计算电机的力矩期望和舵电机的角度期望
    chassis_force_ref[0] = make_unit<force::newton_t>(chassis_speed_x_pid.getOutput()) + chassis_slope_compensation[0];
    chassis_force_ref[1] = make_unit<force::newton_t>(chassis_speed_y_pid.getOutput()) + chassis_slope_compensation[1];
    chassis_torque_ref = make_unit<torque::newton_meter_t>(chassis_speed_w_pid.getOutput());

    wheel_torque_ref[0] =
        wheel_radius * 1_N *
        os::math::sqrt(((chassis_force_ref[0] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[0] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) +
                        (chassis_force_ref[1] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[1] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius))
                           .value()) /
        4.0F;
    wheel_torque_ref[1] =
        wheel_radius * 1_N *
        os::math::sqrt(((chassis_force_ref[0] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[0] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) +
                        (chassis_force_ref[1] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[1] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius))
                           .value()) /
        4.0F;
    wheel_torque_ref[2] =
        wheel_radius * 1_N *
        os::math::sqrt(((chassis_force_ref[0] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[0] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) +
                        (chassis_force_ref[1] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[1] - sqrt2 / 2.0F * chassis_torque_ref / chassis_radius))
                           .value()) /
        4.0F;
    wheel_torque_ref[3] =
        wheel_radius * 1_N *
        os::math::sqrt(((chassis_force_ref[0] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[0] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) +
                        (chassis_force_ref[1] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius) *
                            (chassis_force_ref[1] + sqrt2 / 2.0F * chassis_torque_ref / chassis_radius))
                           .value()) /
        4.0F;

    // 若使用纯力矩底盘，解算舵的角度如下
    // rudder_angle_ref[0] = os::math::atan2(chassis_force_ref[1]() + sqrt2 / 2.0F * chassis_torque_ref() /
    // chassis_radius(),
    //                                       chassis_force_ref[0]() - sqrt2 / 2.0F * chassis_torque_ref() /
    //                                       chassis_radius());
    // rudder_angle_ref[1] = os::math::atan2(chassis_force_ref[1]() - sqrt2 / 2.0F * chassis_torque_ref() /
    // chassis_radius(),
    //                                       chassis_force_ref[0]() - sqrt2 / 2.0F * chassis_torque_ref() /
    //                                       chassis_radius());
    // rudder_angle_ref[2] = os::math::atan2(chassis_force_ref[1]() - sqrt2 / 2.0F * chassis_torque_ref() /
    // chassis_radius(),
    //                                       chassis_force_ref[0]() + sqrt2 / 2.0F * chassis_torque_ref() /
    //                                       chassis_radius());
    // rudder_angle_ref[3] = os::math::atan2(chassis_force_ref[1]() + sqrt2 / 2.0F * chassis_torque_ref() /
    // chassis_radius(),
    //                                       chassis_force_ref[0]() + sqrt2 / 2.0F * chassis_torque_ref() /
    //                                       chassis_radius());
}

/**
 * @brief 动力学正解算
 * @note 从轮电机输出和舵机角度到底盘力矩
 * @note <dynamics> n.动态；动力学，力学；
 */
void DynamicsForwardCal() {
    // 合成xy两个方向底盘收到的力
    chassis_force_fdb[0] = wheel_torque_fdb[0] / wheel_radius * os::math::cos(rudder_angle[0]) +
                           wheel_torque_fdb[0] / wheel_radius * os::math::cos(rudder_angle[0]) +
                           wheel_torque_fdb[1] / wheel_radius * os::math::cos(rudder_angle[1]) +
                           wheel_torque_fdb[2] / wheel_radius * os::math::cos(rudder_angle[2]) +
                           wheel_torque_fdb[3] / wheel_radius * os::math::cos(rudder_angle[3]);
    chassis_force_fdb[1] = wheel_torque_fdb[0] / wheel_radius * os::math::sin(rudder_angle[0]) +
                           wheel_torque_fdb[1] / wheel_radius * os::math::sin(rudder_angle[1]) +
                           wheel_torque_fdb[2] / wheel_radius * os::math::sin(rudder_angle[2]) +
                           wheel_torque_fdb[3] / wheel_radius * os::math::sin(rudder_angle[3]);
}

/**
 * @brief 根据底盘力矩估算底盘加速度
 */
void EstimateAcceleration() {
    chassis_acceleration_estimate[0] =
        chassis_force_fdb[0] / kVehicleMass - chassis_slope_compensation[0] / kVehicleMass;
    chassis_acceleration_estimate[1] = chassis_force_fdb[1] / kVehicleMass;
    // TODO: 结合IMU数据进行加速度和速度估算
}

/**
 * @brief 速度闭环控制
 * @note 就是字面意思啦，主要是三个闭环函数，分别是速度xy，角速度w
 * @note 闭环输出的物理意义是力和速度
 */
void SpeedCloseLoopCal() {
    // 首先是Vx速度环
    chassis_speed_x_pid.calculate(chassis_ref.vx(), chassis_fdb.vx());
    // 然后是Vy速度环
    chassis_speed_y_pid.calculate(chassis_ref.vy(), chassis_fdb.vy());
    // 最后是w速度环
    chassis_speed_w_pid.calculate(chassis_ref.w(), chassis_fdb.w());
}

// 四个电机的摩擦力模型
stribeck_friction torque_model[4]{stribeck_friction(), stribeck_friction(), stribeck_friction(), stribeck_friction()};
/**
 * @brief 力矩补偿前馈控制（零阶和一阶阻尼都要补偿，fuck）
 * @note 力矩补偿前馈ff
 */
void TorqueCompensationFeedForwardCal() {
    // for (int i = 0; i < 4; i++) {
    //     if (wheel_torque_ref[i] > 0_Nm) {
    //         wheel_torque_ref[i] += torque_fd_params[i];
    //     } else if (wheel_torque_ref[i] < 0_Nm) {
    //         wheel_torque_ref[i] -= torque_fd_params[i];
    //     } else {
    //         wheel_torque_ref[i] = 0_Nm;
    //     }
    // }

    // 使用斯特里贝克摩擦模型
    // 得到四个电机中摩擦力最小的一个，然后将其他电机补偿到这个电机
    // 1. 计算四个电机的摩擦力并找到最小值
    float friction[4]{};
    float min_friction = 1000000.0F;
    for (int i = 0; i < 4; i++) {
        friction[i] = torque_model[i].get_friction(device::dji_motors::wheel_motor[i].m_Encoder.GetSpeed() *
                                                   0.1047F);  // rpm-> radps
        if (fabs(friction[i]) < min_friction) {
            min_friction = fabs(friction[i]);
        }
    }
    // 2. 将摩擦力补偿到最小值

    // TODO: 构建二阶粘滞阻力和摩擦力矩模型（划去）
}

float _spd_max = 0.0F;
/**
 * @brief 底层输出计算
 */
void RefOutputCal() {
    for (int i = 0; i < 4; i++) {
        rudder_angle[i] =
            units::make_unit<angle::degree_t>(device::dji_motors::rudder_motor[i].m_Encoder.GetInvLimitedAngle());
    }
    // 舵电机角度和速度控制
    for (int i = 0; i < 4; i++) {
        // 计算角度误差，确保使用劣弧（最短路径）
        auto angle_error = units::make_unit<angle::degree_t>(
            os::math::normalize_angle(rudder_angle_ref[i].value() - rudder_angle[i].value(), -180.0f, 180.0f));

        // 判断是否需要反向控制（如果角度误差超过90度）
        if (std::fabs(angle_error.value()) > 90.0f) {
            // 修正期望角度（反向180度）
            angle_error = units::make_unit<angle::degree_t>(
                os::math::normalize_angle(angle_error.value() + 180.0f, -180.0f, 180.0f));

            // 反转轮电机力矩和速度方向
            wheel_torque_ref[i] = -wheel_torque_ref[i];
            wheel_spd_ref[i] = -wheel_spd_ref[i];
        }

        // 角度外环PID计算
        float const angle_output =
            rudder_angle_pid[i].calculate(0.0f, 0.0F - angle_error.value());  // ref为0相当于直接输入ERROR

        // 速度内环PID计算
        float speed_output =
            rudder_speed_pid[i].calculate(angle_output, device::dji_motors::rudder_motor[i].m_Encoder.GetInvSpeed());

        // 6. 输出到舵电机han轮电机
        device::dji_motors::rudder_motor[i].SetInvOutput(static_cast<int16_t>(speed_output));

        float wheel_output{};
        // 先计算电机的速度环PID
        if (chassis_control_mode == ControlMode::FORCE_CTRL) {
            wheel_output = wheel_speed_pid_forceCtrl[i].calculate(
                wheel_spd_ref[i](), device::dji_motors::wheel_motor[i].m_Encoder.GetShaftSpeed(),
                static_cast<int16_t>(wheel_torque_ref[i].value() / kTorqueCurrentConstant * kOutputCurrentConstant));
        } else {  // 速控底盘
            if (chassis::logic::Chassis_FSM.getCurrentState() == "gyro") {
                wheel_output = wheel_speed_pid_gyroCtrl[i].calculate(
                    wheel_spd_ref[i](), device::dji_motors::wheel_motor[i].m_Encoder.GetShaftSpeed(), 0);
            }else{
                wheel_speed_pid_gyroCtrl[i].reset();
                wheel_output = wheel_speed_pid_spdCtrl[i].calculate(
                wheel_spd_ref[i](), device::dji_motors::wheel_motor[i].m_Encoder.GetShaftSpeed(), 0);
            }
            
        }

        device::dji_motors::wheel_motor[i].SetOutput(static_cast<int16_t>(wheel_output));
    }
}

float TestSpeed{};
PIDController wheel_Test_speed_pid[4]{PIDController({75.0f, 25.0f, 0.0f, 10000.0f, 16000.0f, 10000.0f}),
                                      PIDController({75.0f, 25.0f, 0.0f, 10000.0f, 16000.0f, 10000.0f}),
                                      PIDController({75.0f, 25.0f, 0.0f, 10000.0f, 16000.0f, 10000.0f}),
                                      PIDController({75.0f, 25.0f, 0.0f, 10000.0F, 16000.0f, 10000.0f})};
uint8_t test_flag = 0;
float k_fdf[4] = {0.0473, 0.0501, 0.0907, 0.0715};
float b_fdf[4] = {-16.8778, -16.3741, -16.5214, -18.1099};
void SendOutput() {
    // if (test_flag == 1) {
    //     for (int i = 0; i < 4; i++) {
    //         wheel_Test_speed_pid[i].setReference(TestSpeed);
    //
    //         if (i == 2 or i == 3) {
    //             wheel_Test_speed_pid[i].setFeedback(-wheel_spd_fdb[i].value());
    //             wheel_Test_speed_pid[i].calculate();
    //             device::dji_motors::wheel_motor[i].SetOutput(static_cast<int16_t>(wheel_Test_speed_pid[i].getOutput())
    //             +
    //                                                      k_fdf[i] * wheel_spd_fdb[i]() + b_fdf[i]);
    //         }else {
    //             wheel_Test_speed_pid[i].setFeedback(wheel_spd_fdb[i].value());
    //             wheel_Test_speed_pid[i].calculate();
    //             device::dji_motors::wheel_motor[i].SetOutput(static_cast<int16_t>(wheel_Test_speed_pid[i].getOutput())
    //             +
    //                                                      k_fdf[i] * wheel_spd_fdb[i]() + b_fdf[i]);
    //         }
    //
    //     }
    // } else if (test_flag == 0) {
    //     if (wheel_spd_fdb[0]() <= 0.1) {
    //         for (auto & i : device::dji_motors::wheel_motor) {
    //             i.SetOutput(0);
    //         }
    //     }
    //     for (int i = 0; i < 4; i++) {
    //         wheel_Test_speed_pid[i].reset();
    //         if (i == 2 or i == 3) {
    //             device::dji_motors::wheel_motor[i].SetOutput(static_cast<int16_t>(0 -
    //                                                       k_fdf[i] * wheel_spd_fdb[i]() - b_fdf[i]));
    //         }else {
    //             device::dji_motors::wheel_motor[i].SetOutput(static_cast<int16_t>(0 +
    //                                                      k_fdf[i] * wheel_spd_fdb[i]() + b_fdf[i]));
    //         }
    //
    //     }
    // }

    device::dji_motors::wheel_group.TransmitData();
    device::dji_motors::rudder_group.TransmitData();
}

void SlopeForceFeedforward() {
    // TODO: 斜坡力前馈
    // 假设已经获取了IMU数据（pitch&Roll）
    units::angle::degree_t const pitch = 0_deg;  // 假设获取的IMU数据
    units::angle::degree_t const roll = 0_deg;   // 假设获取的IMU数据
    // 计算斜坡力矩补偿
    chassis_slope_compensation[0] = kVehicleMass * kGravity * os::math::sin(pitch);
    chassis_slope_compensation[1] = kVehicleMass * kGravity * os::math::sin(roll);
    // TODO: 按照斜坡上重心位置标定前馈
}

// Follow和GYRO建议放到状态机里
/**
 * @brief 随动模式
 * @param ref 底盘速度期望
 * @return 随动处理后的底盘速度期望
 */
float speed_fdf = 0.0F;
float speed_error = 0.0F;
float last_ref = 0.0F;
float Kf = 0.0f;
os::filter::LowPassFilter fdf_lowpass_filter(15.9154922f);
Chassis_Spd_t Follow(Chassis_Spd_t const& ref) {
    // 计算当前yaw角度与0，90，180，-90(270)四个方向之间的最小值（注意归一化）作为error传给PID控制器
    angle::degree_t error{};
    Chassis_Spd_t cal_chassis_ref{};
    angle::degree_t const now_angle =
        make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle());
    auto _dead_zone = follow_dead_zone();
    if (board_com::gimbal2chassis_pkg::Instance().GetData().fold_flag == board_com::FoldState::Unfold) {  // 未折叠状态
        if (now_angle <= 45.0_deg and now_angle > -45.0_deg) {
            error = 0_deg - now_angle;
        } else if (now_angle <= 135.0_deg and now_angle > 45.0_deg) {
            error = 90_deg - now_angle;
        } else if (now_angle <= -135.0_deg and now_angle > -45.0_deg) {
            error = -90_deg - now_angle;
        } else {
            error = 180_deg - now_angle;
        }
        error = make_unit<angle::degree_t>(os::math::normalize_angle(error(), -45.0F, 45.0F));  // 归一化到[-45,45]之间
    } else if (board_com::gimbal2chassis_pkg::Instance().GetData().fold_flag ==
               board_com::FoldState::Fold) {  // 折叠状态
        if (now_angle <= 180.0_deg and now_angle > 0.0_deg) {
            error = 90_deg - now_angle;
        } else {
            error = -90_deg - now_angle;
        }
        _dead_zone = 0.1F;
        error = make_unit<angle::degree_t>(os::math::normalize_angle(error(), -90.0F, 90.0F));  // 标准到[-90,90]之间
    }

    // 添加死区
    if (std::fabs(error()) < _dead_zone) {
        // error = 0.0_deg;
        cal_chassis_ref.w = make_unit<angular_velocity::radians_per_second_t>(0.0f);
        // 如果在死区内，PID控制器不工作
        follow_angle_pid.reset();
        follow_spd_pid.reset();

    } else {
        // 修改error，让其闭环到死区边界而不是0
        if (error > 0.0_deg) {
            error = make_unit<angle::degree_t>(error() - _dead_zone);
        } else {
            error = make_unit<angle::degree_t>(error() + _dead_zone);
        }

        follow_angle_pid.calculate(0, error());
        follow_spd_pid.setFeedback(device::dji_motors::GimbalMotor.m_Encoder.GetSpeed() * 0.105F);  // rpm -> rad/s
        follow_spd_pid.setReference(follow_angle_pid.getOutput());
        speed_fdf = fdf_lowpass_filter.calculate(Kf * (follow_angle_pid.getOutput() - last_ref));
        last_ref = follow_angle_pid.getOutput();
        follow_spd_pid.calculate();
        cal_chassis_ref.w = make_unit<angular_velocity::radians_per_second_t>(follow_spd_pid.getOutput() + speed_fdf);
    }

    // 下面计算vx 和 vy 的期望
    cal_chassis_ref.vx =
        ref.vx *
            os::math::cos(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle())) +
        ref.vy * os::math::sin(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle()));
    cal_chassis_ref.vy =
        ref.vx *
            os::math::sin(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle())) -
        ref.vy * os::math::cos(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle()));

    return cal_chassis_ref;
}

/**
 * @brief 当GYRO不处于maxout和customize模式时的底盘旋转速度计算
 * @param power_limit 功率限制
 * @return 底盘旋转速度
 */
auto calculateChassisRotationSpeed(float const power_limit) {
    // 功率范围：55W-120W
    // 速度范围：6.0-11.0 rad/s
    float constexpr min_power = 55.0f;
    float constexpr max_power = 120.0f;
    float constexpr min_speed = 7.0f;
    float constexpr max_speed = 11.5f;

    // 限制功率范围
    float limited_power{};
    if (power_limit < min_power) {
        limited_power = min_power;
    } else if (power_limit > max_power) {
        limited_power = max_power;
    } else {
        limited_power = power_limit;
    }

    // 线性映射公式
    float rotation_speed = min_speed + (limited_power - min_power) * (max_speed - min_speed) / (max_power - min_power);
    return make_unit<angular_velocity::radians_per_second_t>(rotation_speed);
}

/**
 * @brief
 * @param ref 底盘速度期望
 * @return 小陀螺速度处理后的底盘速度期望
 */
Chassis_Spd_t GYRO(Chassis_Spd_t const& ref) {
    Chassis_Spd_t cal_chassis_ref{};

    auto const class_w_ref = calculateChassisRotationSpeed(referee_data.robotPerf_.PowerLimit);
    if (chassis_power::logic::PowerCtrl_FSM.getCurrentState() == "max_out") {
        cal_chassis_ref.w = Upper_Gyro_Speed * Gyro_Directions;
    } else if (chassis_power::logic::PowerCtrl_FSM.getCurrentState() == "customized") {
        cal_chassis_ref.w = MAX(class_w_ref * Gyro_Shift_scale, Upper_Gyro_Speed) * Gyro_Directions;
    } else {
        cal_chassis_ref.w = class_w_ref * Gyro_Directions;
    }

    constexpr float gyro_reduce_scale = 0.35F;  // 小陀螺时候降低速度可以转的快一点

    // 先对ref进行偏置处理，得到新的ref
    Chassis_Spd_t new_ref = ref;
    constexpr float bias_scale = 0.2F;  // 偏置系数，可调整

    // // 计算ref的平动总速度
    // float ref_speed_magnitude = os::math::sqrt(ref.vx() * ref.vx() + ref.vy() * ref.vy());

    // if (cal_chassis_ref.w() != 0 && ref_speed_magnitude > 0.01f) {  // 避免除零
    //     // 计算ref当前方向角度
    //     auto ref_angle = os::math::atan2(ref.vy(), ref.vx());

    //     // 根据w的方向确定偏置方向
    //     float bias_angle;
    //     if (cal_chassis_ref.w() > 0) {
    //         // w>0，逆时针旋转90度
    //         bias_angle = ref_angle + PI/2;
    //     } else {
    //         // w<0，顺时针旋转90度
    //         bias_angle = ref_angle - PI/2;
    //     }

    //     // 计算偏置速度，与ref平动总速度成比例
    //     float bias_magnitude = bias_scale * ref_speed_magnitude;
    //     velocity::meters_per_second_t bias_vx = make_unit<velocity::meters_per_second_t>(bias_magnitude *
    //     os::math::cos(bias_angle)); velocity::meters_per_second_t bias_vy =
    //     make_unit<velocity::meters_per_second_t>(bias_magnitude * os::math::sin(bias_angle));

    //     // 添加偏置到ref，得到新的ref
    //     new_ref.vx = ref.vx + bias_vx;
    //     new_ref.vy = ref.vy + bias_vy;
    // }

    // 下面用新的ref计算vx 和 vy 的期望
    cal_chassis_ref.vx =
        new_ref.vx * gyro_reduce_scale *
            os::math::cos(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle())) +
        new_ref.vy * gyro_reduce_scale *
            os::math::sin(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle()));
    cal_chassis_ref.vy =
        new_ref.vx * gyro_reduce_scale *
            os::math::sin(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle())) -
        new_ref.vy * gyro_reduce_scale *
            os::math::cos(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle()));

    return cal_chassis_ref;
}

/**
 * @brief “normal”模式 无随动，最纯粹的底盘模式
 * @param ref 底盘速度期望
 * @return 解算后的底盘速度期望
 */
Chassis_Spd_t Normal(Chassis_Spd_t const& ref) {
    Chassis_Spd_t cal_chassis_ref{};

    // 下面计算vx 和 vy 的期望
    cal_chassis_ref.vx =
        ref.vx *
            os::math::cos(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle())) +
        ref.vy * os::math::sin(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle()));
    cal_chassis_ref.vy =
        ref.vx *
            os::math::sin(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle())) -
        ref.vy * os::math::cos(make_unit<angle::degree_t>(device::dji_motors::GimbalMotor.m_Encoder.GetLimitedAngle()));

    cal_chassis_ref.w = 0_rad_per_s;
    return cal_chassis_ref;
}

Chassis_Spd_t STOP() {
    Chassis_Spd_t cal_chassis_ref{};
    cal_chassis_ref.vx = 0_mps;
    cal_chassis_ref.vy = 0_mps;
    cal_chassis_ref.w = 0_rad_per_s;
    return cal_chassis_ref;
}

void IfLockDoLock() {
    static time::microsecond_t lock_judge_timer = 0_ms;  // 等待锁定时间
    if (chassis_ref.vx == 0_mps and chassis_ref.vy == 0_mps and chassis_ref.w == 0_rad_per_s) {
        rudder_angle_ref[0] = 135_deg;
        rudder_angle_ref[1] = 45_deg;
        rudder_angle_ref[2] = 135_deg;
        rudder_angle_ref[3] = 45_deg;
        if (os::GetTime() - lock_judge_timer > lock_time) {
            // 轮电机速度期望设置为0
            for (int i = 0; i < 4; i++) {
                wheel_spd_ref[i] = 0_rad_per_s;
                wheel_torque_ref[i] = 0_Nm;
            }
            // 舵机角度期望设置为45和-45
            rudder_angle_ref[0] = 45_deg;
            rudder_angle_ref[1] = -45_deg;
            rudder_angle_ref[2] = 45_deg;
            rudder_angle_ref[3] = -45_deg;
        }
    } else {
        lock_judge_timer = os::GetTime();
    }
}

/**
 * @brief 翻滚阻抗控制青春增强版
 * @note 使用IMU数据计算当前加速度，并根据加速度计算滑移率，然后根据滑移率计算加速度期望
 */
void RollOverResistanceLite() {
    auto now_speed = math::sqrt(chassis_ref.vx * chassis_ref.vx + chassis_ref.vy * chassis_ref.vy);
    // if (now_speed < 2_mps) {  // 低速时，不进行翻滚阻抗控制
    //     return;
    // }
    // -------------用于计算平动加速度---------------
    static time::microsecond_t last_time{};
    static velocity::meters_per_second_t last_chassis_ref[2]{};
    // 获取当前时间
    time::microsecond_t const delta_time = os::GetDWTTime() - last_time;
    last_time = os::GetDWTTime();
    // 获取当前加速度期望
    acceleration::meters_per_second_squared_t const ref_acc_vx = (chassis_ref.vx - last_chassis_ref[0]) / (delta_time);
    acceleration::meters_per_second_squared_t const ref_acc_vy = (chassis_ref.vy - last_chassis_ref[1]) / (delta_time);
    acceleration::meters_per_second_squared_t const ref_acc_sum =
        math::sqrt(ref_acc_vx * ref_acc_vx + ref_acc_vy * ref_acc_vy);
    // 限制当前加速度
    auto temp_max_acceleration = max_allowed_acceleration;
    if (now_speed < 1.5_mps) {
        temp_max_acceleration *= 1.5F;  // 低速时候允许更高的加速度
    }
    if (ref_acc_sum > temp_max_acceleration) {
        float const scale = temp_max_acceleration() / ref_acc_sum();
        chassis_ref.vx = last_chassis_ref[0] + ref_acc_vx * delta_time * scale;
        chassis_ref.vy = last_chassis_ref[1] + ref_acc_vy * delta_time * scale;
    }
    last_chassis_ref[0] = chassis_ref.vx;
    last_chassis_ref[1] = chassis_ref.vy;

    // -------------用于计算转弯产生的向心加速度---------------
    // 暂时没有用到的数据，先注释了
    // static length::meter_t TurningCurvatureRadius{};
    // static acceleration::meters_per_second_squared_t CentripetalAcceleration{};
    // // 计算曲率半径
    // TurningCurvatureRadius =
    //     math::sqrt(chassis_ref.vx * chassis_ref.vx + chassis_ref.vy * chassis_ref.vy) / math::fabs(chassis_ref.w);
    // // 计算向心加速度
    // CentripetalAcceleration = TurningCurvatureRadius * chassis_ref.w * chassis_ref.w;

    // 计算最大允许的omega（绝对值）
    angular_velocity::radians_per_second_t const max_allowed_omega = make_unit<angular_velocity::radians_per_second_t>(
        math::sqrt(max_allowed_acceleration)() /
        math::sqrt(chassis_ref.vx * chassis_ref.vx + chassis_ref.vy * chassis_ref.vy)());
    if (math::fabs(chassis_ref.w) > max_allowed_omega) {
        // 计算缩放比例
        float const scale = max_allowed_omega() / math::fabs(chassis_ref.w)();
        // 计算缩放后的角速度期望
        chassis_ref.w = make_unit<angular_velocity::radians_per_second_t>(chassis_ref.w() * scale);
    }

    // TODO: 完整版翻滚阻抗控制，需要使用IMU数据，使用惯性导航器件估计当前加速度计算滑移率（试试）
}

}  // namespace chassis
namespace powerCtrl {
using namespace os::power;
PowerCtrl chassis_PowerCtrl;                                // 实例化功率控制器
float chassis_power[4]{};                                   // 功率计反馈值
constexpr uint16_t buffer_length = sizeof(chassis_power);   // 功率计数据长度
inline float rls_data[2] = {0.035045117F, 0.00114030263F};  // RLS初始矩阵参数

float power_target = 0.0F;  // 功率目标值
float wheel_current[4]{};   // 电机电流反馈值
float current_now[4]{};     // 电机电流反馈值
float wheel_speed[4]{};     // 电机速度反馈值

uint16_t len_test = 0;
InitLate(init_power_ctrl) {
    chassis_PowerCtrl.Get_rls().setParamVector(Matrixf<2, 1>(rls_data));
    // 设置功率计的回调函数
    os::Uart::Instance(USART1).on_receive.connect([&](uint8_t const* buff, uint16_t const len) {
        len_test = len;
        if (len == 8) {
            memcpy(&chassis_power, buff, len);
        }
    });
}

}  // namespace powerCtrl

// 原子定义一个功率求和变量
etl::atomic<float> sum_power_chassis = 0.0F;
etl::atomic<float> power_target_ref = 0.0F;
namespace powerCtrl {
using namespace os::power;

void PowerCtrl() {
    // 接收功率目标值
    os::power_target_queue.ReceiveTo(&power_target, 0_ms);
    // 计算功率目标值
    for (short i = 0; i < 4; ++i) {
        wheel_current[i] = static_cast<float>(device::dji_motors::wheel_motor[i].GetOutput()) * 20.0F / 16384.0F;
        wheel_speed[i] =
            device::dji_motors::wheel_motor[i].m_Encoder.GetShaftSpeed() * 2 * PI / 60.0F;  // rpm转换成弧度每秒
        current_now[i] = device::dji_motors::wheel_motor[i].m_Encoder.GetCurrent();
    }
    float rudder_power = chassis_power[1] * 0.4F;  // 一个小小的系数作为小小的偏置控制小小的量
    if (rudder_power > 20.0F) {
        rudder_power = 20.0F;
    }

    // UI绘制数据赋值（原子操作）
    power_target_ref = power_target;
    sum_power_chassis = chassis_power[1] + chassis_power[0];

    // 似乎底盘有点限制的太狠了，可以加一个系数修正一下
    chassis_PowerCtrl.calculator(1.35F * (power_target - rudder_power), wheel_current, wheel_speed, chassis_power[0],
                                 current_now);
    // 分配功率
    for (short i = 0; i < 4; ++i) {
        device::dji_motors::wheel_motor[i].SetOutput(static_cast<int16_t>(wheel_current[i] * 16384.0F / 20.0F));
    }
}

}  // namespace powerCtrl

CREATE_THREAD_STATIC(chassis_ctrl_test_thread, 1024, NULL, 5) {
    using namespace chassis;
    for (;;) {
        Get_chassis_ref();            // 获取底盘期望值
        GetAllMotorFdb();             // 获取底盘电机反馈
        KinematicsForwardCal();       // 运动学正解算
        GetMaxAllowedAcceleration();  // 获取最大允许加速度
        RollOverResistanceLite();     // 翻滚阻抗控制青春版
        KinematicsInverseCal();       // 运动学逆解算
        if (chassis_control_mode == ControlMode::FORCE_CTRL) {
            SpeedCloseLoopCal();   // 底盘整体速度闭环
            DynamicsInverseCal();  // 动力学逆解算
        }
        // TorqueCompensationFeedForwardCal();  // 力矩补偿前馈
        IfLockDoLock();          // 判断并执行是否锁定底盘
        RefOutputCal();          // 输出量计算
        powerCtrl::PowerCtrl();  // 功率控制
        SendOutput();            // 输出
        // os::Sleep(1_ms);
        osDelay(1);
    }
}