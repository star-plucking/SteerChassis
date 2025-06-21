/**
 * @file           : chassisEstimator.cpp
 * @author         : Star_Plucking
 * @brief          : 底盘状态估计器
 * @date           : 25-5-3 下午9:30
 * @lastEditor     :
 * @lastEditTime   : 25-5-3 下午9:30
 */

#pragma once

#include <algorithm/ekf.hpp>
#include <algorithm/matrix.h>
#include <units.h>

namespace chassis {

/**
 * @brief 底盘速度估计器
 *
 * 使用扩展卡尔曼滤波器结合底盘反馈速度和IMU加速度计数据
 * 特性：
 * 1. 低速状态下更信任底盘里程计速度
 * 2. 高速状态下增加IMU加速度计信息的权重
 * 3. 自适应调整过程噪声和测量噪声
 */
class ChassisVelocityEstimator {
   public:
    // 状态变量: [vx, vy, ax, ay]
    using StateVector = os::math::Matrix<4, 1>;
    // 测量变量: [vx_chassis, vy_chassis, ax_imu, ay_imu]
    using MeasurementVector = os::math::Matrix<4, 1>;

    ChassisVelocityEstimator()
        : ekf_(
              // 状态转移函数 f(x, dt)
              [this](StateVector const& x, float const& dt) -> StateVector {
                  // 速度 = 上一时刻速度 + 加速度*dt
                  float x_pre_param[] = {x(0, 0) + x(2, 0) * dt,  // vx = vx + ax*dt
                                         x(1, 0) + x(3, 0) * dt,  // vy = vy + ay*dt
                                         x(2, 0),                 // ax = ax (假设加速度不变)
                                         x(3, 0)};                // ay = ay
                  auto x_pred = os::math::Matrix<4, 1>(x_pre_param);
                  return x_pred;
              },

              // 测量函数 h(x)
              [this](StateVector const& x) -> MeasurementVector {
                  float z_pred_param[] = {x(0, 0),   // vx
                                          x(1, 0),   // vy
                                          x(2, 0),   // ax
                                          x(3, 0)};  // ay
                  auto z_pred = os::math::Matrix<4, 1>(z_pred_param);
                  return z_pred;
              },

              // 状态转移雅可比矩阵 F
              [this](StateVector const& x, float const& dt) -> os::kalman::ExtensionKalmanFilter<4, 4>::JacobianF {
                  float F_param[] = {1.0f, 0.0f, dt,   0.0f,   // 第一行
                                     0.0f, 1.0f, 0.0f, dt,     // 第二行
                                     0.0f, 0.0f, 1.0f, 0.0f,   // 第三行
                                     0.0f, 0.0f, 0.0f, 1.0f};  // 第四行
                  auto F = os::math::Matrix<4, 4>(F_param);
                  return F;
              },

              // 测量雅可比矩阵 H
              [this](StateVector const& x) -> os::kalman::ExtensionKalmanFilter<4, 4>::JacobianH {
                  // H_param
                  float H_param[] = {1.0f, 0.0f, 0.0f, 0.0f,   // 第一行
                                     0.0f, 1.0f, 0.0f, 0.0f,   // 第二行
                                     0.0f, 0.0f, 1.0f, 0.0f,   // 第三行
                                     0.0f, 0.0f, 0.0f, 1.0f};  // 第四行
                  auto H = os::math::Matrix<4, 4>(H_param);
                  return H;
              }) {
        // initialize();
    }

    /**
     * @brief 初始化滤波器
     */
    void initialize() {
        // 初始状态
        constexpr StateVector x0{};

        // 初始协方差
        auto P0 = os::math::getIdentityMatrix<4>();
        P0 = P0 * 0.1f;

        // 初始化滤波器
        ekf_.initialize(x0, P0);

        // 设置过程噪声
        // 使用数组初始化
        float Q_param[] = {0.01f, 0.0F,  0.0F, 0.0F,  // vx, vy, ax, ay}
                           0.0F,  0.01f, 0.0F, 0.0F,  //
                           0.0F,  0.0F,  0.1f, 0.0F,  //
                           0.0F,  0.0F,  0.0F, 0.1f};
        auto Q = os::math::Matrix<4, 4>(Q_param);
        ekf_.setProcessNoise(Q);

        // 设置测量噪声
        // 使用数组初始化
        float R_param[] = {0.1f, 0.0F, 0.0F, 0.0F,   // vx, vy, ax, ay}
                           0.0F, 0.1f, 0.0F, 0.0F,   //
                           0.0F, 0.0F, 0.5f, 0.0F,   //
                           0.0F, 0.0F, 0.0F, 0.5f};  //
        auto R = os::math::Matrix<4, 4>(R_param);
        ekf_.setMeasurementNoise(R);

        is_initialized_ = true;
    }

    /**
     * @brief 更新状态估计
     *
     * @param vx_chassis 底盘里程计x方向速度
     * @param vy_chassis 底盘里程计y方向速度
     * @param ax_imu IMU x方向加速度
     * @param ay_imu IMU y方向加速度
     * @param dt 时间增量
     */
    void update(units::velocity::meters_per_second_t vx_chassis, units::velocity::meters_per_second_t vy_chassis,
                units::acceleration::meters_per_second_squared_t ax_imu,
                units::acceleration::meters_per_second_squared_t ay_imu, float dt) {
        // 自适应调整噪声矩阵
        adjustNoiseMatrices(vx_chassis, vy_chassis);

        // 预测步骤
        ekf_.predict(dt);

        // 构建测量向量

        float z_param[] = {vx_chassis(),  // vx_chassis
                           vy_chassis(),  // vy_chassis
                           ax_imu(),      // ax_imu
                           ay_imu()};     // ay_imu
        MeasurementVector z = os::math::Matrix<4, 1>(z_param);

        // 更新步骤
        ekf_.update(z);

        // 更新上次速度
        last_velocity_magnitude_ = std::sqrt(vx_chassis() * vx_chassis() + vy_chassis() * vy_chassis());
    }

    /**
     * @brief 获取估计的x方向速度
     */
    [[nodiscard]] units::velocity::meters_per_second_t getVelocityX() const {
        return units::make_unit<units::velocity::meters_per_second_t>(ekf_.getState()(0, 0));
    }

    /**
     * @brief 获取估计的y方向速度
     */
    [[nodiscard]] units::velocity::meters_per_second_t getVelocityY() const {
        return units::make_unit<units::velocity::meters_per_second_t>(ekf_.getState()(1, 0));
    }

    /**
     * @brief 获取估计的x方向加速度
     */
    [[nodiscard]] units::acceleration::meters_per_second_squared_t getAccelerationX() const {
        return units::make_unit<units::acceleration::meters_per_second_squared_t>(ekf_.getState()(2, 0));
    }

    /**
     * @brief 获取估计的y方向加速度
     */
    [[nodiscard]] units::acceleration::meters_per_second_squared_t getAccelerationY() const {
        return units::make_unit<units::acceleration::meters_per_second_squared_t>(ekf_.getState()(3, 0));
    }

    /**
     * @brief 获取估计的速度大小
     */
    [[nodiscard]] units::velocity::meters_per_second_t getVelocityMagnitude() const {
        auto vx = ekf_.getState()(0, 0);
        auto vy = ekf_.getState()(1, 0);
        return units::make_unit<units::velocity::meters_per_second_t>(std::sqrt(vx * vx + vy * vy));
    }

   private:
    /**
     * @brief 自适应调整噪声矩阵
     *
     * 根据速度大小动态调整过程噪声和测量噪声
     * 低速时更信任底盘反馈，高速时更信任IMU加速度积分
     */
    void adjustNoiseMatrices(units::velocity::meters_per_second_t vx, units::velocity::meters_per_second_t vy) {
        // 计算速度大小
        float const velocity_magnitude = std::sqrt(vx() * vx() + vy() * vy());

        // 速度阈值，小于此值认为是低速
        constexpr float LOW_SPEED_THRESHOLD = 0.2f;  // m/s
        // 速度阈值，大于此值认为是高速
        constexpr float HIGH_SPEED_THRESHOLD = 1.0f;  // m/s

        // 自适应调整测量噪声

        float R_param[] = {0.1f, 0.0F, 0.0F, 0.0F,   // 底盘vx测量噪声
                           0.0F, 0.1f, 0.0F, 0.0F,   // 底盘vy测量噪声
                           0.0F, 0.0F, 1.0f, 0.0F,   // IMU ax测量噪声
                           0.0F, 0.0F, 0.0F, 1.0f};  // IMU ay测量噪声

        // 低速时，更信任底盘反馈，减小底盘速度测量噪声
        if (velocity_magnitude < LOW_SPEED_THRESHOLD) {
            // 低速情况
            R_param[0*5] = 0.05f;  // 底盘vx测量噪声小
            R_param[1*5] = 0.05f;  // 底盘vy测量噪声小
            R_param[2*5] = 1.0f;   // IMU ax测量噪声大
            R_param[3*5] = 1.0f;   // IMU ay测量噪声大
        }
        // 高速时，增加IMU加速度计的权重
        else if (velocity_magnitude > HIGH_SPEED_THRESHOLD) {
            // 高速情况
            R_param[0*5] = 1.0f;   // 底盘vx测量噪声大
            R_param[1*5] = 1.0f;   // 底盘vy测量噪声大
            R_param[2*5] = 0.05f;  // IMU ax测量噪声小
            R_param[3*5] = 0.05f;  // IMU ay测量噪声小
        }
        // 中速时，平衡两者权重
        else {
            // 线性插值中间状态
            float factor = (velocity_magnitude - LOW_SPEED_THRESHOLD) / (HIGH_SPEED_THRESHOLD - LOW_SPEED_THRESHOLD);

            R_param[0*5] = 0.05f + factor * 0.25f;  // 线性增加底盘vx测量噪声
            R_param[1*5] = 0.05f + factor * 0.25f;  // 线性增加底盘vy测量噪声
            R_param[2*5] = 1.0f - factor * 0.8f;    // 线性减小IMU ax测量噪声
            R_param[3*5] = 1.0f - factor * 0.8f;    // 线性减小IMU ay测量噪声
        }
        auto R = os::math::Matrix<4, 4>(R_param);

        // 加速时增加过程噪声
        float acceleration_factor = std::abs(velocity_magnitude - last_velocity_magnitude_);
        float Q_param[] = {0.01f + acceleration_factor * 0.1f, 0.0F, 0.0F, 0.0F, 0.0F,
                           0.01f + acceleration_factor * 0.1f, 0.0F, 0.0F, 0.0F, 0.0F,
                           0.1f + acceleration_factor * 0.5f,  0.0F, 0.0F, 0.0F, 0.0F,
                           0.1f + acceleration_factor * 0.5f};
        auto Q = os::math::Matrix<4, 4>(Q_param);

        // 更新滤波器噪声矩阵
        ekf_.setMeasurementNoise(R);
        ekf_.setProcessNoise(Q);
    }

   private:
    os::kalman::ExtensionKalmanFilter<4, 4> ekf_;
    bool is_initialized_ = false;
    float last_velocity_magnitude_ = 0.0f;
};

}  // namespace chassis