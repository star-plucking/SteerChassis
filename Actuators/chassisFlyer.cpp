/**
 * @file           : chassisFlyer.cpp
 * @author         : Star_Plucking
 * @brief          : 反正也不让我调这车我自己写
 * @date           : 25-5-19 下午1:52
 * @lastEditor     :
 * @lastEditTime   : 25-5-19 下午1:52
 */

#include <clock.h>
#include <units.h>

namespace chassis {
using namespace units::literals;
class Flyer {
   private:
    static constexpr float inertia_scale = 0.2F;  // 上半部分对下半部分惯量之比
    units::velocity::meters_per_second_t m_vx{};  // 底盘速度Vx
    units::velocity::meters_per_second_t m_vy{};  // 底盘速度Vy

    units::acceleration::meters_per_second_squared_t m_az{};  // Z方向加速度

    units::angle::degree_t m_pitch{};               // 底盘角度
    units::angle::degree_t m_Target_Motor_pitch{};  // 电机目标角度
    units::angle::degree_t m_Now_Motor_pitch{};     // 电机当前角度

    bool m_Is_Soar{};  // 是否腾空

    units::time::second_t m_last_update_time{};       // 上一次更新时间
    units::time::second_t m_soar_detection_time{};    // 检测到腾空的时间
    units::time::second_t m_last_calculation_time{};  // 上次计算目标角度的时间
    bool m_is_calculation_locked{};                   // 是否锁定计算
    static constexpr auto DELAY_AFTER_SOAR = 200_ms;  // 腾空后延迟时间
    static constexpr auto LOCK_DURATION = 500_ms;     // 锁死持续时间

   public:
    void UpdateSpeed(units::velocity::meters_per_second_t const vx, units::velocity::meters_per_second_t const vy) {
        m_vx = vx;
        m_vy = vy;
    }

    void UpdatePitch(units::angle::degree_t const pitch) { m_pitch = pitch; }

    void UpdateAz(units::acceleration::meters_per_second_squared_t const az) { m_az = az; }

    void UpdateNowPitch(units::angle::degree_t const now_pitch) { m_Now_Motor_pitch = now_pitch; }

    /**
     * @brief 测试底盘是否腾空
     * @return true 腾空
     * @return false 未腾空
     */
    bool TestChassisSoar() {
        if (m_az <= 2_mps_sq) {  // 腾空时会失重
            if (!m_Is_Soar) {    // 首次检测到腾空
                m_soar_detection_time = os::GetTime();
            }
            m_Is_Soar = true;
            return true;
        }
        m_Is_Soar = false;
        return false;
    }

    /**
     * @brief 计算目标pitch角度
     *
     */
    void CalculateTargetPitch() {
        auto current_time = os::GetTime();

        // 检查是否在锁死期间
        if (m_is_calculation_locked) {
            if (current_time - m_last_calculation_time >= LOCK_DURATION) {
                m_is_calculation_locked = false;
            } else {
                return;  // 在锁死期间，不更新目标角度
            }
        }

        // 检查是否满足延迟条件
        if (m_Is_Soar && current_time - m_soar_detection_time >= DELAY_AFTER_SOAR) {
            m_Target_Motor_pitch = m_Now_Motor_pitch + m_pitch / inertia_scale;
            m_last_calculation_time = current_time;
            m_is_calculation_locked = true;  // 设置锁死状态
        } else if (!m_Is_Soar) {
            m_Target_Motor_pitch = m_Now_Motor_pitch;
        }
    }

    [[nodiscard]] units::angle::degree_t GetTargetPitch() const { return m_Target_Motor_pitch; }
};

}  // namespace chassis