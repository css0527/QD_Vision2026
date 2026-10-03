#ifndef RM_SERIAL_DRIVER_PROTOCOL_GIMBAL_COMMAND_SMOOTHER_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_GIMBAL_COMMAND_SMOOTHER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace qd::serial_driver::protocol {

/**
 * @brief 从实测姿态起步，对绝对角指令进行低通和角速度限制。
 * @note 本类不负责角度标定或机械限位；调用方须保证输入 pitch 已限位。
 *       所有时间均使用同一单调时钟，跨线程访问须由调用方加锁。
 */
class GimbalCommandSmoother {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    /** @brief 串口绝对角的平滑参数；角速度单位为度每秒。 */
    struct Config {
        bool enabled = false;
        double time_constant_s = 0.10;
        double pitch_rate_deg_s = 20.0;
        double yaw_rate_deg_s = 60.0;
        double feedback_timeout_s = 0.30;
        double pitch_deadband_deg = 0.0; // 相对上一发送角的连续死区，单位度。
    };

    /** @brief pitch 为电机绝对角度数，yaw 为连续绝对角度数。 */
    struct Output {
        double pitch_deg;
        double yaw_deg;
    };

    /** @brief 创建默认关闭的平滑器，保持已有绝对角发送行为。 */
    GimbalCommandSmoother(): GimbalCommandSmoother(Config {}) {}

    /**
     * @brief 创建采用指定参数的平滑器。
     * @param config 平滑开关、正有限时间常数、角速度、反馈超时和非负俯仰死区。
     * @warning 非法时间、角速度或死区参数会抛出 std::invalid_argument。
     */
    explicit GimbalCommandSmoother(Config config): config_(config) {
        if (!positive_finite(config_.time_constant_s) || !positive_finite(config_.pitch_rate_deg_s)
            || !positive_finite(config_.yaw_rate_deg_s)
            || !positive_finite(config_.feedback_timeout_s)
            || !std::isfinite(config_.pitch_deadband_deg) || config_.pitch_deadband_deg < 0.0)
        {
            throw std::invalid_argument("Invalid gimbal smoothing time, rate, timeout or deadband");
        }
    }

    /**
     * @brief 更新实测姿态，保留连续跟随时的滤波状态。
     * @param pitch_deg 已限位的电机绝对 pitch，单位度。
     * @param yaw_deg 实测绝对 yaw，单位度，可跨越一整圈。
     * @param time 本次接收姿态的单调时钟时刻。
     * @note 非有限角度会清空反馈和滤波状态，直至再次收到有效反馈。
     */
    void set_feedback(double pitch_deg, double yaw_deg, TimePoint time) {
        if (!std::isfinite(pitch_deg) || !std::isfinite(yaw_deg)) {
            feedback_.reset();
            reset();
            return;
        }
        feedback_ = Output { pitch_deg, yaw_deg };
        feedback_time_ = time;
    }

    /**
     * @brief 生成本次发送的平滑绝对角指令。
     * @param target_pitch_deg 已校准和限位的电机绝对 pitch，单位度。
     * @param target_yaw_deg 期望绝对 yaw，单位度，可使用等价的一圈内角度。
     * @param time 本次发送指令的单调时钟时刻。
     * @param apply_pitch_deadband 是否启用俯仰死区，回水平时关闭以到达精确零位。
     * @return 有效发送角；启用时缺少新鲜反馈或时间倒退则返回空值。
     * @note 首次或超过 0.30 秒未发送后从当前反馈起步，首包保持反馈角度。
     *       每次积分最多使用 0.05 秒；yaw 沿最短角差变化并保持输出连续。
     *       俯仰先减去连续死区，死区内保持上一输出，yaw 不受死区影响。
     *       关闭平滑时直接透传有限目标角，不要求反馈。
     */
    std::optional<Output> update(
        double target_pitch_deg,
        double target_yaw_deg,
        TimePoint time,
        bool apply_pitch_deadband = true
    ) {
        if (!std::isfinite(target_pitch_deg) || !std::isfinite(target_yaw_deg)) {
            reset();
            return std::nullopt;
        }
        if (!config_.enabled) {
            return Output { target_pitch_deg, target_yaw_deg };
        }
        if (!feedback_.has_value()) {
            reset();
            return std::nullopt;
        }
        const double feedback_age_s = elapsed_seconds(time, feedback_time_);
        if (feedback_age_s < 0.0 || feedback_age_s > config_.feedback_timeout_s) {
            reset();
            return std::nullopt;
        }

        double elapsed_s = 0.0;
        if (output_.has_value()) {
            elapsed_s = elapsed_seconds(time, output_time_);
            if (elapsed_s < 0.0) {
                reset();
                return std::nullopt;
            }
            if (elapsed_s > 0.30) {
                reset();
            }
        }
        if (!output_.has_value()) {
            output_ = feedback_;
            output_time_ = time;
            return output_;
        }

        const double dt_s = std::min(elapsed_s, 0.05);
        const double alpha = -std::expm1(-dt_s / config_.time_constant_s);
        double pitch_difference_deg = target_pitch_deg - output_->pitch_deg;
        if (apply_pitch_deadband) {
            const double effective_magnitude =
                std::max(0.0, std::abs(pitch_difference_deg) - config_.pitch_deadband_deg);
            pitch_difference_deg = std::copysign(effective_magnitude, pitch_difference_deg);
        }
        const double yaw_difference_deg = std::remainder(target_yaw_deg - output_->yaw_deg, 360.0);
        output_->pitch_deg +=
            limited_step(pitch_difference_deg, alpha, config_.pitch_rate_deg_s * dt_s);
        output_->yaw_deg += limited_step(yaw_difference_deg, alpha, config_.yaw_rate_deg_s * dt_s);
        output_time_ = time;
        return output_;
    }

    /** @brief 清空滤波状态并保留反馈，使下次有效指令从最新实测姿态起步。 */
    void reset() {
        output_.reset();
    }

private:
    /**
     * @brief 检查配置值是否为正有限数。
     * @param value 待校验的时间或角速度参数。
     * @return 参数为有限正数时返回 true。
     */
    static bool positive_finite(double value) {
        return std::isfinite(value) && value > 0.0;
    }

    /**
     * @brief 计算两个单调时钟时刻之间的秒数。
     * @param current 当前时刻。
     * @param previous 上次时刻。
     * @return 当前时刻减去上次时刻的秒数，可为负。
     */
    static double elapsed_seconds(TimePoint current, TimePoint previous) {
        return std::chrono::duration<double>(current - previous).count();
    }

    /**
     * @brief 限制低通后的单次角度步长。
     * @param difference_deg 目标相对上次输出的角差，单位度。
     * @param alpha 本次低通系数，范围为 [0, 1]。
     * @param max_step_deg 本次允许的最大角度变化，单位度。
     * @return 经过低通和角速度约束的有符号角度步长。
     */
    static double limited_step(double difference_deg, double alpha, double max_step_deg) {
        return std::clamp(alpha * difference_deg, -max_step_deg, max_step_deg);
    }

    Config config_;
    std::optional<Output> feedback_;
    TimePoint feedback_time_ {};
    std::optional<Output> output_;
    TimePoint output_time_ {};
};

} // namespace qd::serial_driver::protocol

#endif // RM_SERIAL_DRIVER_PROTOCOL_GIMBAL_COMMAND_SMOOTHER_HPP_
