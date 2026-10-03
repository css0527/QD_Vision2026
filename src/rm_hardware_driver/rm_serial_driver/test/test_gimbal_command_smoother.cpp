#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "rm_serial_driver/protocol/gimbal_command_smoother.hpp"

namespace {

using qd::serial_driver::protocol::GimbalCommandSmoother;

/**
 * @brief 为纯平滑测试创建指定秒数的单调时钟时刻。
 * @param seconds 相对于测试时钟原点的秒数。
 * @return 对应的单调时钟时刻。
 */
GimbalCommandSmoother::TimePoint at_seconds(double seconds) {
    const auto nanoseconds =
        std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(seconds * 1e9)));
    return GimbalCommandSmoother::TimePoint(
        std::chrono::duration_cast<GimbalCommandSmoother::Clock::duration>(nanoseconds)
    );
}

/**
 * @brief 创建使用默认时间常数和角速度的已启用平滑配置。
 * @return 用于测试主动平滑的配置。
 */
GimbalCommandSmoother::Config enabled_config() {
    GimbalCommandSmoother::Config config;
    config.enabled = true;
    return config;
}

TEST(GimbalCommandSmootherTest, disabled_smoothing_preserves_absolute_angles_without_feedback) {
    GimbalCommandSmoother smoother;
    const auto output = smoother.update(33.2, -179.0, at_seconds(1.0));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 33.2);
    EXPECT_DOUBLE_EQ(output->yaw_deg, -179.0);
}

TEST(
    GimbalCommandSmootherTest,
    first_command_holds_measured_position_instead_of_jumping_to_target
) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(42.0, 118.0, at_seconds(1.0));
    const auto output = smoother.update(23.0, 150.0, at_seconds(1.0));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 42.0);
    EXPECT_DOUBLE_EQ(output->yaw_deg, 118.0);
}

TEST(GimbalCommandSmootherTest, upward_pitch_decreases_motor_angle_with_bounded_speed) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.01));
    ASSERT_TRUE(output.has_value());
    EXPECT_NEAR(output->pitch_deg, 33.0, 1e-12);
    EXPECT_NEAR(output->yaw_deg, 0.6, 1e-12);
}

TEST(GimbalCommandSmootherTest, downward_pitch_increases_motor_angle_with_bounded_speed) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(43.0, -90.0, at_seconds(0.0)).has_value());
    const auto output = smoother.update(43.0, -90.0, at_seconds(0.01));
    ASSERT_TRUE(output.has_value());
    EXPECT_NEAR(output->pitch_deg, 33.4, 1e-12);
    EXPECT_NEAR(output->yaw_deg, -0.6, 1e-12);
}

TEST(
    GimbalCommandSmootherTest,
    low_pass_response_depends_on_elapsed_time_instead_of_callback_count
) {
    auto config = enabled_config();
    config.pitch_rate_deg_s = 1000.0;
    config.yaw_rate_deg_s = 1000.0;
    GimbalCommandSmoother fast(config);
    GimbalCommandSmoother slow(config);
    fast.set_feedback(30.0, 0.0, at_seconds(0.0));
    slow.set_feedback(30.0, 0.0, at_seconds(0.0));
    ASSERT_TRUE(fast.update(31.0, 1.0, at_seconds(0.0)).has_value());
    ASSERT_TRUE(slow.update(31.0, 1.0, at_seconds(0.0)).has_value());
    for (int step = 1; step < 10; ++step) {
        ASSERT_TRUE(fast.update(31.0, 1.0, at_seconds(0.01 * step)).has_value());
    }
    ASSERT_TRUE(slow.update(31.0, 1.0, at_seconds(0.05)).has_value());
    const auto fast_output = fast.update(31.0, 1.0, at_seconds(0.10));
    const auto slow_output = slow.update(31.0, 1.0, at_seconds(0.10));
    ASSERT_TRUE(fast_output.has_value());
    ASSERT_TRUE(slow_output.has_value());
    EXPECT_NEAR(fast_output->pitch_deg, slow_output->pitch_deg, 1e-9);
    EXPECT_NEAR(fast_output->yaw_deg, 1.0 - std::exp(-1.0), 1e-9);
    EXPECT_NEAR(fast_output->yaw_deg, slow_output->yaw_deg, 1e-9);
}

TEST(GimbalCommandSmootherTest, frequent_feedback_does_not_restart_active_smoothing) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.01)).has_value());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.02));
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.02));
    ASSERT_TRUE(output.has_value());
    EXPECT_NEAR(output->pitch_deg, 32.8, 1e-12);
    EXPECT_NEAR(output->yaw_deg, 1.2, 1e-12);
}

TEST(GimbalCommandSmootherTest, reacquisition_after_reset_starts_from_latest_manual_position) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.01)).has_value());
    smoother.reset();
    smoother.set_feedback(42.0, -30.0, at_seconds(0.02));
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.02));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 42.0);
    EXPECT_DOUBLE_EQ(output->yaw_deg, -30.0);
}

TEST(GimbalCommandSmootherTest, reset_retains_recent_feedback) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 118.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 150.0, at_seconds(0.0)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 150.0, at_seconds(0.01)).has_value());
    smoother.reset();
    const auto output = smoother.update(23.0, 150.0, at_seconds(0.02));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 33.2);
    EXPECT_DOUBLE_EQ(output->yaw_deg, 118.0);
}

TEST(GimbalCommandSmootherTest, missing_or_expired_feedback_suppresses_motion_and_resets_state) {
    GimbalCommandSmoother smoother(enabled_config());
    EXPECT_FALSE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.01)).has_value());
    EXPECT_FALSE(smoother.update(23.0, 90.0, at_seconds(0.31)).has_value());
    smoother.set_feedback(42.0, 118.0, at_seconds(0.32));
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.32));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 42.0);
    EXPECT_DOUBLE_EQ(output->yaw_deg, 118.0);
}

TEST(GimbalCommandSmootherTest, yaw_crosses_wrap_boundary_through_short_continuous_rotation) {
    auto config = enabled_config();
    config.yaw_rate_deg_s = 1000.0;
    GimbalCommandSmoother smoother(config);
    smoother.set_feedback(33.2, 179.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(33.2, -179.0, at_seconds(0.0)).has_value());
    auto output = smoother.update(33.2, -179.0, at_seconds(0.05));
    ASSERT_TRUE(output.has_value());
    EXPECT_GT(output->yaw_deg, 179.0);
    EXPECT_LT(output->yaw_deg, 180.0);
    for (int step = 2; step <= 5; ++step) {
        output = smoother.update(33.2, -179.0, at_seconds(0.05 * step));
        ASSERT_TRUE(output.has_value());
    }
    EXPECT_GT(output->yaw_deg, 180.0);
    EXPECT_LT(output->yaw_deg, 181.0);
}

TEST(GimbalCommandSmootherTest, equivalent_yaw_targets_preserve_accumulated_absolute_yaw) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 540.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(33.2, -180.0, at_seconds(0.0)).has_value());
    const auto output = smoother.update(33.2, -180.0, at_seconds(0.01));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->yaw_deg, 540.0);
}

TEST(GimbalCommandSmootherTest, scheduling_pause_caps_single_step_without_large_catch_up) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.20));
    ASSERT_TRUE(output.has_value());
    EXPECT_NEAR(output->pitch_deg, 32.2, 1e-12);
    EXPECT_NEAR(output->yaw_deg, 3.0, 1e-12);
}

TEST(GimbalCommandSmootherTest, long_command_gap_restarts_from_current_feedback) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(0.01)).has_value());
    smoother.set_feedback(42.0, 118.0, at_seconds(0.4));
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.4));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 42.0);
    EXPECT_DOUBLE_EQ(output->yaw_deg, 118.0);
}

TEST(GimbalCommandSmootherTest, backward_feedback_or_command_time_suppresses_motion) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(1.0));
    EXPECT_FALSE(smoother.update(23.0, 90.0, at_seconds(0.99)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(1.0)).has_value());
    ASSERT_TRUE(smoother.update(23.0, 90.0, at_seconds(1.02)).has_value());
    EXPECT_FALSE(smoother.update(23.0, 90.0, at_seconds(1.01)).has_value());
}

TEST(GimbalCommandSmootherTest, invalid_angles_suppress_motion_instead_of_poisoning_filter) {
    GimbalCommandSmoother smoother(enabled_config());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    EXPECT_FALSE(smoother.update(nan, 90.0, at_seconds(0.0)).has_value());
    EXPECT_FALSE(smoother.update(23.0, infinity, at_seconds(0.0)).has_value());
    smoother.set_feedback(nan, 0.0, at_seconds(0.0));
    EXPECT_FALSE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    smoother.set_feedback(33.2, infinity, at_seconds(0.0));
    EXPECT_FALSE(smoother.update(23.0, 90.0, at_seconds(0.0)).has_value());
    smoother.set_feedback(42.0, 118.0, at_seconds(0.01));
    const auto output = smoother.update(23.0, 90.0, at_seconds(0.01));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 42.0);
    EXPECT_DOUBLE_EQ(output->yaw_deg, 118.0);
}

TEST(GimbalCommandSmootherTest, finite_pitch_trajectory_stays_between_limited_baseline_and_target) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(43.0, 0.0, at_seconds(0.0));
    auto output = smoother.update(23.0, 0.0, at_seconds(0.0));
    ASSERT_TRUE(output.has_value());
    double previous_pitch_deg = output->pitch_deg;
    for (int step = 1; step <= 100; ++step) {
        const auto time = at_seconds(step * 0.02);
        smoother.set_feedback(43.0, 0.0, time);
        output = smoother.update(23.0, 0.0, time);
        ASSERT_TRUE(output.has_value());
        EXPECT_GE(output->pitch_deg, 23.0);
        EXPECT_LE(output->pitch_deg, previous_pitch_deg);
        EXPECT_LE(previous_pitch_deg - output->pitch_deg, 0.4 + 1e-12);
        previous_pitch_deg = output->pitch_deg;
    }
}

TEST(GimbalCommandSmootherTest, invalid_time_constant_rates_and_timeout_are_rejected) {
    using Config = GimbalCommandSmoother::Config;
    const std::vector<double> invalid_values { 0.0,
                                               -1.0,
                                               std::numeric_limits<double>::quiet_NaN(),
                                               std::numeric_limits<double>::infinity() };
    const std::vector<double Config::*> fields { &Config::time_constant_s,
                                                 &Config::pitch_rate_deg_s,
                                                 &Config::yaw_rate_deg_s,
                                                 &Config::feedback_timeout_s };
    for (auto field: fields) {
        for (double invalid_value: invalid_values) {
            auto config = enabled_config();
            config.*field = invalid_value;
            EXPECT_THROW(GimbalCommandSmoother { config }, std::invalid_argument);
        }
    }
}

TEST(GimbalCommandSmootherTest, small_alternating_pitch_noise_holds_output_without_freezing_yaw) {
    auto config = enabled_config();
    config.pitch_deadband_deg = 0.25;
    GimbalCommandSmoother smoother(config);
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(33.2, 10.0, at_seconds(0.0)).has_value());
    for (int step = 1; step <= 100; ++step) {
        const auto time = at_seconds(step * 0.01);
        smoother.set_feedback(33.2, 0.0, time);
        const double target_pitch_deg = 33.2 + ((step % 2 == 0) ? 0.2 : -0.2);
        const auto output = smoother.update(target_pitch_deg, 10.0, time);
        ASSERT_TRUE(output.has_value());
        EXPECT_DOUBLE_EQ(output->pitch_deg, 33.2);
        EXPECT_GT(output->yaw_deg, 0.0);
    }
}

TEST(GimbalCommandSmootherTest, real_height_change_tracks_monotonically_with_bounded_pitch_speed) {
    auto config = enabled_config();
    config.pitch_deadband_deg = 0.25;
    config.pitch_rate_deg_s = 12.0;
    GimbalCommandSmoother smoother(config);
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(28.2, 0.0, at_seconds(0.0)).has_value());
    double previous_pitch_deg = 33.2;
    for (int step = 1; step <= 200; ++step) {
        const auto time = at_seconds(step * 0.01);
        smoother.set_feedback(previous_pitch_deg, 0.0, time);
        const auto output = smoother.update(28.2, 0.0, time);
        ASSERT_TRUE(output.has_value());
        EXPECT_LE(output->pitch_deg, previous_pitch_deg);
        EXPECT_GE(output->pitch_deg, 28.2);
        EXPECT_LE(previous_pitch_deg - output->pitch_deg, 0.12 + 1e-12);
        previous_pitch_deg = output->pitch_deg;
    }
    EXPECT_NEAR(previous_pitch_deg, 28.45, 1e-6);
}

TEST(GimbalCommandSmootherTest, noisy_feedback_does_not_move_deadband_center_during_tracking) {
    auto config = enabled_config();
    config.pitch_deadband_deg = 0.25;
    GimbalCommandSmoother smoother(config);
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(33.2, 0.0, at_seconds(0.0)).has_value());
    for (int step = 1; step <= 100; ++step) {
        const auto time = at_seconds(step * 0.01);
        smoother.set_feedback((step % 2 == 0) ? 34.0 : 32.0, 0.0, time);
        const auto output = smoother.update(33.3, 0.0, time);
        ASSERT_TRUE(output.has_value());
        EXPECT_DOUBLE_EQ(output->pitch_deg, 33.2);
    }
}

TEST(GimbalCommandSmootherTest, idle_return_can_reach_horizontal_inside_tracking_deadband) {
    auto config = enabled_config();
    config.pitch_deadband_deg = 0.25;
    GimbalCommandSmoother smoother(config);
    smoother.set_feedback(33.4, 118.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(33.2, 118.0, at_seconds(0.0)).has_value());
    auto output = smoother.update(33.2, 118.0, at_seconds(0.01));
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output->pitch_deg, 33.4);
    for (int step = 2; step <= 200; ++step) {
        const auto time = at_seconds(step * 0.01);
        smoother.set_feedback(output->pitch_deg, 118.0, time);
        output = smoother.update(33.2, 118.0, time, false);
        ASSERT_TRUE(output.has_value());
    }
    EXPECT_NEAR(output->pitch_deg, 33.2, 1e-8);
    EXPECT_DOUBLE_EQ(output->yaw_deg, 118.0);
}

TEST(GimbalCommandSmootherTest, zero_deadband_retains_small_pitch_response) {
    GimbalCommandSmoother smoother(enabled_config());
    smoother.set_feedback(33.2, 0.0, at_seconds(0.0));
    ASSERT_TRUE(smoother.update(33.3, 0.0, at_seconds(0.0)).has_value());
    const auto output = smoother.update(33.3, 0.0, at_seconds(0.01));
    ASSERT_TRUE(output.has_value());
    EXPECT_GT(output->pitch_deg, 33.2);
    EXPECT_LT(output->pitch_deg, 33.3);
}

TEST(GimbalCommandSmootherTest, negative_or_nonfinite_pitch_deadband_is_rejected) {
    const std::vector<double> invalid_values { -0.01,
                                               std::numeric_limits<double>::quiet_NaN(),
                                               std::numeric_limits<double>::infinity() };
    for (double invalid_value: invalid_values) {
        auto config = enabled_config();
        config.pitch_deadband_deg = invalid_value;
        EXPECT_THROW(GimbalCommandSmoother { config }, std::invalid_argument);
    }
}

} // namespace
