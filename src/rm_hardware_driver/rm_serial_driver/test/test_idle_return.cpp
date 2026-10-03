#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "rm_serial_driver/float_frame.hpp"
#include "rm_serial_driver/protocol/seasky_protocol.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/logger/log.hpp"

namespace {

using qd::serial_driver::CrcProfile;
using qd::serial_driver::FloatFrame;
using qd::serial_driver::protocol::GimbalCommandSmoother;
using qd::serial_driver::protocol::IdleReturnConfig;
using qd::serial_driver::protocol::PitchCalibration;
using qd::serial_driver::protocol::SeaskyProtocol;

constexpr double DEG_TO_RAD = M_PI / 180.0;

/** @brief 为回中测试保存下行帧，并允许逐次追加电控实测反馈。 */
class IdleReturnTransporter: public qd::serial_driver::TransporterInterface {
public:
    bool open() override {
        open_ = true;
        return true;
    }

    void close() override {
        open_ = false;
    }

    bool isOpen() override {
        return open_;
    }

    int read(void* buffer, std::size_t length) override {
        if (!open_ || read_offset_ == incoming_.size()) {
            return -1;
        }
        const auto count = std::min(length, incoming_.size() - read_offset_);
        std::memcpy(buffer, incoming_.data() + read_offset_, count);
        read_offset_ += count;
        return static_cast<int>(count);
    }

    int write(const void* buffer, std::size_t length) override {
        if (!open_) {
            return -1;
        }
        const auto* bytes = static_cast<const std::uint8_t*>(buffer);
        sent_frames.emplace_back(bytes, bytes + length);
        return static_cast<int>(length);
    }

    std::string errorMessage() override {
        return "idle return test transport error";
    }

    /**
     * @brief 追加电控 yaw 和电机 pitch 实测姿态。
     * @param yaw_deg 实测 yaw，单位度。
     * @param pitch_rad 实测电机绝对 pitch，单位弧度。
     */
    void append_feedback(float yaw_deg, float pitch_rad) {
        const auto frame = FloatFrame::encode({ yaw_deg, pitch_rad }, CrcProfile::CRC8_31_MODBUS);
        incoming_.insert(incoming_.end(), frame.begin(), frame.end());
    }

    std::vector<std::vector<std::uint8_t>> sent_frames;

private:
    bool open_ = false;
    std::size_t read_offset_ = 0;
    std::vector<std::uint8_t> incoming_;
};

class IdleReturnTest: public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
        FYT_REGISTER_LOGGER("serial_driver", "/tmp/qd-idle-return-test-logs", WARN);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        make_protocol();
    }

    /**
     * @brief 创建启用确定性时钟与平滑回中的串口协议。
     * @param calibration 电机 pitch 标定与限位。
     * @param idle_enabled 是否允许自动水平回中。
     */
    void make_protocol(PitchCalibration calibration = {}, bool idle_enabled = true) {
        transporter_ = std::make_shared<IdleReturnTransporter>();
        GimbalCommandSmoother::Config smoothing;
        smoothing.enabled = true;
        smoothing.pitch_deadband_deg = 0.25;
        IdleReturnConfig idle_return;
        idle_return.enabled = idle_enabled;
        protocol_ = std::make_unique<SeaskyProtocol>(
            "hero",
            transporter_,
            false,
            CrcProfile::CRC8_31_MODBUS,
            calibration,
            smoothing,
            idle_return,
            [this]() { return now_; }
        );
    }

    /**
     * @brief 将测试单调时钟设到指定时刻。
     * @param seconds 从测试时钟原点起的秒数。
     */
    void set_time(double seconds) {
        now_ = GimbalCommandSmoother::TimePoint(
            std::chrono::duration_cast<GimbalCommandSmoother::Clock::duration>(
                std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(seconds * 1e9)))
            )
        );
    }

    /**
     * @brief 将实测姿态编码并通过真实协议接收路径更新反馈。
     * @param yaw_deg 电控实测 yaw，单位度。
     * @param pitch_rad 电控实测绝对 pitch，单位弧度。
     */
    void receive_feedback(float yaw_deg, float pitch_rad) {
        transporter_->append_feedback(yaw_deg, pitch_rad);
        rm_interfaces::msg::SerialReceiveData received;
        ASSERT_TRUE(protocol_->receive(received));
        EXPECT_FLOAT_EQ(received.yaw, yaw_deg);
    }

    /**
     * @brief 发送使用固定目标 ID 的有效装甲板指令。
     * @param pitch_deg 以水平为零、抬头为正的目标 pitch，单位度。
     * @param yaw_deg 目标 yaw，单位度。
     */
    void send_target(double pitch_deg = 10.0, double yaw_deg = 90.0) {
        rm_interfaces::msg::GimbalCmd target;
        target.id = "1";
        target.pitch = pitch_deg;
        target.yaw = yaw_deg;
        target.distance = 1.0;
        protocol_->send(target);
    }

    /** @brief 向协议发送无目标消息，模拟解算器连续丢帧。 */
    void send_no_target() {
        rm_interfaces::msg::GimbalCmd missing;
        missing.distance = -1.0;
        protocol_->send(missing);
    }

    /**
     * @brief 解码最后一个真实下行帧。
     * @return 电机 pitch 和 yaw 的绝对目标角，单位度。
     */
    GimbalCommandSmoother::Output last_command() const {
        if (transporter_->sent_frames.empty()) {
            ADD_FAILURE() << "Expected an outgoing frame";
            return { 0.0, 0.0 };
        }
        std::vector<float> values;
        if (!FloatFrame::decode(
                transporter_->sent_frames.back(),
                values,
                CrcProfile::CRC8_31_MODBUS
            )
            || values.size() != 2)
        {
            ADD_FAILURE() << "Expected a valid [pitch_deg, yaw_deg] frame";
            return { 0.0, 0.0 };
        }
        return { values[0], values[1] };
    }

    GimbalCommandSmoother::TimePoint now_ {};
    std::shared_ptr<IdleReturnTransporter> transporter_;
    std::unique_ptr<SeaskyProtocol> protocol_;
};

TEST_F(IdleReturnTest, startup_waits_for_feedback_instead_of_sending_zero_yaw) {
    protocol_->send_idle();
    set_time(1.0);
    protocol_->send_idle();
    send_no_target();
    protocol_->send_idle();

    EXPECT_TRUE(transporter_->sent_frames.empty());
}

TEST_F(IdleReturnTest, startup_returns_smoothly_to_horizontal_and_keeps_measured_yaw) {
    receive_feedback(118.0F, 0.75F);
    protocol_->send_idle();
    ASSERT_EQ(transporter_->sent_frames.size(), 1U);
    auto previous = last_command();
    EXPECT_NEAR(previous.pitch_deg * DEG_TO_RAD, 0.75, 1e-6);
    EXPECT_DOUBLE_EQ(previous.yaw_deg, 118.0);

    for (int step = 1; step <= 200; ++step) {
        set_time(step * 0.01);
        receive_feedback(118.0F, static_cast<float>(previous.pitch_deg * DEG_TO_RAD));
        protocol_->send_idle();
        const auto current = last_command();
        EXPECT_LE(current.pitch_deg, previous.pitch_deg);
        EXPECT_GE(current.pitch_deg * DEG_TO_RAD, 0.58 - 1e-6);
        EXPECT_LE(previous.pitch_deg - current.pitch_deg, 0.20 + 1e-5);
        EXPECT_DOUBLE_EQ(current.yaw_deg, 118.0);
        previous = current;
    }

    EXPECT_NEAR(previous.pitch_deg * DEG_TO_RAD, 0.58, 1e-5);
    EXPECT_EQ(transporter_->sent_frames.size(), 201U);
}

TEST_F(IdleReturnTest, idle_yaw_is_locked_once_instead_of_chasing_new_feedback) {
    receive_feedback(118.0F, 0.75F);
    protocol_->send_idle();
    set_time(0.01);
    receive_feedback(140.0F, 0.75F);
    protocol_->send_idle();

    EXPECT_DOUBLE_EQ(last_command().yaw_deg, 118.0);
}

TEST_F(IdleReturnTest, brief_loss_keeps_smoothing_state_and_same_target_resumes_without_reset) {
    receive_feedback(118.0F, 0.65F);
    send_target();
    set_time(0.01);
    send_target();
    const auto before_loss = last_command();
    const auto frames_before_loss = transporter_->sent_frames.size();
    set_time(0.05);
    send_no_target();
    protocol_->send_idle();
    set_time(0.10);
    receive_feedback(118.0F, 0.75F);
    protocol_->send_idle();
    EXPECT_EQ(transporter_->sent_frames.size(), frames_before_loss);

    send_target();

    const auto resumed = last_command();
    EXPECT_LT(resumed.pitch_deg, before_loss.pitch_deg);
    EXPECT_LE(before_loss.pitch_deg - resumed.pitch_deg, 1.0 + 1e-5);
    EXPECT_GT(resumed.yaw_deg, 90.0);
    EXPECT_LT(resumed.yaw_deg, before_loss.yaw_deg);
    EXPECT_EQ(transporter_->sent_frames.size(), frames_before_loss + 1);
}

TEST_F(IdleReturnTest, repeated_no_target_messages_do_not_extend_the_return_deadline) {
    receive_feedback(118.0F, 0.65F);
    send_target();
    const auto tracking_frames = transporter_->sent_frames.size();
    for (int step = 1; step <= 14; ++step) {
        set_time(step * 0.01);
        send_no_target();
        protocol_->send_idle();
        EXPECT_EQ(transporter_->sent_frames.size(), tracking_frames);
    }
    set_time(0.15);
    receive_feedback(120.0F, 0.70F);
    send_no_target();
    protocol_->send_idle();

    EXPECT_EQ(transporter_->sent_frames.size(), tracking_frames + 1);
    EXPECT_NEAR(last_command().pitch_deg * DEG_TO_RAD, 0.70, 1e-6);
    EXPECT_DOUBLE_EQ(last_command().yaw_deg, 120.0);
    set_time(0.16);
    protocol_->send_idle();
    EXPECT_LT(last_command().pitch_deg * DEG_TO_RAD, 0.70);
}

TEST_F(IdleReturnTest, stopping_the_command_stream_also_starts_horizontal_return) {
    receive_feedback(118.0F, 0.65F);
    send_target();
    set_time(0.149);
    protocol_->send_idle();
    EXPECT_EQ(transporter_->sent_frames.size(), 1U);
    set_time(0.15);
    receive_feedback(123.0F, 0.68F);
    protocol_->send_idle();

    EXPECT_EQ(transporter_->sent_frames.size(), 2U);
    EXPECT_NEAR(last_command().pitch_deg * DEG_TO_RAD, 0.68, 1e-6);
    EXPECT_DOUBLE_EQ(last_command().yaw_deg, 123.0);
    set_time(0.16);
    protocol_->send_idle();
    EXPECT_LT(last_command().pitch_deg * DEG_TO_RAD, 0.68);
}

TEST_F(IdleReturnTest, fresh_target_commands_prevent_additional_timer_writes) {
    receive_feedback(118.0F, 0.65F);
    for (int step = 0; step <= 10; ++step) {
        set_time(step * 0.05);
        receive_feedback(118.0F, 0.65F);
        send_target();
        const auto tracking_frames = transporter_->sent_frames.size();
        set_time(step * 0.05 + 0.01);
        protocol_->send_idle();
        EXPECT_EQ(transporter_->sent_frames.size(), tracking_frames);
    }

    EXPECT_EQ(transporter_->sent_frames.size(), 11U);
}

TEST_F(IdleReturnTest, stale_feedback_stops_idle_and_recovery_locks_the_new_measured_yaw) {
    receive_feedback(118.0F, 0.75F);
    protocol_->send_idle();
    set_time(0.01);
    protocol_->send_idle();
    const auto frames_before_timeout = transporter_->sent_frames.size();
    set_time(0.301);
    protocol_->send_idle();
    set_time(0.50);
    protocol_->send_idle();
    EXPECT_EQ(transporter_->sent_frames.size(), frames_before_timeout);

    receive_feedback(-30.0F, 0.69F);
    protocol_->send_idle();

    EXPECT_EQ(transporter_->sent_frames.size(), frames_before_timeout + 1);
    EXPECT_NEAR(last_command().pitch_deg * DEG_TO_RAD, 0.69, 1e-6);
    EXPECT_DOUBLE_EQ(last_command().yaw_deg, -30.0);
}

TEST_F(IdleReturnTest, stale_feedback_suppresses_target_commands_until_measurement_recovers) {
    receive_feedback(118.0F, 0.65F);
    send_target();
    set_time(0.301);
    send_target();
    EXPECT_EQ(transporter_->sent_frames.size(), 1U);

    set_time(0.31);
    receive_feedback(-30.0F, 0.69F);
    send_target();

    EXPECT_EQ(transporter_->sent_frames.size(), 2U);
    EXPECT_NEAR(last_command().pitch_deg * DEG_TO_RAD, 0.69, 1e-6);
    EXPECT_DOUBLE_EQ(last_command().yaw_deg, -30.0);
}

TEST_F(IdleReturnTest, capture_during_horizontal_return_starts_from_latest_measured_position) {
    receive_feedback(118.0F, 0.75F);
    protocol_->send_idle();
    set_time(0.01);
    protocol_->send_idle();
    set_time(0.02);
    receive_feedback(-30.0F, 0.62F);

    send_target();

    EXPECT_NEAR(last_command().pitch_deg * DEG_TO_RAD, 0.62, 1e-6);
    EXPECT_DOUBLE_EQ(last_command().yaw_deg, -30.0);
    const auto capture_frames = transporter_->sent_frames.size();
    protocol_->send_idle();
    EXPECT_EQ(transporter_->sent_frames.size(), capture_frames);
    set_time(0.03);
    send_target();
    EXPECT_LT(last_command().pitch_deg * DEG_TO_RAD, 0.62);
    EXPECT_GT(last_command().yaw_deg, -30.0);
}

TEST_F(IdleReturnTest, custom_horizontal_calibration_is_used_and_mechanical_limits_are_respected) {
    PitchCalibration calibration;
    calibration.horizontal_rad = 0.20;
    calibration.up_sign = 1.0;
    calibration.min_rad = -0.10;
    calibration.max_rad = 0.50;
    make_protocol(calibration);
    receive_feedback(540.0F, 0.50F);
    protocol_->send_idle();
    auto previous = last_command();
    EXPECT_NEAR(previous.pitch_deg * DEG_TO_RAD, 0.50, 1e-6);
    for (int step = 1; step <= 250; ++step) {
        set_time(step * 0.01);
        receive_feedback(540.0F, static_cast<float>(previous.pitch_deg * DEG_TO_RAD));
        protocol_->send_idle();
        const auto current = last_command();
        EXPECT_GE(current.pitch_deg * DEG_TO_RAD, calibration.min_rad);
        EXPECT_LE(current.pitch_deg * DEG_TO_RAD, calibration.max_rad + 1e-6);
        EXPECT_GE(current.pitch_deg * DEG_TO_RAD, calibration.horizontal_rad - 1e-6);
        EXPECT_LE(current.pitch_deg, previous.pitch_deg);
        EXPECT_DOUBLE_EQ(current.yaw_deg, 540.0);
        previous = current;
    }

    EXPECT_NEAR(previous.pitch_deg * DEG_TO_RAD, calibration.horizontal_rad, 1e-5);
}

TEST_F(IdleReturnTest, disabling_idle_return_leaves_startup_and_lost_target_without_writes) {
    make_protocol({}, false);
    receive_feedback(118.0F, 0.75F);
    protocol_->send_idle();
    EXPECT_TRUE(transporter_->sent_frames.empty());
    send_target();
    const auto tracking_frames = transporter_->sent_frames.size();
    set_time(0.20);
    receive_feedback(118.0F, 0.75F);
    send_no_target();
    protocol_->send_idle();

    EXPECT_EQ(transporter_->sent_frames.size(), tracking_frames);
}

} // namespace
