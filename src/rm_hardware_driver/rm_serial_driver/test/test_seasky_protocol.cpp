#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "rm_serial_driver/float_frame.hpp"
#include "rm_serial_driver/protocol/seasky_protocol.hpp"
#include "rm_serial_driver/transporter_interface.hpp"

namespace {

/** @brief 在内存中模拟串口的分段读取和整帧写入。 */
class TestTransporter: public qd::serial_driver::TransporterInterface {
public:
    explicit TestTransporter(std::vector<std::uint8_t> incoming = {}):
        incoming_(std::move(incoming)) {}

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
        const auto count = std::min({ length, std::size_t(5), incoming_.size() - read_offset_ });
        std::memcpy(buffer, incoming_.data() + read_offset_, count);
        read_offset_ += count;
        return static_cast<int>(count);
    }

    int write(const void* buffer, std::size_t length) override {
        if (!open_) {
            return -1;
        }
        const auto* bytes = static_cast<const std::uint8_t*>(buffer);
        sent.insert(sent.end(), bytes, bytes + length);
        return static_cast<int>(length);
    }

    std::string errorMessage() override {
        return "test transport error";
    }

    std::vector<std::uint8_t> sent;

private:
    bool open_ = false;
    std::size_t read_offset_ = 0;
    std::vector<std::uint8_t> incoming_;
};

class SeaskyProtocolTest: public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(SeaskyProtocolTest, sends_absolute_pitch_in_radians_and_yaw_in_degrees) {
    auto transporter = std::make_shared<TestTransporter>();
    qd::serial_driver::protocol::SeaskyProtocol protocol("hero", transporter, false);
    rm_interfaces::msg::GimbalCmd command;
    command.pitch = 0.0;
    command.yaw = -2.5;
    command.distance = 3.0;
    command.fire_advice = true;

    protocol.send(command);

    EXPECT_EQ(
        transporter->sent,
        (std::vector<std::uint8_t> { 0xA5,
                                     0x08,
                                     0x00,
                                     0x67,
                                     0xE1,
                                     0x7A,
                                     0x14,
                                     0x3F,
                                     0x00,
                                     0x00,
                                     0x20,
                                     0xC0,
                                     0x29,
                                     0xB6 })
    );
}

TEST_F(SeaskyProtocolTest, receives_fragmented_mcu_attitude_frame) {
    const std::vector<float> mcu_values { 10.5F,  0.75F,  1.25F, 100.0F, 1.0F,
                                          200.0F, 300.0F, 4.0F,  500.0F, 600.0F,
                                          700.0F, 8.0F,   9.0F,  10.0F,  11.0F };
    auto transporter =
        std::make_shared<TestTransporter>(qd::serial_driver::FloatFrame::encode(mcu_values));
    qd::serial_driver::protocol::SeaskyProtocol protocol("hero", transporter, false);
    rm_interfaces::msg::SerialReceiveData received;

    ASSERT_TRUE(protocol.receive(received));

    EXPECT_FLOAT_EQ(received.yaw, 10.5F);
    EXPECT_NEAR(received.pitch, -9.74028252F, 1e-5F);
    EXPECT_FLOAT_EQ(received.roll, 1.25F);
    EXPECT_EQ(received.mode, 0);
    EXPECT_FLOAT_EQ(received.bullet_speed, 0.0F);
    EXPECT_EQ(received.mcu_timestamp, 0U);
}

TEST_F(SeaskyProtocolTest, receives_captured_two_float_frame) {
    const std::vector<std::uint8_t> captured { 0xA5, 0x08, 0x00, 0xF4, 0x00, 0x00, 0xEC,
                                               0x42, 0xE1, 0x7A, 0x14, 0x3F, 0xC1, 0x3D };
    auto transporter = std::make_shared<TestTransporter>(captured);
    qd::serial_driver::protocol::SeaskyProtocol
        protocol("hero", transporter, false, qd::serial_driver::CrcProfile::CRC8_31_MODBUS);
    rm_interfaces::msg::SerialReceiveData received;
    received.roll = 9.0F;

    ASSERT_TRUE(protocol.receive(received));

    EXPECT_FLOAT_EQ(received.yaw, 118.0F);
    EXPECT_NEAR(received.pitch, 0.0F, 1e-5F);
    EXPECT_FLOAT_EQ(received.roll, 0.0F);
}

TEST_F(SeaskyProtocolTest, uses_selected_crc_for_outgoing_pitch_yaw) {
    auto transporter = std::make_shared<TestTransporter>();
    qd::serial_driver::protocol::SeaskyProtocol
        protocol("hero", transporter, false, qd::serial_driver::CrcProfile::CRC8_31_MODBUS);
    rm_interfaces::msg::GimbalCmd command;
    command.pitch = 0.0;
    command.yaw = -2.5;
    command.distance = 3.0;

    protocol.send(command);

    EXPECT_EQ(
        transporter->sent,
        (std::vector<std::uint8_t> { 0xA5,
                                     0x08,
                                     0x00,
                                     0xF4,
                                     0xE1,
                                     0x7A,
                                     0x14,
                                     0x3F,
                                     0x00,
                                     0x00,
                                     0x20,
                                     0xC0,
                                     0xED,
                                     0xA9 })
    );
}

TEST_F(SeaskyProtocolTest, recovers_from_corrupt_captured_frame) {
    const std::vector<std::uint8_t> captured { 0xA5, 0x08, 0x00, 0xF4, 0x00, 0x00, 0xEC,
                                               0x42, 0xE1, 0x7A, 0x14, 0x3F, 0xC1, 0x3D };
    auto stream = captured;
    stream[4] ^= 1U;
    stream.insert(stream.end(), captured.begin(), captured.end());
    auto transporter = std::make_shared<TestTransporter>(stream);
    qd::serial_driver::protocol::SeaskyProtocol
        protocol("hero", transporter, false, qd::serial_driver::CrcProfile::CRC8_31_MODBUS);
    rm_interfaces::msg::SerialReceiveData received;

    ASSERT_TRUE(protocol.receive(received));

    EXPECT_FLOAT_EQ(received.yaw, 118.0F);
    EXPECT_NEAR(received.pitch, 0.0F, 1e-5F);
}

TEST_F(SeaskyProtocolTest, preserves_upward_positive_pitch_through_tf_and_send) {
    using qd::serial_driver::CrcProfile;
    using qd::serial_driver::FloatFrame;
    const std::vector<std::pair<float, double>> poses { { 0.58F, 0.0 },
                                                        { 0.4F, 10.313240312 },
                                                        { 0.75F, -9.74028252 } };
    for (const auto& [pitch_rad, expected_pitch_deg]: poses) {
        auto transporter = std::make_shared<TestTransporter>(
            FloatFrame::encode({ 21.0F, pitch_rad }, CrcProfile::CRC8_31_MODBUS)
        );
        qd::serial_driver::protocol::SeaskyProtocol
            protocol("hero", transporter, false, CrcProfile::CRC8_31_MODBUS);
        rm_interfaces::msg::SerialReceiveData received;
        ASSERT_TRUE(protocol.receive(received));
        EXPECT_NEAR(received.pitch, expected_pitch_deg, 1e-5);
        EXPECT_FLOAT_EQ(received.yaw, 21.0F);

        // 使用节点的 TF 约定，正仰角应让云台前向轴朝向正 z。
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, -received.pitch * M_PI / 180.0, received.yaw * M_PI / 180.0);
        const auto forward = tf2::Matrix3x3(orientation) * tf2::Vector3(1.0, 0.0, 0.0);
        EXPECT_NEAR(forward.z(), std::sin(expected_pitch_deg * M_PI / 180.0), 1e-6);

        rm_interfaces::msg::GimbalCmd command;
        command.pitch = received.pitch;
        command.yaw = received.yaw;
        command.distance = 3.0;
        protocol.send(command);
        std::vector<float> sent;
        ASSERT_TRUE(FloatFrame::decode(transporter->sent, sent, CrcProfile::CRC8_31_MODBUS));
        ASSERT_EQ(sent.size(), 2U);
        EXPECT_NEAR(sent[0], pitch_rad, 1e-6F);
        EXPECT_FLOAT_EQ(sent[1], 21.0F);
    }
}

TEST_F(SeaskyProtocolTest, does_not_send_no_target_zero_angles) {
    auto transporter = std::make_shared<TestTransporter>();
    qd::serial_driver::protocol::SeaskyProtocol protocol("hero", transporter, false);
    rm_interfaces::msg::GimbalCmd command;
    command.pitch = 0.0;
    command.yaw = 0.0;
    command.distance = -1.0;

    protocol.send(command);

    EXPECT_TRUE(transporter->sent.empty());
}

TEST_F(SeaskyProtocolTest, converts_commands_and_clamps_to_mechanical_limits) {
    using qd::serial_driver::CrcProfile;
    using qd::serial_driver::FloatFrame;
    const std::vector<std::pair<double, float>> commands { { 0.0, 0.58F },
                                                           { 5.0, 0.4927335374F },
                                                           { -5.0, 0.6672664626F },
                                                           { 90.0, 0.4F },
                                                           { -90.0, 0.75F } };
    for (const auto& [pitch_deg, expected_rad]: commands) {
        auto transporter = std::make_shared<TestTransporter>();
        qd::serial_driver::protocol::SeaskyProtocol
            protocol("hero", transporter, false, CrcProfile::CRC8_31_MODBUS);
        rm_interfaces::msg::GimbalCmd command;
        command.pitch = pitch_deg;
        command.yaw = 118.0;
        command.distance = 3.0;
        protocol.send(command);
        std::vector<float> sent;
        ASSERT_TRUE(FloatFrame::decode(transporter->sent, sent, CrcProfile::CRC8_31_MODBUS));
        ASSERT_EQ(sent.size(), 2U);
        EXPECT_FLOAT_EQ(sent[0], expected_rad);
        EXPECT_FLOAT_EQ(sent[1], 118.0F);
    }
}

TEST_F(SeaskyProtocolTest, applies_custom_zero_direction_and_limits) {
    using qd::serial_driver::CrcProfile;
    using qd::serial_driver::FloatFrame;
    qd::serial_driver::protocol::PitchCalibration calibration;
    calibration.horizontal_rad = 0.2;
    calibration.up_sign = 1.0;
    calibration.min_rad = -0.1;
    calibration.max_rad = 0.5;
    auto transporter = std::make_shared<TestTransporter>(
        FloatFrame::encode({ 21.0F, 0.3F }, CrcProfile::CRC8_31_MODBUS)
    );
    qd::serial_driver::protocol::SeaskyProtocol
        protocol("hero", transporter, false, CrcProfile::CRC8_31_MODBUS, calibration);
    rm_interfaces::msg::SerialReceiveData received;
    ASSERT_TRUE(protocol.receive(received));
    EXPECT_NEAR(received.pitch, 5.72957795, 1e-5);
    rm_interfaces::msg::GimbalCmd command;
    command.pitch = received.pitch;
    command.yaw = received.yaw;
    command.distance = 3.0;
    protocol.send(command);
    std::vector<float> sent;
    ASSERT_TRUE(FloatFrame::decode(transporter->sent, sent, CrcProfile::CRC8_31_MODBUS));
    ASSERT_EQ(sent.size(), 2U);
    EXPECT_FLOAT_EQ(sent[0], 0.3F);
    EXPECT_FLOAT_EQ(sent[1], 21.0F);
}

TEST_F(SeaskyProtocolTest, rejects_invalid_calibration_before_opening_port) {
    using qd::serial_driver::protocol::PitchCalibration;
    using qd::serial_driver::protocol::SeaskyProtocol;
    const std::vector<PitchCalibration> invalid { { 0.58, 0.0, 0.4, 0.75 },
                                                  { 0.58, -1.0, 0.75, 0.4 },
                                                  { 0.58, -1.0, 0.6, 0.75 },
                                                  { 0.58, -1.0, 0.4, 0.5 } };
    for (const auto& calibration: invalid) {
        auto transporter = std::make_shared<TestTransporter>();
        EXPECT_THROW(
            SeaskyProtocol(
                "hero",
                transporter,
                false,
                qd::serial_driver::CrcProfile::CRC8_31_MODBUS,
                calibration
            ),
            std::invalid_argument
        );
        EXPECT_FALSE(transporter->isOpen());
    }
}

} // namespace
