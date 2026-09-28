#include "rm_serial_driver/protocol/seasky_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"

namespace qd::serial_driver::protocol {
namespace {
    constexpr std::size_t MCU_FLOAT_COUNT = 15;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    /**
     * @brief 将电控两 float 姿态或 15 个 float 遥测映射到 ROS 消息。
     * @param values [yaw(度), pitch(弧度)] 或从 yaw 到 cmd_y 的 15 个遥测字段。
     * @param data 成功时写入的 ROS 接收消息。
     * @param calibration 电控 pitch 的水平零位及方向。
     * @return 数量和姿态角均有效时返回 true。
     */
    bool decode_mcu_data(
        const std::vector<float>& values,
        rm_interfaces::msg::SerialReceiveData& data,
        const PitchCalibration& calibration
    ) {
        if ((values.size() != 2 && values.size() != MCU_FLOAT_COUNT) || !std::isfinite(values[0])
            || !std::isfinite(values[1])
            || (values.size() == MCU_FLOAT_COUNT && !std::isfinite(values[2])))
        {
            return false;
        }
        data.yaw = values[0];
        data.pitch = static_cast<float>(
            (values[1] - calibration.horizontal_rad) * calibration.up_sign * RAD_TO_DEG
        );
        data.roll = values.size() == MCU_FLOAT_COUNT ? values[2] : 0.0F;
        // 当前电控浮点数据段没有模式、实际弹速或 MCU 时间戳。
        data.mode = 0;
        data.bullet_speed = 0.0F;
        data.mcu_timestamp = 0;
        return true;
    }
} // namespace

SeaskyProtocol::SeaskyProtocol(
    std::string_view protocol_type,
    std::string_view port_name,
    int speed,
    bool enable_data_print,
    CrcProfile crc_profile,
    PitchCalibration pitch_calibration
):
    SeaskyProtocol(
        protocol_type,
        std::make_shared<UartTransporter>(std::string(port_name), speed),
        enable_data_print,
        crc_profile,
        pitch_calibration
    ) {}

SeaskyProtocol::SeaskyProtocol(
    std::string_view protocol_type,
    std::shared_ptr<TransporterInterface> transporter,
    bool enable_data_print,
    CrcProfile crc_profile,
    PitchCalibration pitch_calibration
):
    protocol_type_(protocol_type),
    transporter_(std::move(transporter)),
    crc_profile_(crc_profile),
    pitch_calibration_(pitch_calibration),
    parser_(crc_profile),
    enable_data_print_(enable_data_print) {
    if (transporter_ == nullptr) {
        throw std::invalid_argument("SeaskyProtocol transporter is null");
    }
    if (!std::isfinite(pitch_calibration_.horizontal_rad)
        || (pitch_calibration_.up_sign != -1.0 && pitch_calibration_.up_sign != 1.0)
        || !std::isfinite(pitch_calibration_.min_rad) || !std::isfinite(pitch_calibration_.max_rad)
        || pitch_calibration_.min_rad >= pitch_calibration_.max_rad
        || pitch_calibration_.horizontal_rad < pitch_calibration_.min_rad
        || pitch_calibration_.horizontal_rad > pitch_calibration_.max_rad)
    {
        throw std::invalid_argument("Invalid pitch zero, direction or limits");
    }
    if (!transporter_->isOpen() && !transporter_->open()) {
        FYT_ERROR(
            "serial_driver",
            "Failed to open Seasky serial port: {}",
            transporter_->errorMessage()
        );
    }
}

void SeaskyProtocol::send(const rm_interfaces::msg::GimbalCmd& data) {
    // 两 float 帧无法传递无目标标志，不能把解算器的无目标零值当成角度指令。
    if (data.distance < 0.0) {
        return;
    }
    const auto requested_pitch_rad =
        pitch_calibration_.horizontal_rad + pitch_calibration_.up_sign * data.pitch * DEG_TO_RAD;
    const auto yaw_deg = static_cast<float>(data.yaw);
    if (!std::isfinite(requested_pitch_rad) || !std::isfinite(yaw_deg)) {
        FYT_WARN("serial_driver", "Skipping non-finite gimbal angle");
        return;
    }
    const double limited_pitch_rad =
        std::clamp(requested_pitch_rad, pitch_calibration_.min_rad, pitch_calibration_.max_rad);
    const auto pitch_rad = static_cast<float>(limited_pitch_rad);
    const auto frame = FloatFrame::encode({ pitch_rad, yaw_deg }, crc_profile_);
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!transporter_->isOpen() && !transporter_->open()) {
        FYT_ERROR("serial_driver", "Cannot open serial port: {}", transporter_->errorMessage());
        return;
    }
    std::size_t written = 0;
    while (written < frame.size()) {
        const auto count = transporter_->write(frame.data() + written, frame.size() - written);
        if (count <= 0) {
            FYT_ERROR(
                "serial_driver",
                "Failed to send Seasky frame: {}",
                transporter_->errorMessage()
            );
            transporter_->close();
            return;
        }
        written += static_cast<std::size_t>(count);
    }
    const auto now = std::chrono::steady_clock::now();
    if (enable_data_print_ && now - last_send_print_ >= std::chrono::seconds(1)) {
        std::cout << "TX Seasky WRITTEN: pitch_rad=" << pitch_rad << " yaw_deg=" << yaw_deg
                  << " cmd_pitch_deg=" << data.pitch << " distance_m=" << data.distance
                  << " pitch_limited=" << (limited_pitch_rad != requested_pitch_rad) << std::endl;
        last_send_print_ = now;
    }
}

bool SeaskyProtocol::receive(rm_interfaces::msg::SerialReceiveData& data) {
    std::vector<float> values;
    std::array<std::uint8_t, 256> bytes {};
    while (rclcpp::ok()) {
        while (parser_.next(values)) {
            if (decode_mcu_data(values, data, pitch_calibration_)) {
                const auto now = std::chrono::steady_clock::now();
                if (enable_data_print_ && now - last_data_print_ >= std::chrono::seconds(1)) {
                    std::cout << "RX Seasky OK: floats=" << values.size() << " yaw_deg=" << data.yaw
                              << " pitch_rad=" << values[1] << " pitch_deg=" << data.pitch
                              << " roll_deg=" << data.roll << std::endl;
                    last_data_print_ = now;
                }
                return true;
            }
            FYT_WARN(
                "serial_driver",
                "Ignoring invalid Seasky attitude with {} floats (expected 2 or 15)",
                values.size()
            );
        }
        if (!transporter_->isOpen() && !transporter_->open()) {
            FYT_ERROR("serial_driver", "Cannot open serial port: {}", transporter_->errorMessage());
            return false;
        }
        const auto count = transporter_->read(bytes.data(), bytes.size());
        if (count < 0) {
            FYT_ERROR(
                "serial_driver",
                "Failed to read Seasky frame: {}",
                transporter_->errorMessage()
            );
            transporter_->close();
            return false;
        }
        if (count == 0) {
            continue;
        }
        parser_.append(bytes.data(), static_cast<std::size_t>(count));
    }
    return false;
}

std::vector<rclcpp::SubscriptionBase::SharedPtr>
SeaskyProtocol::getSubscriptions(rclcpp::Node::SharedPtr node) {
    auto armor = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal",
        rclcpp::SensorDataQoS(),
        [this](rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { send(*msg); }
    );
    if (protocol_type_ == "hero" || protocol_type_ == "air") {
        return { armor };
    }
    auto rune = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "rune_solver/cmd_gimbal",
        rclcpp::SensorDataQoS(),
        [this](rm_interfaces::msg::GimbalCmd::ConstSharedPtr msg) { send(*msg); }
    );
    return { armor, rune };
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
SeaskyProtocol::getClients(rclcpp::Node::SharedPtr node) const {
    auto detector = node->create_client<rm_interfaces::srv::SetMode>(
        "armor_detector/set_mode",
        rmw_qos_profile_services_default
    );
    if (protocol_type_ == "hero" || protocol_type_ == "air") {
        return { detector };
    }
    auto solver = node->create_client<rm_interfaces::srv::SetMode>(
        "armor_solver/set_mode",
        rmw_qos_profile_services_default
    );
    if (protocol_type_ == "sentry") {
        return { detector, solver };
    }
    auto rune_detector = node->create_client<rm_interfaces::srv::SetMode>(
        "rune_detector/set_mode",
        rmw_qos_profile_services_default
    );
    auto rune_solver = node->create_client<rm_interfaces::srv::SetMode>(
        "rune_solver/set_mode",
        rmw_qos_profile_services_default
    );
    return { detector, solver, rune_detector, rune_solver };
}

std::string SeaskyProtocol::getErrorMessage() {
    return transporter_->errorMessage();
}

} // namespace qd::serial_driver::protocol
