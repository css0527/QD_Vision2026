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
    PitchCalibration pitch_calibration,
    GimbalCommandSmoother::Config smoothing,
    IdleReturnConfig idle_return
):
    SeaskyProtocol(
        protocol_type,
        std::make_shared<UartTransporter>(std::string(port_name), speed),
        enable_data_print,
        crc_profile,
        pitch_calibration,
        smoothing,
        idle_return
    ) {}

SeaskyProtocol::SeaskyProtocol(
    std::string_view protocol_type,
    std::shared_ptr<TransporterInterface> transporter,
    bool enable_data_print,
    CrcProfile crc_profile,
    PitchCalibration pitch_calibration,
    GimbalCommandSmoother::Config smoothing,
    IdleReturnConfig idle_return,
    ClockFunction clock
):
    protocol_type_(protocol_type),
    transporter_(std::move(transporter)),
    crc_profile_(crc_profile),
    pitch_calibration_(pitch_calibration),
    command_smoother_(smoothing),
    idle_return_(idle_return),
    clock_(std::move(clock)),
    feedback_timeout_s_(smoothing.feedback_timeout_s),
    parser_(crc_profile),
    enable_data_print_(enable_data_print) {
    if (transporter_ == nullptr) {
        throw std::invalid_argument("SeaskyProtocol transporter is null");
    }
    if (!clock_ || !std::isfinite(idle_return_.command_timeout_s)
        || idle_return_.command_timeout_s <= 0.0)
    {
        throw std::invalid_argument("Invalid idle return timeout or clock");
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
    std::lock_guard<std::mutex> lock(send_mutex_);
    // 短时缺失先保持旧指令，独立定时器超时后回水平，避免每次掉帧重置造成点头。
    if (data.distance < 0.0) {
        return;
    }
    const auto requested_pitch_rad =
        pitch_calibration_.horizontal_rad + pitch_calibration_.up_sign * data.pitch * DEG_TO_RAD;
    const auto yaw_deg = static_cast<float>(data.yaw);
    if (!std::isfinite(requested_pitch_rad) || !std::isfinite(yaw_deg)) {
        command_smoother_.reset();
        FYT_WARN("serial_driver", "Skipping non-finite gimbal angle");
        return;
    }
    const double limited_pitch_rad =
        std::clamp(requested_pitch_rad, pitch_calibration_.min_rad, pitch_calibration_.max_rad);
    // 电控将收到的 pitch 乘 DEG_TO_RAD，所以下行必须发送电机绝对角的度数。
    if (returning_horizontal_ || data.id != last_target_id_) {
        command_smoother_.reset();
        last_target_id_ = data.id;
    }
    returning_horizontal_ = false;
    const auto now = clock_();
    last_target_command_time_ = now;
    write_command(limited_pitch_rad, yaw_deg, data, limited_pitch_rad != requested_pitch_rad, now);
}

void SeaskyProtocol::send_idle() {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (!idle_return_.enabled) {
        return;
    }
    const auto now = clock_();
    if (last_target_command_time_) {
        const auto command_age_s =
            std::chrono::duration<double>(now - *last_target_command_time_).count();
        if (command_age_s >= 0.0 && command_age_s < idle_return_.command_timeout_s) {
            return;
        }
    }
    // 即使关闭跟随低通，回中也须有新鲜姿态，不能用未初始化的 yaw=0。
    const double feedback_age_s =
        feedback_time_ ? std::chrono::duration<double>(now - *feedback_time_).count() : -1.0;
    if (feedback_age_s < 0.0 || feedback_age_s > feedback_timeout_s_) {
        command_smoother_.reset();
        returning_horizontal_ = false;
        return;
    }
    if (!returning_horizontal_) {
        command_smoother_.reset();
        idle_yaw_deg_ = feedback_yaw_deg_;
        returning_horizontal_ = true;
    }
    rm_interfaces::msg::GimbalCmd idle_command;
    idle_command.pitch = 0.0;
    idle_command.yaw = idle_yaw_deg_;
    idle_command.distance = -1.0;
    idle_command.fire_advice = false;
    write_command(pitch_calibration_.horizontal_rad, idle_yaw_deg_, idle_command, false, now);
}

void SeaskyProtocol::write_command(
    double pitch_rad,
    double yaw_deg,
    const rm_interfaces::msg::GimbalCmd& source,
    bool pitch_limited,
    GimbalCommandSmoother::TimePoint now
) {
    // 回水平不用跟随死区，确保最终到达标定零位；有目标时抑制微幅上下换向。
    const auto output =
        command_smoother_.update(pitch_rad * RAD_TO_DEG, yaw_deg, now, !returning_horizontal_);
    if (!output) {
        if (now - last_feedback_warning_ >= std::chrono::seconds(1)) {
            FYT_WARN(
                "serial_driver",
                "Skipping gimbal command: measured feedback is missing or stale"
            );
            last_feedback_warning_ = now;
        }
        return;
    }
    const auto pitch_deg = static_cast<float>(output->pitch_deg);
    const auto sent_yaw_deg = static_cast<float>(output->yaw_deg);
    const auto frame = FloatFrame::encode({ pitch_deg, sent_yaw_deg }, crc_profile_);
    if (!transporter_->isOpen() && !transporter_->open()) {
        command_smoother_.reset();
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
            command_smoother_.reset();
            return;
        }
        written += static_cast<std::size_t>(count);
    }
    if (enable_data_print_ && now - last_send_print_ >= std::chrono::seconds(1)) {
        std::cout << "TX Seasky WRITTEN: pitch_deg=" << pitch_deg << " yaw_deg=" << sent_yaw_deg
                  << " cmd_pitch_deg=" << source.pitch
                  << " target_pitch_rad=" << pitch_deg * DEG_TO_RAD
                  << " goal_pitch_rad=" << pitch_rad << " goal_yaw_deg=" << yaw_deg
                  << " feedback_pitch_rad=" << feedback_pitch_rad_
                  << " feedback_yaw_deg=" << feedback_yaw_deg_ << " distance_m=" << source.distance
                  << " pitch_limited=" << pitch_limited << " idle_return=" << returning_horizontal_
                  << std::endl;
        last_send_print_ = now;
    }
}

bool SeaskyProtocol::receive(rm_interfaces::msg::SerialReceiveData& data) {
    std::vector<float> values;
    std::array<std::uint8_t, 256> bytes {};
    while (rclcpp::ok()) {
        while (parser_.next(values)) {
            if (decode_mcu_data(values, data, pitch_calibration_)) {
                const auto now = clock_();
                {
                    std::lock_guard<std::mutex> lock(send_mutex_);
                    feedback_pitch_rad_ = values[1];
                    feedback_yaw_deg_ = values[0];
                    feedback_time_ = now;
                    // 初始化值也限制到机械范围，避免反馈舍入越界导致首包越限。
                    command_smoother_.set_feedback(
                        std::clamp(
                            feedback_pitch_rad_,
                            pitch_calibration_.min_rad,
                            pitch_calibration_.max_rad
                        ) * RAD_TO_DEG,
                        feedback_yaw_deg_,
                        now
                    );
                }
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
    idle_timer_ = node->create_wall_timer(std::chrono::milliseconds(10), [this]() { send_idle(); });
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
