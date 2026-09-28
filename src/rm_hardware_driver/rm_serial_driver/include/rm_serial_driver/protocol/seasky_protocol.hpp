#ifndef RM_SERIAL_DRIVER_PROTOCOL_SEASKY_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_SEASKY_PROTOCOL_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "rm_serial_driver/float_frame.hpp"
#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/transporter_interface.hpp"

namespace qd::serial_driver::protocol {

/** @brief 电控 pitch 绝对角到水平为零、抬头为正仰角的标定。 */
struct PitchCalibration {
    double horizontal_rad = 0.58; // 云台水平时的电控绝对角。
    double up_sign = -1.0; // 抬头时电控角度增大为 +1，减小为 -1。
    double min_rad = 0.4; // 电控允许的最小绝对角。
    double max_rad = 0.75; // 电控允许的最大绝对角。
};

/**
 * @brief 使用电控 Seasky 浮点帧收发云台角度及姿态。
 * @note 串口 pitch 为电机绝对角（弧度），yaw 用度；ROS 仰角以水平为零、抬头为正（度）。
 *       视觉发送 [pitch, yaw]；电控发送 [yaw, pitch] 或从 yaw 起连续 15 个 float。
 */
class SeaskyProtocol: public Protocol {
public:
    /**
     * @brief 创建并打开物理串口。
     * @param protocol_type 机器人类型，决定 ROS 订阅和服务客户端。
     * @param port_name 串口设备路径。
     * @param speed 波特率，单位 bit/s。
     * @param enable_data_print 是否每秒打印一次接收姿态和发送角度。
     * @param crc_profile 收发双方使用的 CRC 算法组合。
     * @param pitch_calibration 电控 pitch 的水平零位、方向及绝对角限位。
     */
    SeaskyProtocol(
        std::string_view protocol_type,
        std::string_view port_name,
        int speed,
        bool enable_data_print,
        CrcProfile crc_profile = CrcProfile::ROBOMASTER,
        PitchCalibration pitch_calibration = {}
    );

    /**
     * @brief 使用现有传输设备创建协议，供集成测试或设备复用。
     * @param protocol_type 机器人类型。
     * @param transporter 串口传输设备，由协议共享持有。
     * @param enable_data_print 是否每秒打印一次接收姿态和发送角度。
     * @param crc_profile 收发双方使用的 CRC 算法组合。
     * @param pitch_calibration 电控 pitch 的水平零位、方向及绝对角限位。
     */
    SeaskyProtocol(
        std::string_view protocol_type,
        std::shared_ptr<TransporterInterface> transporter,
        bool enable_data_print,
        CrcProfile crc_profile = CrcProfile::ROBOMASTER,
        PitchCalibration pitch_calibration = {}
    );

    /**
     * @brief 发送电控要求的 pitch、yaw 浮点数据。
     * @param data 视觉计算的云台指令，pitch/yaw 单位为度，pitch 抬头为正。
     * @note distance 小于 0 表示无有效目标，不发送角度指令。
     *       发送前 pitch 换算为电机绝对角并限制在标定范围内。
     */
    void send(const rm_interfaces::msg::GimbalCmd& data) override;

    /**
     * @brief 从串口流接收并解析电控姿态帧。
     * @param data 校验通过后写入云台姿态，角度单位为度，pitch 抬头为正。
     * @return 成功读取两个或 15 个 float 的姿态帧时返回 true；两 float 帧的 roll 设为 0。
     * @note 该函数会等待串口数据；ROS 关闭时返回 false。
     */
    bool receive(rm_interfaces::msg::SerialReceiveData& data) override;

    /**
     * @brief 创建当前机器人类型需要的云台指令订阅。
     * @param node 持有订阅的 ROS 节点。
     * @return ROS 订阅列表。
     */
    std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(rclcpp::Node::SharedPtr node
    ) override;

    /**
     * @brief 创建当前机器人类型需要的模式服务客户端。
     * @param node 持有客户端的 ROS 节点。
     * @return 服务客户端列表。
     */
    std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr>
    getClients(rclcpp::Node::SharedPtr node) const override;

    /**
     * @brief 获取最近一次串口错误。
     * @return 传输设备的错误文本。
     */
    std::string getErrorMessage() override;

private:
    std::string protocol_type_;
    std::shared_ptr<TransporterInterface> transporter_;
    CrcProfile crc_profile_;
    PitchCalibration pitch_calibration_;
    FloatFrame parser_;
    bool enable_data_print_;
    std::mutex send_mutex_;
    std::chrono::steady_clock::time_point last_data_print_ {};
    std::chrono::steady_clock::time_point last_send_print_ {};
};

} // namespace qd::serial_driver::protocol

#endif // RM_SERIAL_DRIVER_PROTOCOL_SEASKY_PROTOCOL_HPP_
