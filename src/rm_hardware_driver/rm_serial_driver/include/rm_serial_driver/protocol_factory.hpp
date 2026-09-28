// Created by Chengfu Zou on 2023.7.6
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
#define SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_

#include <memory>
#include <string_view>

#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol/default_protocol.hpp"
#include "rm_serial_driver/protocol/infantry_protocol.hpp"
#include "rm_serial_driver/protocol/seasky_protocol.hpp"
#include "rm_serial_driver/protocol/sentry_protocol.hpp"

namespace qd::serial_driver {

class ProtocolFactory {
public:
    ProtocolFactory() = delete;
    /**
     * @brief 根据机器人类型创建串口协议并应用角度标定。
     * @param protocol_type 机器人类型，决定订阅和模式服务。
     * @param port_name 串口设备路径。
     * @param speed 波特率，单位 bit/s。
     * @param enable_data_print 是否打印收发角度。
     * @param crc_profile 收发双方约定的 CRC 算法组合。
     * @param pitch_calibration 电控 pitch 的水平零位、方向和绝对角限位。
     * @return 对应的协议实例；不支持的机器人类型返回空指针。
     */
    static std::unique_ptr<protocol::Protocol> createProtocol(
        std::string_view protocol_type,
        std::string_view port_name,
        int speed,
        bool enable_data_print,
        CrcProfile crc_profile = CrcProfile::ROBOMASTER,
        protocol::PitchCalibration pitch_calibration = {}
    ) {
        if (protocol_type == "infantry" || protocol_type == "hero" || protocol_type == "air"
            || protocol_type == "sentry" || protocol_type == "seasky")
        {
            return std::make_unique<protocol::SeaskyProtocol>(
                protocol_type,
                port_name,
                speed,
                enable_data_print,
                crc_profile,
                pitch_calibration
            );
        }
        return nullptr;
    }
};

}; // namespace qd::serial_driver
#endif // SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
