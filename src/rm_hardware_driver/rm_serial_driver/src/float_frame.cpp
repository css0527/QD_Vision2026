#include "rm_serial_driver/float_frame.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace qd::serial_driver {
namespace {
    constexpr std::uint8_t SOF = 0xA5;
    constexpr std::size_t FLOAT_SIZE = sizeof(float);
    static_assert(sizeof(float) == sizeof(std::uint32_t), "协议要求 32 位 float");
} // namespace

FloatFrame::FloatFrame(CrcProfile profile): profile_(profile) {}

CrcProfile FloatFrame::parse_crc_profile(std::string_view name) {
    if (name == "robomaster") {
        return CrcProfile::ROBOMASTER;
    }
    if (name == "crc8_31_modbus") {
        return CrcProfile::CRC8_31_MODBUS;
    }
    throw std::invalid_argument("未知 CRC 配置: " + std::string(name));
}

std::uint8_t FloatFrame::crc8(const std::uint8_t* data, std::size_t length, CrcProfile profile) {
    const bool msb_first = profile == CrcProfile::CRC8_31_MODBUS;
    std::uint8_t crc = msb_first ? 0x00 : 0xFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (msb_first) {
                // CRC8：初值 00，正向多项式 31，不反射，无结果异或。
                crc = (crc & 0x80U) != 0U ? static_cast<std::uint8_t>((crc << 1U) ^ 0x31U)
                                          : static_cast<std::uint8_t>(crc << 1U);
            } else {
                // RoboMaster：初值 FF，反射多项式 8C，无结果异或。
                crc = (crc & 1U) != 0U ? static_cast<std::uint8_t>((crc >> 1U) ^ 0x8CU)
                                       : static_cast<std::uint8_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

std::uint16_t FloatFrame::crc16(const std::uint8_t* data, std::size_t length, CrcProfile profile) {
    // 两组均为初值 FFFF、反射、无结果异或，校验结果按低字节在前发送。
    const std::uint16_t polynomial = profile == CrcProfile::CRC8_31_MODBUS ? 0xA001U : 0x8408U;
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ polynomial)
                                   : static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

std::vector<std::uint8_t> FloatFrame::encode(const std::vector<float>& values, CrcProfile profile) {
    if (values.size() > MAX_FLOAT_COUNT) {
        return {};
    }
    const auto data_length = static_cast<std::uint16_t>(values.size() * FLOAT_SIZE);
    std::vector<std::uint8_t> frame(HEADER_SIZE + data_length + TAIL_SIZE);
    frame[0] = SOF;
    frame[1] = static_cast<std::uint8_t>(data_length & 0xFFU);
    frame[2] = static_cast<std::uint8_t>(data_length >> 8U);
    frame[3] = crc8(frame.data(), HEADER_SIZE - 1, profile);
    for (std::size_t i = 0; i < values.size(); ++i) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &values[i], FLOAT_SIZE);
        const auto offset = HEADER_SIZE + i * FLOAT_SIZE;
        for (std::size_t byte = 0; byte < FLOAT_SIZE; ++byte) {
            frame[offset + byte] = static_cast<std::uint8_t>(bits >> (byte * 8U));
        }
    }
    const auto check = crc16(frame.data(), frame.size() - TAIL_SIZE, profile);
    frame[frame.size() - 2] = static_cast<std::uint8_t>(check & 0xFFU);
    frame[frame.size() - 1] = static_cast<std::uint8_t>(check >> 8U);
    return frame;
}

bool FloatFrame::decode(
    const std::vector<std::uint8_t>& frame,
    std::vector<float>& values,
    CrcProfile profile
) {
    if (frame.size() < HEADER_SIZE + TAIL_SIZE || frame[0] != SOF
        || frame[3] != crc8(frame.data(), HEADER_SIZE - 1, profile))
    {
        return false;
    }
    const auto data_length =
        static_cast<std::size_t>(frame[1]) | (static_cast<std::size_t>(frame[2]) << 8U);
    if (data_length > MAX_FLOAT_COUNT * FLOAT_SIZE || data_length % FLOAT_SIZE != 0
        || frame.size() != HEADER_SIZE + data_length + TAIL_SIZE)
    {
        return false;
    }
    const auto check = crc16(frame.data(), frame.size() - TAIL_SIZE, profile);
    if (frame[frame.size() - 2] != static_cast<std::uint8_t>(check & 0xFFU)
        || frame[frame.size() - 1] != static_cast<std::uint8_t>(check >> 8U))
    {
        return false;
    }
    std::vector<float> decoded(data_length / FLOAT_SIZE);
    for (std::size_t i = 0; i < decoded.size(); ++i) {
        const auto offset = HEADER_SIZE + i * FLOAT_SIZE;
        std::uint32_t bits = 0;
        for (std::size_t byte = 0; byte < FLOAT_SIZE; ++byte) {
            bits |= static_cast<std::uint32_t>(frame[offset + byte]) << (byte * 8U);
        }
        std::memcpy(&decoded[i], &bits, FLOAT_SIZE);
    }
    values = std::move(decoded);
    return true;
}

void FloatFrame::append(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr || length == 0) {
        return;
    }
    buffer_.insert(buffer_.end(), data, data + length);
}

bool FloatFrame::next(std::vector<float>& values) {
    while (!buffer_.empty()) {
        const auto sof = std::find(buffer_.begin(), buffer_.end(), SOF);
        buffer_.erase(buffer_.begin(), sof);
        if (buffer_.size() < HEADER_SIZE) {
            return false;
        }
        if (buffer_[3] != crc8(buffer_.data(), HEADER_SIZE - 1, profile_)) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        const auto data_length =
            static_cast<std::size_t>(buffer_[1]) | (static_cast<std::size_t>(buffer_[2]) << 8U);
        if (data_length > MAX_FLOAT_COUNT * FLOAT_SIZE || data_length % FLOAT_SIZE != 0) {
            buffer_.erase(buffer_.begin());
            continue;
        }
        const auto frame_length = HEADER_SIZE + data_length + TAIL_SIZE;
        if (buffer_.size() < frame_length) {
            return false;
        }
        std::vector<std::uint8_t> frame(buffer_.begin(), buffer_.begin() + frame_length);
        if (decode(frame, values, profile_)) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + frame_length);
            return true;
        }
        buffer_.erase(buffer_.begin());
    }
    return false;
}

} // namespace qd::serial_driver
