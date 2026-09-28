#ifndef RM_SERIAL_DRIVER_FLOAT_FRAME_HPP_
#define RM_SERIAL_DRIVER_FLOAT_FRAME_HPP_

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace qd::serial_driver {

/** @brief 双端显式约定的 CRC 参数组合。 */
enum class CrcProfile { ROBOMASTER, CRC8_31_MODBUS };

/**
 * @brief 编解码无命令 ID 的浮点数串口帧，并从字节流中恢复完整帧。
 * @note 帧格式为 A5、两字节数据长度、CRC8、浮点数据、CRC16；多字节字段均为小端。
 */
class FloatFrame {
public:
    static constexpr std::size_t MAX_FLOAT_COUNT = 64;
    static constexpr std::size_t HEADER_SIZE = 4;
    static constexpr std::size_t TAIL_SIZE = 2;

    /**
     * @brief 创建使用指定 CRC 参数的流解析器。
     * @param profile 接收端采用的 CRC 参数组合。
     */
    explicit FloatFrame(CrcProfile profile = CrcProfile::ROBOMASTER);

    /**
     * @brief 解析配置中的 CRC 参数名称。
     * @param name robomaster 或 crc8_31_modbus，区分大小写。
     * @return 对应的参数组合；名称无效时抛出 std::invalid_argument。
     */
    static CrcProfile parse_crc_profile(std::string_view name);

    /**
     * @brief 将浮点数组编码为串口帧。
     * @param values 按协议字段顺序排列的浮点数，最多 64 个。
     * @param profile 发送端采用的 CRC 参数组合。
     * @return 包含帧头和校验码的完整字节序列；超过上限时返回空数组。
     */
    static std::vector<std::uint8_t>
    encode(const std::vector<float>& values, CrcProfile profile = CrcProfile::ROBOMASTER);

    /**
     * @brief 验证完整帧并提取浮点数组。
     * @param frame 完整的串口帧。
     * @param values 校验通过后写入解析结果；失败时保持原值。
     * @param profile 接收端采用的 CRC 参数组合。
     * @return 帧头、长度、CRC8 和 CRC16 均有效时返回 true。
     */
    static bool decode(
        const std::vector<std::uint8_t>& frame,
        std::vector<float>& values,
        CrcProfile profile = CrcProfile::ROBOMASTER
    );

    /**
     * @brief 将新收到的字节追加到接收缓存。
     * @param data 字节数据地址。
     * @param length 可读取的字节数。
     */
    void append(const std::uint8_t* data, std::size_t length);

    /**
     * @brief 从接收缓存取出一个通过校验的完整帧。
     * @param values 成功时写入帧中的浮点数。
     * @return 找到完整有效帧时返回 true；数据不足时返回 false。
     */
    bool next(std::vector<float>& values);

private:
    static std::uint8_t crc8(const std::uint8_t* data, std::size_t length, CrcProfile profile);
    static std::uint16_t crc16(const std::uint8_t* data, std::size_t length, CrcProfile profile);

    CrcProfile profile_;
    std::vector<std::uint8_t> buffer_;
};

} // namespace qd::serial_driver

#endif // RM_SERIAL_DRIVER_FLOAT_FRAME_HPP_
