#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "rm_serial_driver/float_frame.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using qd::serial_driver::CrcProfile;
using qd::serial_driver::FloatFrame;

struct Options {
    std::string port = "/dev/ttyACM0";
    int baud = 921600;
    CrcProfile crc_profile = CrcProfile::CRC8_31_MODBUS;
    int count = 1;
    int interval_ms = 1000;
    int timeout_ms = 1000;
    bool expect_echo = false;
    bool dry_run = false;
    bool help = false;
    std::vector<float> values { 33.231552F, -2.5F };
};

/**
 * @brief 将支持的数值波特率转换为 termios 速率。
 * @param baud 串口波特率，单位 bit/s。
 * @return termios 对应的速率常量；不支持的速率抛出异常。
 */
speed_t baud_speed(int baud) {
    switch (baud) {
        case 921600:
            return B921600;
        case 115200:
            return B115200;
        case 19200:
            return B19200;
        case 9600:
            return B9600;
        case 4800:
            return B4800;
        case 2400:
            return B2400;
        case 1200:
            return B1200;
        case 300:
            return B300;
        default:
            throw std::invalid_argument("不支持的波特率: " + std::to_string(baud));
    }
}

/**
 * @brief 解析并检查命令行中的非负整数。
 * @param value 原始参数文本。
 * @param maximum 允许的最大值，含边界。
 * @return 解析出的非负整数；格式错误或越界时抛出异常。
 */
int parse_integer(const std::string& value, int maximum) {
    if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
        throw std::invalid_argument("无效整数: " + value);
    }
    std::size_t consumed = 0;
    const auto parsed = std::stoul(value, &consumed);
    if (consumed != value.size() || parsed > static_cast<unsigned long>(maximum)) {
        throw std::invalid_argument("整数超出范围: " + value);
    }
    return static_cast<int>(parsed);
}

/**
 * @brief 在打开设备前解析和验证全部参数。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数列表。
 * @return 已验证的工具配置；参数不合法时抛出异常。
 */
Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--help" || argument == "-h") {
            options.help = true;
        } else if (argument == "--expect-echo") {
            options.expect_echo = true;
        } else if (argument == "--dry-run") {
            options.dry_run = true;
        } else if (argument == "--values") {
            options.values.clear();
            while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                const std::string value = argv[++i];
                std::size_t consumed = 0;
                const auto parsed = std::stof(value, &consumed);
                if (consumed != value.size() || !std::isfinite(parsed)) {
                    throw std::invalid_argument("无效浮点数: " + value);
                }
                options.values.push_back(parsed);
            }
            if (options.values.size() != 2) {
                throw std::invalid_argument("--values 需要恰好 2 个有限浮点数：PITCH_DEG YAW_DEG");
            }
        } else if (argument == "--port" || argument == "--baud" || argument == "--count" || argument == "--interval-ms" || argument == "--timeout-ms" || argument == "--crc-profile")
        {
            if (i + 1 >= argc) {
                throw std::invalid_argument("缺少参数值: " + argument);
            }
            const std::string value = argv[++i];
            if (argument == "--port") {
                options.port = value;
            } else if (argument == "--crc-profile") {
                options.crc_profile = FloatFrame::parse_crc_profile(value);
            } else if (argument == "--baud") {
                options.baud = parse_integer(value, 921600);
            } else if (argument == "--count") {
                options.count = parse_integer(value, 10000);
            } else if (argument == "--interval-ms") {
                options.interval_ms = parse_integer(value, 60000);
            } else {
                options.timeout_ms = parse_integer(value, 60000);
            }
        } else {
            throw std::invalid_argument("未知参数: " + argument);
        }
    }
    if (options.port.empty() || options.count == 0 || options.timeout_ms == 0) {
        throw std::invalid_argument("设备名不能为空，发送次数和超时必须大于 0");
    }
    baud_speed(options.baud);
    return options;
}

/** @brief 输出命令行使用说明。 */
void print_help() {
    std::cout << "用法: serial_probe [选项]\n"
                 "  --port DEVICE       串口设备，默认 /dev/ttyACM0\n"
                 "  --baud RATE         波特率，默认 921600，固定 8N1、无流控\n"
                 "  --crc-profile NAME  crc8_31_modbus（默认）或 robomaster\n"
                 "  --values PITCH_DEG YAW_DEG  原样发送两个 float，默认 33.231552 deg（水平） -2.5 deg\n"
                 "  --count N           发送次数，默认 1，最大 10000\n"
                 "  --interval-ms MS    两次发送的最小间隔，默认 1000\n"
                 "  --timeout-ms MS     每次发送后等待接收的时限，默认 1000\n"
                 "  --expect-echo       要求电控回传相同的有效帧，未收到则退出 2\n"
                 "  --dry-run           只打印帧，不打开串口\n"
                 "crc8_31_modbus: CRC8 init=00 poly=31 (不反射); "
                 "CRC16 init=FFFF poly=A001 (反射)\n"
                 "robomaster: CRC8 init=FF poly=8C (反射); "
                 "CRC16 init=FFFF poly=8408 (反射)\n"
                 "以上多项式按计算方向表示；所有 CRC 均无异或输出。\n"
                 "TX_WRITTEN 仅说明本机已写入；电控解析返回 1 或原帧回显才可确认。\n";
}

/**
 * @brief 打印串口原始字节的十六进制表示。
 * @param label 用于区分发送与接收的标签。
 * @param data 原始字节地址。
 * @param length 字节数量。
 */
void print_bytes(const std::string& label, const std::uint8_t* data, std::size_t length) {
    std::cout << label << " (" << length << " bytes):" << std::hex << std::uppercase
              << std::setfill('0');
    for (std::size_t i = 0; i < length; ++i) {
        std::cout << ' ' << std::setw(2) << static_cast<unsigned int>(data[i]);
    }
    std::cout << std::dec << std::nouppercase << std::setfill(' ') << std::endl;
}

/** @brief 管理验证串口，并在工具退出时关闭设备和解除独占打开限制。 */
class ProbePort {
public:
    ProbePort() = default;
    ProbePort(const ProbePort&) = delete;
    ProbePort& operator=(const ProbePort&) = delete;

    ~ProbePort() {
        if (fd_ >= 0) {
            ioctl(fd_, TIOCNXCL);
            close(fd_);
        }
    }

    /**
     * @brief 打开验证设备并设置 8N1 原始通信模式。
     * @param path 串口设备路径。
     * @param baud 串口波特率，单位 bit/s。
     * @note 运行前需停止占用同一设备的串口节点；失败时抛出异常。
     */
    void open_port(const std::string& path, int baud) {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            throw std::runtime_error("打开串口失败: " + path + ": " + std::strerror(errno));
        }
        if (ioctl(fd_, TIOCEXCL) != 0) {
            throw std::runtime_error("设置串口独占失败: " + std::string(std::strerror(errno)));
        }
        termios config {};
        if (tcgetattr(fd_, &config) != 0) {
            throw std::runtime_error("读取串口配置失败: " + std::string(std::strerror(errno)));
        }
        cfmakeraw(&config);
        config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        config.c_cflag |= CS8 | CLOCAL | CREAD;
        config.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | IGNCR | ICRNL);
        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 0;
        const auto speed = baud_speed(baud);
        if (cfsetispeed(&config, speed) != 0 || cfsetospeed(&config, speed) != 0
            || tcsetattr(fd_, TCSANOW, &config) != 0 || tcflush(fd_, TCIFLUSH) != 0)
        {
            throw std::runtime_error("设置串口配置失败: " + std::string(std::strerror(errno)));
        }
    }

    /**
     * @brief 在指定时限内将整帧写入串口。
     * @param frame 包含校验码的完整发送帧。
     * @param timeout_ms 完成写入的最大等待时间，单位毫秒。
     * @note 处理短写和中断；失败时抛出异常，成功不代表电控已接收。
     */
    void write_frame(const std::vector<std::uint8_t>& frame, int timeout_ms) const {
        const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
        std::size_t written = 0;
        while (written < frame.size()) {
            if (!wait_ready(POLLOUT, deadline)) {
                throw std::runtime_error(
                    "串口写入超时，已写入 " + std::to_string(written) + "/"
                    + std::to_string(frame.size()) + " 字节"
                );
            }
            const auto result = write(fd_, frame.data() + written, frame.size() - written);
            if (result > 0) {
                written += static_cast<std::size_t>(result);
            } else if (result == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
                throw std::runtime_error("串口写入失败: " + std::string(std::strerror(errno)));
            }
        }
    }

    /**
     * @brief 在时限内接收一批字节。
     * @param buffer 接收结果的写入地址。
     * @param length 可写入的最大字节数。
     * @param deadline 等待截止时间，使用单调时钟。
     * @return 实际接收的字节数；超时返回 0，错误时抛出异常。
     */
    std::size_t
    read_bytes(std::uint8_t* buffer, std::size_t length, Clock::time_point deadline) const {
        while (wait_ready(POLLIN, deadline)) {
            const auto result = read(fd_, buffer, length);
            if (result > 0) {
                return static_cast<std::size_t>(result);
            }
            if (result == 0) {
                throw std::runtime_error("串口连接已断开");
            }
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                throw std::runtime_error("串口读取失败: " + std::string(std::strerror(errno)));
            }
        }
        return 0;
    }

private:
    /**
     * @brief 在时限内等待串口可读或可写。
     * @param events poll 所需的事件标志。
     * @param deadline 等待截止时间，使用单调时钟。
     * @return 事件到达时返回 true；超时返回 false，设备错误时抛出异常。
     */
    bool wait_ready(short events, Clock::time_point deadline) const {
        while (Clock::now() < deadline) {
            const auto remaining =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now())
                    .count();
            pollfd descriptor { fd_, events, 0 };
            const auto result = poll(&descriptor, 1, static_cast<int>(remaining) + 1);
            if (result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw std::runtime_error("等待串口事件失败: " + std::string(std::strerror(errno)));
            }
            if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                throw std::runtime_error("串口连接异常或已断开");
            }
            if ((descriptor.revents & events) != 0) {
                return true;
            }
        }
        return false;
    }

    int fd_ = -1;
};

/**
 * @brief 观察电控回传的数据并检查是否为完整的相同帧。
 * @param port 已配置的串口。
 * @param expected 本次发送的完整帧。
 * @param timeout_ms 等待接收的上限，单位毫秒。
 * @param profile 与发送帧一致的 CRC 参数组合。
 * @return 收到校验通过且与发送内容一致的帧时返回 true。
 * @note 普通遥测数据、损坏帧和部分回传均不视为确认。
 */
bool observe_reply(
    const ProbePort& port,
    const std::vector<std::uint8_t>& expected,
    int timeout_ms,
    CrcProfile profile
) {
    FloatFrame parser(profile);
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    std::uint8_t buffer[512] {};
    std::vector<float> values;
    while (Clock::now() < deadline) {
        const auto received = port.read_bytes(buffer, sizeof(buffer), deadline);
        if (received == 0) {
            break;
        }
        print_bytes("RX", buffer, received);
        parser.append(buffer, received);
        while (parser.next(values)) {
            if (FloatFrame::encode(values, profile) == expected) {
                return true;
            }
            std::cout << "RX_VALID_FRAME: 内容与本次测试帧不同，不能作为确认。" << std::endl;
        }
    }
    return false;
}

} // namespace

/**
 * @brief 发送指定浮点测试帧，并报告本机写入及可选的电控回显结果。
 * @param argc 命令行参数数量。
 * @param argv 命令行参数列表。
 * @return 0 表示写入成功或所要求的回显成功；1 表示参数/串口错误；2 表示未收到所要求的回显。
 */
int main(int argc, char** argv) {
    try {
        const auto options = parse_options(argc, argv);
        if (options.help) {
            print_help();
            return 0;
        }
        const auto frame = FloatFrame::encode(options.values, options.crc_profile);
        std::cout << "PORT=" << options.port << " BAUD=" << options.baud << " 8N1\n"
                  << "PITCH_DEG=" << options.values[0] << " YAW_DEG=" << options.values[1];
        if (options.crc_profile == CrcProfile::CRC8_31_MODBUS) {
            std::cout << "\nCRC_PROFILE=crc8_31_modbus\n"
                      << "CRC8 init=00 poly=31 (不反射); CRC16 init=FFFF poly=A001 (反射)\n";
        } else {
            std::cout << "\nCRC_PROFILE=robomaster\n"
                      << "CRC8 init=FF poly=8C (反射); CRC16 init=FFFF poly=8408 (反射)\n";
        }
        std::cout << "CRC 无异或输出，多项式按计算方向表示。\n";
        print_bytes("TX_FRAME", frame.data(), frame.size());
        if (options.dry_run) {
            std::cout << "DRY_RUN: 未打开串口，未发送数据。" << std::endl;
            return 0;
        }
        ProbePort port;
        port.open_port(options.port, options.baud);
        bool all_confirmed = true;
        for (int i = 0; i < options.count; ++i) {
            const auto start = Clock::now();
            port.write_frame(frame, options.timeout_ms);
            std::cout << "TX_WRITTEN #" << i + 1 << ": " << frame.size() << '/' << frame.size()
                      << " bytes（本机写入成功）" << std::endl;
            const bool echoed = observe_reply(port, frame, options.timeout_ms, options.crc_profile);
            if (echoed) {
                std::cout << "ECHO_CONFIRMED #" << i + 1 << ": 已收到内容一致且 CRC 有效的回显帧。"
                          << std::endl;
            } else {
                all_confirmed = false;
                std::cout << "DELIVERY_UNCONFIRMED #" << i + 1
                          << ": 未收到匹配回显，请在电控查看 get_protocol_info() 返回值和数据。"
                          << std::endl;
            }
            if (i + 1 < options.count) {
                std::this_thread::sleep_until(
                    start + std::chrono::milliseconds(options.interval_ms)
                );
            }
        }
        return options.expect_echo && !all_confirmed ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n' << "使用 --help 查看参数。" << std::endl;
        return 1;
    }
}
