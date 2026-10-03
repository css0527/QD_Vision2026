#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "gtest/gtest.h"

namespace {

constexpr std::uint8_t EXPECTED_FRAME[] {
    0xA5, 0x08, 0x00, 0xF4, 0x1C, 0xED, 0x04, 0x42, 0x00, 0x00, 0x20, 0xC0, 0xAB, 0x27,
};

constexpr std::uint8_t OTHER_VALID_FRAME[] {
    0xA5, 0x08, 0x00, 0xF4, 0x38, 0xDA, 0xE1, 0x41, 0x00, 0x00, 0x60, 0xC0, 0x8D, 0xFA,
};

constexpr std::uint8_t LEGACY_FRAME[] {
    0xA5, 0x08, 0x00, 0x67, 0x1C, 0xED, 0x04, 0x42, 0x00, 0x00, 0x20, 0xC0, 0x94, 0x90,
};

/** @brief 管理仅用于测试的伪终端两端。 */
class PseudoTerminal {
public:
    ~PseudoTerminal() {
        if (slave_fd_ >= 0) {
            close(slave_fd_);
        }
        if (master_fd_ >= 0) {
            close(master_fd_);
        }
    }

    /** @brief 建立伪终端并保留从端，以便进程退出后检查串口配置。 */
    bool open_pty() {
        master_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (master_fd_ < 0 || grantpt(master_fd_) != 0 || unlockpt(master_fd_) != 0) {
            return false;
        }
        char path[128] {};
        if (ptsname_r(master_fd_, path, sizeof(path)) != 0) {
            return false;
        }
        port_path_ = path;
        slave_fd_ = open(path, O_RDWR | O_NOCTTY);
        return slave_fd_ >= 0;
    }

    int master_fd() const {
        return master_fd_;
    }
    int slave_fd() const {
        return slave_fd_;
    }
    const std::string& port_path() const {
        return port_path_;
    }

private:
    int master_fd_ { -1 };
    int slave_fd_ { -1 };
    std::string port_path_;
};

/** @brief 管理被测程序及其标准输出管道。 */
class ProbeProcess {
public:
    ~ProbeProcess() {
        if (pid_ > 0) {
            kill(pid_, SIGKILL);
            waitpid(pid_, nullptr, 0);
        }
        if (output_fd_ >= 0) {
            close(output_fd_);
        }
    }

    /**
     * @brief 在子进程中运行验证工具。
     * @param args 除程序名以外的命令行参数。
     * @return 子进程和输出管道均建立成功时返回 true。
     */
    bool start(const std::vector<std::string>& args) {
        int pipe_fds[2] {};
        if (pipe(pipe_fds) != 0) {
            return false;
        }
        pid_ = fork();
        if (pid_ < 0) {
            close(pipe_fds[0]);
            close(pipe_fds[1]);
            return false;
        }
        if (pid_ == 0) {
            close(pipe_fds[0]);
            if (dup2(pipe_fds[1], STDOUT_FILENO) < 0 || dup2(pipe_fds[1], STDERR_FILENO) < 0) {
                _exit(127);
            }
            close(pipe_fds[1]);
            const char* executable = std::getenv("SERIAL_PROBE_EXE");
            if (executable == nullptr || executable[0] == '\0') {
                executable = "serial_probe";
            }
            std::vector<char*> argv { const_cast<char*>(executable) };
            for (const auto& arg: args) {
                argv.push_back(const_cast<char*>(arg.c_str()));
            }
            argv.push_back(nullptr);
            execvp(executable, argv.data());
            _exit(127);
        }
        close(pipe_fds[1]);
        output_fd_ = pipe_fds[0];
        return true;
    }

    /**
     * @brief 等待子进程结束，并读取其全部输出。
     * @param timeout_ms 等待上限，单位毫秒。
     * @param output 子进程退出后写入其标准输出和错误输出。
     * @return 子进程退出码；超时或异常时返回 -1。
     */
    int finish(int timeout_ms, std::string& output) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        int status = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto result = waitpid(pid_, &status, WNOHANG);
            if (result == pid_) {
                pid_ = -1;
                char buffer[512];
                ssize_t count = 0;
                while ((count = read(output_fd_, buffer, sizeof(buffer))) > 0) {
                    output.append(buffer, static_cast<std::size_t>(count));
                }
                return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            }
            if (result < 0 && errno != EINTR) {
                return -1;
            }
            usleep(5000);
        }
        return -1;
    }

private:
    pid_t pid_ { -1 };
    int output_fd_ { -1 };
};

/**
 * @brief 在时限内读取指定数量的原始串口字节。
 * @param fd 伪终端主端。
 * @param count 期望接收的字节数。
 * @return 收到的原始字节；超时则返回已有字节。
 */
std::vector<std::uint8_t> read_bytes(int fd, std::size_t count) {
    std::vector<std::uint8_t> bytes;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (bytes.size() < count && std::chrono::steady_clock::now() < deadline) {
        pollfd descriptor { fd, POLLIN, 0 };
        if (poll(&descriptor, 1, 100) <= 0) {
            continue;
        }
        std::uint8_t buffer[128];
        const auto received = read(fd, buffer, sizeof(buffer));
        if (received <= 0) {
            break;
        }
        bytes.insert(bytes.end(), buffer, buffer + received);
    }
    return bytes;
}

/**
 * @brief 生成单次发送 pitch 和 yaw 绝对角度数的测试参数，不做角度换算。
 * @param port 伪终端从端设备路径。
 * @return 供验证工具使用的参数序列。
 * @note pitch 为当前标定下的水平绝对角；探针直接发送参数原始值。
 */
std::vector<std::string> send_args(const std::string& port) {
    return { "--port", port, "--baud", "921600", "--values", "33.231552", "-2.5" };
}

TEST(SerialProbe, SendsDefaultWireUnitsAndDoesNotClaimDelivery) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    ProbeProcess process;
    ASSERT_TRUE(process.start({ "--port", pty.port_path() }));

    const auto bytes = read_bytes(pty.master_fd(), sizeof(EXPECTED_FRAME));
    std::string output;
    const auto exit_code = process.finish(2000, output);
    EXPECT_EQ(
        bytes,
        (std::vector<std::uint8_t>(std::begin(EXPECTED_FRAME), std::end(EXPECTED_FRAME)))
    );
    EXPECT_EQ(exit_code, 0) << output;
    EXPECT_NE(output.find("TX_FRAME (14 bytes)"), std::string::npos) << output;
    EXPECT_NE(output.find("PITCH_DEG=33.2316 YAW_DEG=-2.5"), std::string::npos) << output;
    EXPECT_NE(output.find("CRC_PROFILE=crc8_31_modbus"), std::string::npos) << output;
    EXPECT_NE(output.find("TX_WRITTEN"), std::string::npos) << output;
    EXPECT_NE(output.find("DELIVERY_UNCONFIRMED"), std::string::npos) << output;
    EXPECT_EQ(output.find("ECHO_CONFIRMED"), std::string::npos) << output;

    termios config {};
    ASSERT_EQ(tcgetattr(pty.slave_fd(), &config), 0);
    EXPECT_EQ(cfgetispeed(&config), B921600);
    EXPECT_EQ(cfgetospeed(&config), B921600);
    EXPECT_EQ(config.c_cflag & CSIZE, CS8);
    EXPECT_EQ(config.c_cflag & (PARENB | CSTOPB), 0);
#ifdef CRTSCTS
    EXPECT_EQ(config.c_cflag & CRTSCTS, 0);
#endif
    EXPECT_EQ(config.c_lflag & (ICANON | ECHO), 0);
    EXPECT_EQ(config.c_iflag & (IXON | IXOFF | ICRNL), 0);
    EXPECT_EQ(config.c_oflag & OPOST, 0);
}

TEST(SerialProbe, ConfirmsOnlyCompleteMatchingEcho) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    ProbeProcess process;
    auto args = send_args(pty.port_path());
    args.insert(args.end(), { "--expect-echo", "--timeout-ms", "500" });
    ASSERT_TRUE(process.start(args));
    const auto sent = read_bytes(pty.master_fd(), sizeof(EXPECTED_FRAME));
    ASSERT_EQ(
        sent,
        (std::vector<std::uint8_t>(std::begin(EXPECTED_FRAME), std::end(EXPECTED_FRAME)))
    );

    ASSERT_EQ(write(pty.master_fd(), EXPECTED_FRAME, 3), 3);
    usleep(10000);
    ASSERT_EQ(write(pty.master_fd(), EXPECTED_FRAME + 3, 5), 5);
    usleep(10000);
    ASSERT_EQ(
        write(pty.master_fd(), EXPECTED_FRAME + 8, sizeof(EXPECTED_FRAME) - 8),
        static_cast<ssize_t>(sizeof(EXPECTED_FRAME) - 8)
    );

    std::string output;
    EXPECT_EQ(process.finish(2000, output), 0) << output;
    EXPECT_NE(output.find("TX_WRITTEN"), std::string::npos) << output;
    EXPECT_NE(output.find("ECHO_CONFIRMED"), std::string::npos) << output;
}

TEST(SerialProbe, RejectsCorruptEcho) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    ProbeProcess process;
    auto args = send_args(pty.port_path());
    args.insert(args.end(), { "--expect-echo", "--timeout-ms", "200" });
    ASSERT_TRUE(process.start(args));
    ASSERT_EQ(read_bytes(pty.master_fd(), sizeof(EXPECTED_FRAME)).size(), sizeof(EXPECTED_FRAME));

    auto corrupt = std::vector<std::uint8_t>(std::begin(EXPECTED_FRAME), std::end(EXPECTED_FRAME));
    corrupt.back() ^= 1U;
    ASSERT_EQ(
        write(pty.master_fd(), corrupt.data(), corrupt.size()),
        static_cast<ssize_t>(corrupt.size())
    );

    std::string output;
    EXPECT_EQ(process.finish(2000, output), 2) << output;
    EXPECT_EQ(output.find("ECHO_CONFIRMED"), std::string::npos) << output;
}

TEST(SerialProbe, RejectsDifferentValidFrameAsEcho) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    ProbeProcess process;
    auto args = send_args(pty.port_path());
    args.insert(args.end(), { "--expect-echo", "--timeout-ms", "200" });
    ASSERT_TRUE(process.start(args));
    ASSERT_EQ(read_bytes(pty.master_fd(), sizeof(EXPECTED_FRAME)).size(), sizeof(EXPECTED_FRAME));
    ASSERT_EQ(
        write(pty.master_fd(), OTHER_VALID_FRAME, sizeof(OTHER_VALID_FRAME)),
        static_cast<ssize_t>(sizeof(OTHER_VALID_FRAME))
    );

    std::string output;
    EXPECT_EQ(process.finish(2000, output), 2) << output;
    EXPECT_EQ(output.find("ECHO_CONFIRMED"), std::string::npos) << output;
}

TEST(SerialProbe, SupportsLegacyCrcProfileForSendAndEcho) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    ProbeProcess process;
    auto args = send_args(pty.port_path());
    args.insert(
        args.end(),
        { "--crc-profile", "robomaster", "--expect-echo", "--timeout-ms", "500" }
    );
    ASSERT_TRUE(process.start(args));
    const auto sent = read_bytes(pty.master_fd(), sizeof(LEGACY_FRAME));
    ASSERT_EQ(sent, (std::vector<std::uint8_t>(std::begin(LEGACY_FRAME), std::end(LEGACY_FRAME))));
    ASSERT_EQ(
        write(pty.master_fd(), LEGACY_FRAME, sizeof(LEGACY_FRAME)),
        static_cast<ssize_t>(sizeof(LEGACY_FRAME))
    );
    std::string output;
    EXPECT_EQ(process.finish(2000, output), 0) << output;
    EXPECT_NE(output.find("CRC_PROFILE=robomaster"), std::string::npos) << output;
    EXPECT_NE(output.find("ECHO_CONFIRMED"), std::string::npos) << output;
}

TEST(SerialProbe, RejectsEchoUsingDifferentCrcProfile) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    ProbeProcess process;
    auto args = send_args(pty.port_path());
    args.insert(args.end(), { "--expect-echo", "--timeout-ms", "200" });
    ASSERT_TRUE(process.start(args));
    ASSERT_EQ(read_bytes(pty.master_fd(), sizeof(EXPECTED_FRAME)).size(), sizeof(EXPECTED_FRAME));
    ASSERT_EQ(
        write(pty.master_fd(), LEGACY_FRAME, sizeof(LEGACY_FRAME)),
        static_cast<ssize_t>(sizeof(LEGACY_FRAME))
    );
    std::string output;
    EXPECT_EQ(process.finish(2000, output), 2) << output;
    EXPECT_EQ(output.find("ECHO_CONFIRMED"), std::string::npos) << output;
}

TEST(SerialProbe, RejectsUnknownCrcProfileBeforeOpeningPort) {
    ProbeProcess process;
    ASSERT_TRUE(process.start({ "--port", "/invalid/device", "--crc-profile", "auto" }));
    std::string output;
    EXPECT_EQ(process.finish(2000, output), 1) << output;
    EXPECT_NE(output.find("未知 CRC 配置"), std::string::npos) << output;
    EXPECT_EQ(output.find("打开串口失败"), std::string::npos) << output;
}

TEST(SerialProbe, RejectsInvalidValueBeforeConfiguringPort) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    termios before {};
    ASSERT_EQ(tcgetattr(pty.slave_fd(), &before), 0);
    ProbeProcess process;
    ASSERT_TRUE(process.start({ "--port", pty.port_path(), "--values", "nan" }));

    std::string output;
    EXPECT_EQ(process.finish(2000, output), 1) << output;
    termios after {};
    ASSERT_EQ(tcgetattr(pty.slave_fd(), &after), 0);
    EXPECT_EQ(before.c_iflag, after.c_iflag);
    EXPECT_EQ(before.c_oflag, after.c_oflag);
    EXPECT_EQ(before.c_cflag, after.c_cflag);
    EXPECT_EQ(before.c_lflag, after.c_lflag);
    EXPECT_EQ(cfgetispeed(&before), cfgetispeed(&after));
    EXPECT_EQ(cfgetospeed(&before), cfgetospeed(&after));
    for (std::size_t i = 0; i < NCCS; ++i) {
        EXPECT_EQ(before.c_cc[i], after.c_cc[i]);
    }
    pollfd descriptor { pty.master_fd(), POLLIN, 0 };
    EXPECT_EQ(poll(&descriptor, 1, 0), 0);
}

TEST(SerialProbe, RejectsThreeValuesBeforeOpeningPort) {
    PseudoTerminal pty;
    ASSERT_TRUE(pty.open_pty());
    termios before {};
    ASSERT_EQ(tcgetattr(pty.slave_fd(), &before), 0);
    ProbeProcess process;
    ASSERT_TRUE(process.start(
        { "--port", pty.port_path(), "--values", "33.231552", "-2.5", "3.25" }
    ));

    std::string output;
    EXPECT_EQ(process.finish(2000, output), 1) << output;
    EXPECT_NE(output.find("--values 需要恰好 2 个有限浮点数"), std::string::npos) << output;
    EXPECT_EQ(output.find("TX_FRAME"), std::string::npos) << output;

    termios after {};
    ASSERT_EQ(tcgetattr(pty.slave_fd(), &after), 0);
    EXPECT_EQ(before.c_iflag, after.c_iflag);
    EXPECT_EQ(before.c_oflag, after.c_oflag);
    EXPECT_EQ(before.c_cflag, after.c_cflag);
    EXPECT_EQ(before.c_lflag, after.c_lflag);
    EXPECT_EQ(cfgetispeed(&before), cfgetispeed(&after));
    EXPECT_EQ(cfgetospeed(&before), cfgetospeed(&after));
    for (std::size_t i = 0; i < NCCS; ++i) {
        EXPECT_EQ(before.c_cc[i], after.c_cc[i]);
    }
    pollfd descriptor { pty.master_fd(), POLLIN, 0 };
    EXPECT_EQ(poll(&descriptor, 1, 0), 0);
}

} // namespace
