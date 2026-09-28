// Copyright 2021 RoboMaster-OSS
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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dummy_transporter.hpp"
#include "gtest/gtest.h"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"

using namespace qd;

namespace {

/** @brief 模拟需要先打开、再按指定大小分段读取的串口。 */
class ClosedTransporter: public serial_driver::TransporterInterface {
public:
    /**
     * @brief 预置接收帧及每次读取的最大字节数。
     * @param incoming 已准备好的原始接收字节。
     * @param chunks 连续读取时各次最多返回的字节数。
     */
    ClosedTransporter(
        std::vector<std::uint8_t> incoming = {},
        std::vector<std::size_t> chunks = {}
    ):
        incoming_(std::move(incoming)),
        chunks_(std::move(chunks)) {}

    bool open() override {
        ++open_calls;
        open_ = true;
        return true;
    }

    void close() override {
        ++close_calls;
        open_ = false;
    }

    bool isOpen() override {
        return open_;
    }

    int read(void* buffer, std::size_t len) override {
        ++read_calls;
        if (!open_ || offset_ == incoming_.size()) {
            return -1;
        }
        const auto chunk = chunk_index_ < chunks_.size() ? chunks_[chunk_index_++] : len;
        const auto count = std::min({ len, chunk, incoming_.size() - offset_ });
        std::memcpy(buffer, incoming_.data() + offset_, count);
        offset_ += count;
        return static_cast<int>(count);
    }

    int write(const void* buffer, std::size_t len) override {
        ++write_calls;
        if (!open_) {
            return -1;
        }
        const auto* data = static_cast<const std::uint8_t*>(buffer);
        sent.assign(data, data + len);
        return static_cast<int>(len);
    }

    std::string errorMessage() override {
        return "closed test transporter";
    }

    int open_calls = 0;
    int close_calls = 0;
    int read_calls = 0;
    int write_calls = 0;
    std::vector<std::uint8_t> sent;

private:
    bool open_ = false;
    std::size_t offset_ = 0;
    std::size_t chunk_index_ = 0;
    std::vector<std::uint8_t> incoming_;
    std::vector<std::size_t> chunks_;
};

/**
 * @brief 生成符合固定 16 字节包头和包尾要求的测试帧。
 * @return 带有已知数据字段的完整帧。
 */
std::vector<std::uint8_t> make_frame() {
    return { 0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
             0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x00, 0x0d };
}

} // namespace

TEST(FixedPacketTool, construct_with_nullptr) {
  EXPECT_THROW(serial_driver::FixedPacketTool<32>(nullptr), std::invalid_argument);
}

TEST(FixedPacketTool, send_and_recv) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  serial_driver::FixedPacket<32> packet1, packet2;
  // send
  int a = 10;
  packet1.loadData(a, 10);
  bool send_ret = packet_tool1->sendPacket(packet1);
  ASSERT_TRUE(send_ret);
  // recv
  int b;
  bool recv_ret = packet_tool2->recvPacket(packet2);
  ASSERT_TRUE(recv_ret);
  packet2.unloadData<int>(b, 10);
  EXPECT_EQ(a, b);
}

TEST(FixedPacketTool, realtime_send) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  packet_tool1->enbaleRealtimeSend(true);
  serial_driver::FixedPacket<32> packet1, packet2;
  // recv
  auto t = std::thread([&]() {
    int b;
    for (int i = 0; i < 10; i++) {
      bool recv_ret = packet_tool2->recvPacket(packet2);
      ASSERT_TRUE(recv_ret);
      packet2.unloadData<int>(b, 10);
      EXPECT_EQ(i, b);
    }
  });
  // send
  for (int i = 0; i < 10; i++) {
    packet1.loadData(i, 10);
    bool send_ret = packet_tool1->sendPacket(packet1);
    ASSERT_TRUE(send_ret);
  }
  t.join();
}

TEST(FixedPacketTool, opens_closed_transporter_before_first_send) {
    auto transporter = std::make_shared<ClosedTransporter>();
    serial_driver::FixedPacketTool<16> tool(transporter);
    EXPECT_EQ(transporter->open_calls, 1);
    EXPECT_TRUE(tool.isOpen());

    serial_driver::FixedPacket<16> packet;
    const auto frame = make_frame();
    packet.copyFrom(frame.data());
    EXPECT_TRUE(tool.sendPacket(packet));
    EXPECT_EQ(transporter->write_calls, 1);
    EXPECT_EQ(transporter->close_calls, 0);
    EXPECT_EQ(transporter->sent, frame);
}

TEST(FixedPacketTool, opens_closed_transporter_before_first_receive) {
    const auto frame = make_frame();
    auto transporter = std::make_shared<ClosedTransporter>(frame);
    serial_driver::FixedPacketTool<16> tool(transporter);
    EXPECT_EQ(transporter->open_calls, 1);

    serial_driver::FixedPacket<16> received;
    EXPECT_TRUE(tool.recvPacket(received));
    EXPECT_EQ(transporter->read_calls, 1);
    EXPECT_EQ(transporter->close_calls, 0);
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), received.buffer()));
}

TEST(FixedPacketTool, reassembles_fragmented_first_frame) {
    const auto frame = make_frame();
    auto transporter =
        std::make_shared<ClosedTransporter>(frame, std::vector<std::size_t> { 5, 11 });
    serial_driver::FixedPacketTool<16> tool(transporter);
    ASSERT_EQ(transporter->open_calls, 1);

    serial_driver::FixedPacket<16> received;
    EXPECT_FALSE(tool.recvPacket(received));
    EXPECT_TRUE(tool.recvPacket(received));
    EXPECT_EQ(transporter->read_calls, 2);
    EXPECT_EQ(transporter->close_calls, 0);
    EXPECT_TRUE(std::equal(frame.begin(), frame.end(), received.buffer()));
}
