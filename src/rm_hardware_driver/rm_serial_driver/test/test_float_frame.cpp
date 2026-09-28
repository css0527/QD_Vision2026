#include <cstdint>
#include <stdexcept>
#include <vector>

#include "rm_serial_driver/float_frame.hpp"
#include "gtest/gtest.h"

namespace qd::serial_driver {
namespace {
    // 用户实板日志中的完整上行帧，作为独立于编码器的回归向量。
    const std::vector<std::uint8_t> CAPTURED_FRAME {
        0xA5, 0x08, 0x00, 0xF4, 0x00, 0x00, 0xEC, 0x42, 0xE1, 0x7A, 0x14, 0x3F, 0xC1, 0x3D,
    };
} // namespace

TEST(FloatFrame, RoundTripWithoutIdsOrFlags) {
    const std::vector<float> input { 1.0F, -2.5F, 0.0F };
    const auto frame = FloatFrame::encode(input);
    ASSERT_EQ(
        frame.size(),
        FloatFrame::HEADER_SIZE + input.size() * sizeof(float) + FloatFrame::TAIL_SIZE
    );
    EXPECT_EQ(frame[0], 0xA5);
    EXPECT_EQ(frame[1], 12);
    EXPECT_EQ(frame[2], 0);
    EXPECT_EQ(frame[4], 0);
    EXPECT_EQ(frame[5], 0);
    EXPECT_EQ(frame[6], 0x80);
    EXPECT_EQ(frame[7], 0x3F);
    std::vector<float> decoded;
    ASSERT_TRUE(FloatFrame::decode(frame, decoded));
    EXPECT_EQ(decoded, input);
}

TEST(FloatFrame, RejectsBadChecksumsAndLengths) {
    const auto frame = FloatFrame::encode({ 1.0F });
    std::vector<float> decoded { 42.0F };
    auto bad = frame;
    bad[3] ^= 1U;
    EXPECT_FALSE(FloatFrame::decode(bad, decoded));
    bad = frame;
    bad[4] ^= 1U;
    EXPECT_FALSE(FloatFrame::decode(bad, decoded));
    bad = frame;
    bad[1] = 3;
    EXPECT_FALSE(FloatFrame::decode(bad, decoded));
    EXPECT_EQ(decoded, std::vector<float> { 42.0F });
}

TEST(FloatFrame, RecoversFragmentedAndCorruptStream) {
    const auto first = FloatFrame::encode({ 1.0F });
    const auto second = FloatFrame::encode({ -3.0F, 4.0F });
    auto corrupt = first;
    corrupt.back() ^= 1U;
    FloatFrame parser;
    std::vector<float> decoded;
    const std::uint8_t noise[] { 0x11, 0x22 };
    parser.append(noise, sizeof(noise));
    parser.append(corrupt.data(), corrupt.size());
    parser.append(first.data(), 2);
    EXPECT_FALSE(parser.next(decoded));
    parser.append(first.data() + 2, first.size() - 2);
    parser.append(second.data(), second.size());
    ASSERT_TRUE(parser.next(decoded));
    EXPECT_EQ(decoded, std::vector<float> { 1.0F });
    ASSERT_TRUE(parser.next(decoded));
    EXPECT_EQ(decoded, (std::vector<float> { -3.0F, 4.0F }));
    EXPECT_FALSE(parser.next(decoded));
}

TEST(FloatFrame, ParsesOnlySupportedCrcProfiles) {
    EXPECT_EQ(FloatFrame::parse_crc_profile("robomaster"), CrcProfile::ROBOMASTER);
    EXPECT_EQ(FloatFrame::parse_crc_profile("crc8_31_modbus"), CrcProfile::CRC8_31_MODBUS);
    EXPECT_THROW(FloatFrame::parse_crc_profile(""), std::invalid_argument);
    EXPECT_THROW(FloatFrame::parse_crc_profile("auto"), std::invalid_argument);
}

TEST(FloatFrame, DecodesAndReproducesCapturedMcuFrame) {
    std::vector<float> values;
    ASSERT_TRUE(FloatFrame::decode(CAPTURED_FRAME, values, CrcProfile::CRC8_31_MODBUS));
    ASSERT_EQ(values.size(), 2U);
    EXPECT_FLOAT_EQ(values[0], 118.0F);
    EXPECT_FLOAT_EQ(values[1], 0.58F);
    EXPECT_EQ(FloatFrame::encode({ 118.0F, 0.58F }, CrcProfile::CRC8_31_MODBUS), CAPTURED_FRAME);
}

TEST(FloatFrame, RejectsCapturedFrameWithWrongProfileOrCorruption) {
    std::vector<float> values { 42.0F };
    EXPECT_FALSE(FloatFrame::decode(CAPTURED_FRAME, values));
    auto bad_header = CAPTURED_FRAME;
    bad_header[3] ^= 1U;
    EXPECT_FALSE(FloatFrame::decode(bad_header, values, CrcProfile::CRC8_31_MODBUS));
    auto bad_payload = CAPTURED_FRAME;
    bad_payload[4] ^= 1U;
    EXPECT_FALSE(FloatFrame::decode(bad_payload, values, CrcProfile::CRC8_31_MODBUS));
    auto bad_tail = CAPTURED_FRAME;
    bad_tail.back() ^= 1U;
    EXPECT_FALSE(FloatFrame::decode(bad_tail, values, CrcProfile::CRC8_31_MODBUS));
    const auto legacy = FloatFrame::encode({ 118.0F, 0.58F });
    EXPECT_FALSE(FloatFrame::decode(legacy, values, CrcProfile::CRC8_31_MODBUS));
    EXPECT_EQ(values, std::vector<float> { 42.0F });
}

TEST(FloatFrame, RecoversCapturedFrameAfterFragmentationAndBadCrc) {
    FloatFrame parser(CrcProfile::CRC8_31_MODBUS);
    std::vector<float> values;
    const auto legacy = FloatFrame::encode({ 118.0F, 0.58F });
    parser.append(legacy.data(), legacy.size());
    auto bad_tail = CAPTURED_FRAME;
    bad_tail.back() ^= 1U;
    parser.append(bad_tail.data(), bad_tail.size());
    parser.append(CAPTURED_FRAME.data(), 1);
    EXPECT_FALSE(parser.next(values));
    parser.append(CAPTURED_FRAME.data() + 1, 3);
    EXPECT_FALSE(parser.next(values));
    parser.append(CAPTURED_FRAME.data() + 4, CAPTURED_FRAME.size() - 4);
    parser.append(CAPTURED_FRAME.data(), CAPTURED_FRAME.size());
    ASSERT_TRUE(parser.next(values));
    EXPECT_EQ(values, (std::vector<float> { 118.0F, 0.58F }));
    ASSERT_TRUE(parser.next(values));
    EXPECT_EQ(values, (std::vector<float> { 118.0F, 0.58F }));
    EXPECT_FALSE(parser.next(values));

    FloatFrame wrong_profile;
    wrong_profile.append(CAPTURED_FRAME.data(), CAPTURED_FRAME.size());
    EXPECT_FALSE(wrong_profile.next(values));
}

} // namespace qd::serial_driver
