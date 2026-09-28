#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "../seasky_protocol.h"

/** @brief 验证电控编解码与上位机的 14 字节测试帧一致。 */
int main(void)
{
    const float values[2] = {1.0f, -2.5f};
    const uint8_t expected[14] = {
        0xA5, 0x08, 0x00, 0x67, 0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x20, 0xC0, 0xD0, 0x80
    };
    uint8_t frame[14] = {0};
    uint8_t decoded[sizeof(values)] = {0};
    uint16_t frame_length = 0;

    assert(get_protocol_send_data(values, 2, frame, sizeof(frame), &frame_length) == 1);
    assert(frame_length == sizeof(expected));
    assert(memcmp(frame, expected, sizeof(expected)) == 0);
    assert(get_protocol_info(frame, frame_length, decoded, sizeof(decoded)) == 1);
    assert(memcmp(decoded, values, sizeof(values)) == 0);

    frame[5] ^= 0x01u;
    assert(get_protocol_info(frame, frame_length, decoded, sizeof(decoded)) == 0);
    frame[5] ^= 0x01u;
    assert(get_protocol_info(frame, frame_length, decoded, sizeof(decoded) - 1u) == 0);
    assert(get_protocol_info(frame, frame_length - 1u, decoded, sizeof(decoded)) == 0);

    frame_length = 99u;
    assert(get_protocol_send_data(values, 2, frame, sizeof(frame) - 1u, &frame_length) == 0);
    assert(frame_length == 0u);
    return 0;
}
