/**
 * @file seasky_protocol.c
 * @brief 电控与视觉之间的串口浮点数据帧编解码。
 */

#include "seasky_protocol.h"

#include <string.h>

/**
 * @brief 计算帧头 CRC8，初值 FF，反射多项式 8C。
 * @param message 待校验字节序列。
 * @param length 字节序列长度。
 * @return CRC8 校验值。
 */
static uint8_t protocol_crc8(const uint8_t *message, uint16_t length)
{
    uint8_t crc = 0xFFu;
    uint16_t i;
    uint8_t bit;

    for (i = 0; i < length; ++i) {
        crc ^= message[i];
        for (bit = 0; bit < 8u; ++bit) {
            crc = (crc & 1u) != 0u ? (uint8_t)((crc >> 1u) ^ 0x8Cu)
                                    : (uint8_t)(crc >> 1u);
        }
    }
    return crc;
}

/**
 * @brief 计算整帧 CRC16，初值 FFFF，反射多项式 8408。
 * @param message 待校验字节序列。
 * @param length 字节序列长度。
 * @return CRC16 校验值，发送时低字节在前。
 */
static uint16_t protocol_crc16(const uint8_t *message, uint32_t length)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t bit;

    for (i = 0; i < length; ++i) {
        crc ^= message[i];
        for (bit = 0; bit < 8u; ++bit) {
            crc = (crc & 1u) != 0u ? (uint16_t)((crc >> 1u) ^ 0x8408u)
                                    : (uint16_t)(crc >> 1u);
        }
    }
    return crc;
}

uint8_t Get_CRC8_Check(uint8_t *message, uint16_t length)
{
    if (message == NULL) {
        return 0;
    }
    return protocol_crc8(message, length);
}

uint16_t Get_CRC16_Check(uint8_t *message, uint32_t length)
{
    if (message == NULL) {
        return 0;
    }
    return protocol_crc16(message, length);
}

uint8_t get_protocol_send_data(const float *tx_data, uint8_t float_length, uint8_t *tx_buf,
                               uint16_t tx_buf_capacity, uint16_t *tx_buf_len)
{
    uint16_t data_length;
    uint16_t frame_length;
    uint16_t checksum;
    uint16_t i;

    if (tx_buf_len == NULL) {
        return 0;
    }
    *tx_buf_len = 0;
    if (tx_data == NULL || tx_buf == NULL || sizeof(float) != 4u) {
        return 0;
    }

    data_length = (uint16_t)float_length * 4u;
    frame_length = data_length + PROTOCOL_OVERHEAD_SIZE;
    if (tx_buf_capacity < frame_length) {
        return 0;
    }

    tx_buf[0] = PROTOCOL_SOF;
    tx_buf[1] = (uint8_t)(data_length & 0xFFu);
    tx_buf[2] = (uint8_t)(data_length >> 8u);
    tx_buf[3] = protocol_crc8(tx_buf, PROTOCOL_HEADER_SIZE - 1u);

    for (i = 0; i < float_length; ++i) {
        uint32_t bits;
        uint16_t offset = PROTOCOL_HEADER_SIZE + i * 4u;
        memcpy(&bits, &tx_data[i], sizeof(bits));
        tx_buf[offset] = (uint8_t)(bits & 0xFFu);
        tx_buf[offset + 1u] = (uint8_t)((bits >> 8u) & 0xFFu);
        tx_buf[offset + 2u] = (uint8_t)((bits >> 16u) & 0xFFu);
        tx_buf[offset + 3u] = (uint8_t)((bits >> 24u) & 0xFFu);
    }

    checksum = protocol_crc16(tx_buf, frame_length - PROTOCOL_TAIL_SIZE);
    tx_buf[frame_length - 2u] = (uint8_t)(checksum & 0xFFu);
    tx_buf[frame_length - 1u] = (uint8_t)(checksum >> 8u);
    *tx_buf_len = frame_length;
    return 1;
}

uint8_t get_protocol_info(const uint8_t *rx_buf, uint16_t rx_buf_len, uint8_t *rx_data,
                          uint16_t rx_data_capacity)
{
    uint16_t data_length;
    uint16_t checksum;

    if (rx_buf == NULL || rx_data == NULL || rx_buf_len < PROTOCOL_OVERHEAD_SIZE
        || rx_buf[0] != PROTOCOL_SOF) {
        return 0;
    }
    if (protocol_crc8(rx_buf, PROTOCOL_HEADER_SIZE - 1u) != rx_buf[3]) {
        return 0;
    }

    data_length = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8u);
    if ((uint32_t)data_length + PROTOCOL_OVERHEAD_SIZE != rx_buf_len
        || data_length > rx_data_capacity || data_length % 4u != 0u) {
        return 0;
    }

    checksum = protocol_crc16(rx_buf, rx_buf_len - PROTOCOL_TAIL_SIZE);
    if (rx_buf[rx_buf_len - 2u] != (uint8_t)(checksum & 0xFFu)
        || rx_buf[rx_buf_len - 1u] != (uint8_t)(checksum >> 8u)) {
        return 0;
    }

    memcpy(rx_data, rx_buf + PROTOCOL_HEADER_SIZE, data_length);
    return 1;
}
