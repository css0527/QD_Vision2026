#ifndef SEASKY_PROTOCOL_H
#define SEASKY_PROTOCOL_H

#include <stdint.h>

#define PROTOCOL_SOF 0xA5u
#define PROTOCOL_HEADER_SIZE 4u
#define PROTOCOL_TAIL_SIZE 2u
#define PROTOCOL_OVERHEAD_SIZE (PROTOCOL_HEADER_SIZE + PROTOCOL_TAIL_SIZE)

/**
 * @brief 计算指定字节序列的 CRC8。
 * @param message 待校验的字节序列。
 * @param length 字节序列长度。
 * @return CRC8 校验值。
 */
uint8_t Get_CRC8_Check(uint8_t *message, uint16_t length);

/**
 * @brief 计算指定字节序列的 CRC16。
 * @param message 待校验的字节序列。
 * @param length 字节序列长度。
 * @return CRC16 校验值。
 */
uint16_t Get_CRC16_Check(uint8_t *message, uint32_t length);

/**
 * @brief 将 float 数组打包为串口数据帧。
 * @param tx_data 待发送的 float 数组，按协议顺序排列。
 * @param float_length 数组中的 float 数量，每个 float 占 4 字节。
 * @param tx_buf 输出缓冲区。
 * @param tx_buf_capacity 输出缓冲区容量，单位为字节。
 * @param tx_buf_len 实际帧长的输出地址；失败时写入 0。
 * @return 1 表示打包成功，0 表示参数或容量无效。
 */
uint8_t get_protocol_send_data(const float *tx_data, uint8_t float_length, uint8_t *tx_buf,
                               uint16_t tx_buf_capacity, uint16_t *tx_buf_len);

/**
 * @brief 校验串口数据帧并复制其中的原始 float 字节。
 * @param rx_buf 收到的完整数据帧。
 * @param rx_buf_len 实际收到的帧长，单位为字节。
 * @param rx_data 接收数据的输出缓冲区。
 * @param rx_data_capacity 输出缓冲区容量，单位为字节。
 * @return 1 表示校验成功且数据已复制，0 表示帧无效或容量不足。
 * @note 数据字段为小端 32 位 float，调用者需按约定的字段顺序解释。
 */
uint8_t get_protocol_info(const uint8_t *rx_buf, uint16_t rx_buf_len, uint8_t *rx_data,
                          uint16_t rx_data_capacity);

#endif
