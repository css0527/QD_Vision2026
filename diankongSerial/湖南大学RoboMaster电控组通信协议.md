<p align='right'>Seasky LIUWei</p>

<p align='right'>modified by neozng1@hnu.edu.cn</p>

# 湖南大学 RoboMaster 电控组串口通信协议

用于视觉与电控串口通信。当前帧没有 `cmd_id` 和 `flags_register`。本文帧长和 CRC 描述对应本目录样例：上行 15 个 float、`robomaster` CRC。已有实板抓包上行为两个 float，视觉当前配置为 `crc8_31_modbus`，不能将本目录样例直接视为正在运行的实板源码；差异见 [视觉串口驱动说明](../src/rm_hardware_driver/rm_serial_driver/README.md)。

## 串口配置

921600 波特率，8 位数据位，1 位停止位，无校验位、无硬件流控（8N1）。多字节数值低字节在前。

## 数据帧

| 偏移 | 字段 | 长度 | 说明 |
| --- | --- | --- | --- |
| 0 | `sof` | 1 字节 | 固定 `0xA5` |
| 1 | `data_length` | 2 字节 | 数据段**字节数**，小端 |
| 3 | `crc8` | 1 字节 | 校验偏移 0～2 |
| 4 | `float_data` | `data_length` 字节 | 连续的小端 IEEE 754 `float32` |
| 4 + `data_length` | `crc16` | 2 字节 | 校验此前整帧，小端 |

完整帧长为 `data_length + 6`。数据段长度必须是 4 的倍数。CRC8 初值 `0xFF`、反射多项式 `0x8C`；CRC16 初值 `0xFFFF`、反射多项式 `0x8408`。两者均不作结果异或。具体实现见 `seasky_protocol.c`。

### 视觉 → 电控

只发送两个 float，顺序为 `[pitch_deg, yaw_deg]`；`data_length = 8`，总帧长 14 字节。两者均为绝对目标角（度）。pitch 使用电机绝对零位，电控执行 `pitch * DEG_TO_RAD` 后作为绝对弧度目标；yaw 直接作为 `small_yaw` 的绝对目标。电控校验成功后写入 `Vision_Recv_s.pitch` 和 `Vision_Recv_s.yaw`。以下 `[1.0, -2.5]` 仅用于验证编解码，不是实机运行目标；其 `robomaster` CRC 完整帧为：

```text
A5 08 00 67 00 00 80 3F 00 00 20 C0 D0 80
```

### 电控 → 视觉

当前 `VisionSend()` 发送 15 个 float；`data_length = 60`，总帧长 66 字节：

```text
yaw, pitch, roll, control_id, game_progress, current_HP,
current_base_hp, allow_fire_amount, current_outpost_hp,
current_enemy_base_hp, current_enemy_outpost_hp,
hero_x, hero_y, cmd_x, cmd_y
```

这些字段按顺序逐个编码。`Vision_Send_s` 中的 `enemy_color`、`work_mode`、`bullet_speed` 不在当前帧中；视觉端会把缺失的 ROS 模式、实际弹速和 MCU 时间戳设为 0。

前三项必须是实际姿态反馈，不能填目标角：`yaw` 为与下行目标同一参考系的实测绝对角（度），`pitch` 为实测电机绝对角（弧度），`roll` 为实测角（度）。当前机构抬头时 pitch 减小；视觉侧根据水平零位和方向转换成仰角，发送侧不应提前做该转换。`VisionSetAltitude()` 只保存调用者传入的数值，不能验证其来源；调用者接入示例及缺失的业务源码说明见 [master_process](master_process.md)。

实板的两个 float 上行顺序为 `[yaw_deg, pitch_rad]`，与视觉下行的 `[pitch_deg, yaw_deg]` 在顺序和 pitch 单位上均不同，不能直接回传视觉目标包作为姿态反馈。

## 协议接口

```c
uint8_t get_protocol_send_data(const float *tx_data, uint8_t float_length,
                               uint8_t *tx_buf, uint16_t tx_buf_capacity,
                               uint16_t *tx_buf_len);

uint8_t get_protocol_info(uint8_t *rx_buf, uint16_t rx_buf_len,
                          uint8_t *rx_data, uint16_t rx_data_capacity);
```

`get_protocol_send_data()` 将 float 数组编码到输出缓冲区，成功返回 1，并写出实际帧长；容量不足或参数错误返回 0。`get_protocol_info()` 需要收到完整帧及其实际长度；帧头、长度、两个 CRC 和目标容量均通过检查才复制数据段并返回 1。调用者应提供与对应方向字段数匹配的缓冲区。

`VisionInit()` 配置电控接收 14 字节帧；成功解析后可在调试器观察 `master_process.c` 中的 `recv_data.pitch` 和 `recv_data.yaw`。`VisionSend()` 需要由电控业务代码定期调用，且 DMA 发送未完成时不要重用发送缓冲区。
