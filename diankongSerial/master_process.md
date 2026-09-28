# master_process

电控视觉通信封装。串口字节格式、字段顺序和 CRC 见 [湖南大学 RoboMaster 电控组通信协议](湖南大学RoboMaster电控组通信协议.md)。

## 接收视觉指令

`VisionInit(UART_HandleTypeDef *handle)` 注册串口接收回调并返回 `Vision_Recv_s *`。视觉下发一帧 14 字节，仅包含 `[pitch, yaw]` 两个 float。回调检查完整帧长度和 CRC；通过后更新 `recv_data.pitch`、`recv_data.yaw` 并喂通信守护进程。无效帧不会更新这两个变量。

调试时可在 `DecodeVisionFrame()` 返回后观察 `recv_data.pitch` 和 `recv_data.yaw`；发送测试值 `[1.0, -2.5]` 时应分别看到 `1.0` 与 `-2.5`。

`Vision_Recv_s` 仍含 `fire_mode`、`target_state`、`target_type`，但当前串口帧没有传输这些字段。

## 向视觉发送

`VisionSetAltitude(yaw, pitch, roll)`、`VisionSetFlag(...)` 和裁判数据设置函数更新本模块保存的发送数据；`VisionSend(void)` 将其中 15 个 float 按协议顺序编码为 66 字节帧并发送。业务代码需定期调用 `VisionSend()` 才会产生上行数据。当前帧不含 `enemy_color`、`work_mode`、`bullet_speed`。使用 UART DMA 时，底层发送完成前不可再次占用发送缓冲区。

`VISION_USE_UART` 使用固定 14 字节接收；`VISION_USE_VCP` 使用 USB 回调提供的实际接收长度。两种分支都调用相同的帧校验与字段更新逻辑。
