# master_process

电控视觉通信封装。串口字节格式、字段顺序和 CRC 见 [湖南大学 RoboMaster 电控组通信协议](湖南大学RoboMaster电控组通信协议.md)。

## 接收视觉指令

`VisionInit(UART_HandleTypeDef *handle)` 注册串口接收回调并返回 `Vision_Recv_s *`。视觉下发一帧 14 字节，仅包含 `[pitch_deg, yaw_deg]` 两个 float，均为电控使用的绝对目标角（度）。回调检查完整帧长度和 CRC；通过后更新 `recv_data.pitch`、`recv_data.yaw` 并喂通信守护进程。无效帧不会更新这两个变量。

电控将 `recv_data.yaw` 直接赋给 `small_yaw`，将 `recv_data.pitch * DEG_TO_RAD` 赋给 pitch 目标。此处 pitch 已包含电机零位和方向转换，不是水平为零的仰角，也不是相对修正量。当前视觉配置中水平电机角为 `0.58 rad`，对应下发约 `33.2316°`；抬头时电机角减小。

调试时可在 `DecodeVisionFrame()` 返回后观察 `recv_data.pitch` 和 `recv_data.yaw`；发送测试值 `[1.0, -2.5]` 时应分别看到 `1.0` 与 `-2.5`。

`Vision_Recv_s` 仍含 `fire_mode`、`target_state`、`target_type`，但当前串口帧没有传输这些字段。

## 向视觉发送

`VisionSetAltitude(measured_yaw_deg, measured_pitch_rad, measured_roll_deg)` 更新本模块保存的实测姿态，`VisionSetFlag(...)` 和裁判数据设置函数更新其他发送数据；`VisionSend(void)` 将其中 15 个 float 按协议顺序编码为 66 字节帧并发送。业务代码需定期更新实测姿态并调用 `VisionSend()` 才会产生上行数据。当前帧不含 `enemy_color`、`work_mode`、`bullet_speed`。使用 UART DMA 时，底层发送完成前不可再次占用发送缓冲区。

上行姿态必须来自电机或 IMU 的实际反馈：yaw 和 roll 为度，pitch 为电机绝对角弧度；yaw 的参考系须与下行目标一致。不能用 `gimbal_cmd_send` 的目标角或视觉指令回显代替实测角。视觉用上行姿态将相机观测变换到固定坐标系；若把尚未到达的目标角当成当前姿态，会错误估计装甲板位置，可能导致连续低头、限位或抖动。`VisionSetAltitude()` 仅复制输入值，不能判断它们来自传感器还是目标变量。

当前仓库仅有该函数的声明与定义，没有调用它的电控业务文件，因此这里的接口说明和形参命名修改不等于已经修复实板上报来源。接入处应按以下示例替换为固件中的真实反馈变量；三个 `measured_*` 名称均为占位符，不是本仓库已有的 MCU 变量：

```c
VisionSetAltitude(measured_yaw_deg, measured_pitch_rad, measured_roll_deg);
VisionSend();
```

在同一反馈更新周期取得这些值。pitch 反馈保持电机的绝对弧度值，由视觉按标定零位和方向换算；不能直接用 IMU 水平为零的俯仰角替换。验收时手动抬头，上行 pitch 应随实际运动减小；即使目标角已变化，在机构尚未到位时，上行仍应报告当前实测值。

## 本地样例与实板的差异

本目录发送 15 个 float，并使用 `robomaster` CRC。已有实板抓包为 `[yaw_deg, pitch_rad]` 两个 float，视觉当前配置使用 `crc8_31_modbus`；本地样例不能直接视为当前实板的完整源码。本次姿态接口说明不改变帧长度、顺序或 CRC。集成时须核对实际固件，详见 [视觉串口驱动说明](../src/rm_hardware_driver/rm_serial_driver/README.md)。

`VISION_USE_UART` 使用固定 14 字节接收；`VISION_USE_VCP` 使用 USB 回调提供的实际接收长度。两种分支都调用相同的帧校验与字段更新逻辑。
