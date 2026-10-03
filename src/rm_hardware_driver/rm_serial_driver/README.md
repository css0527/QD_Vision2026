# rm_serial_driver

串口驱动使用无 `cmd_id`、无 `flags_register` 的 Seasky 浮点帧。串口为 8N1、无流控，波特率应与实板相同；当前 `serial_driver_params.yaml` 配置为 `/dev/ttyACM0`、115200。

## 帧格式

| 偏移 | 字段 | 长度 |
| --- | --- | --- |
| 0 | 帧起始 `0xA5` | 1 字节 |
| 1 | 数据段字节数，小端 | 2 字节 |
| 3 | 帧头 CRC8，覆盖偏移 0～2 | 1 字节 |
| 4 | 小端 IEEE 754 `float32` 数组 | 数据段字节数 |
| 4 + 数据段字节数 | 整帧 CRC16，覆盖此前所有字节，小端 | 2 字节 |

视觉 → 电控：`[pitch_deg, yaw_deg]`，数据段 8 字节，完整帧 14 字节。`pitch_deg` 是电机绝对目标角（度），不是水平为零的仰角或相对误差。

电控 → 视觉：实板当前发送 `[yaw_deg, pitch_rad]`，完整帧同样为 14 字节。接收器也兼容本地样例的 15 个 float 遥测帧（66 字节）：

```text
yaw, pitch, roll, control_id, game_progress, current_HP,
current_base_hp, allow_fire_amount, current_outpost_hp,
current_enemy_base_hp, current_enemy_outpost_hp,
hero_x, hero_y, cmd_x, cmd_y
```

上行 yaw/pitch 必须是当前实测姿态，不能填 `gimbal_cmd_send` 的目标角。视觉使用这些值建立 TF，目标角不能表示机构尚未到位时的实际朝向。串口数据本身无法区分反馈源，须在电控业务调用处选用实测变量；本仓库 `diankongSerial` 仅有通信封装，缺少该调用处，接入约定见 [master_process](../../../diankongSerial/master_process.md)。

电控下行处理执行 `gimbal_cmd_send.pitch = vision_recv_data->pitch * DEG_TO_RAD`，因此视觉必须发送绝对角度数；电控上报的 pitch 仍是绝对角弧度。yaw 收发均为度，电控直接将收到的 yaw 赋给 `small_yaw`。实机水平 pitch 为 `0.58 rad`，向上抬时数值减小，最高为 `0.4 rad`，最低为 `0.75 rad`。ROS pitch 使用以水平为零、抬头为正的仰角（度）：

```text
接收：ROS pitch_deg = (0.58 - RX pitch_rad) × 180/π
发送：target_pitch_rad = clamp(0.58 - GimbalCmd.pitch × π/180, 0.4, 0.75)
      TX pitch_deg = target_pitch_rad × 180/π
电控：gimbal_cmd_send.pitch = TX pitch_deg × π/180
```

发送前将绝对 pitch 限制在 `[0.4, 0.75] rad`，再转为度，并按下文的平滑配置逐步到达目标。水平目标 `GimbalCmd.pitch=0°` 最终应发送约 `33.231552°`，电控换算后得到 `0.58 rad`。如果发送 `0.58`，电控会得到约 `0.010123 rad`，低于抬头极限，导致云台持续保持抬头限位。对应参数位于 `rm_bringup/config/node_params/serial_driver_params.yaml`：

```yaml
pitch_horizontal_rad: 0.58
pitch_up_sign: -1.0
pitch_min_rad: 0.4
pitch_max_rad: 0.75
```

通用接收公式为 `(pitch_rad - pitch_horizontal_rad) × pitch_up_sign × 180/π`；方向参数只接受 `+1.0` 或 `-1.0`。更换机械零位时需同步核对零位、方向及上下限，重启节点生效。

| 云台位置 | 电控上报/执行的绝对 pitch | 视觉发送的绝对 pitch | ROS 仰角 |
| --- | --- | --- | --- |
| 水平 | 0.58 rad | 33.2316° | 约 0° |
| 最高 | 0.4 rad | 22.9183° | +10.3132° |
| 最低 | 0.75 rad | 42.9718° | −9.7403° |

TF 绕 Y 的角度取 ROS 仰角的负值，因此抬头时 TF 的 pitch 为负，这是坐标轴约定。不能直接把电机绝对角取负作为 TF 角度。两个 float 上行没有 roll，因此设置为 0。帧中没有视觉模式、实际弹速和 MCU 时间戳，这些消息字段暂设为 0；模式 0 表示红色自瞄，解算器继续使用已有弹速配置。

## 平滑跟随

串口节点默认启用平滑，在机械限位之后对实际发送目标进行低通和速度限制。首次识别、切换目标或持续丢失后重捕时，首包发送最新反馈角；后续逐步接近解算目标。短时掉帧不反复重置平滑状态；反馈过期时停止发送。上行必须使用实测姿态，否则平滑仍会从错误姿态起步。

```yaml
follow:
  enabled: true
  time_constant_s: 0.18
  pitch_rate_deg_s: 12.0
  pitch_deadband_deg: 0.25
  yaw_rate_deg_s: 60.0
  feedback_timeout_s: 0.30
idle_return:
  enabled: true
  command_timeout_s: 0.15
```

将 `pitch_rate_deg_s` 调为 `10.0` 可减缓俯仰；增大 `time_constant_s` 会进一步减轻短时抖动，但增加跟随滞后。参数修改后重启节点。yaw 按最短角差平滑，并维持连续绝对角，跨过 ±180° 时不会突然跳 360°。`enabled: false` 可用于对照排查，此时直接发送限位后的目标。

`pitch_deadband_deg` 相对上一发送角抑制俯仰微小波动：角差不超过 0.25° 时保持原角，超过时将剩余角差低通并限速。死区中心不随每帧反馈重置，yaw 不受影响；稳定跟随允许最多约 0.25° 的俯仰残差。启动或丢失后的回水平关闭死区，仍精确趋近标定的水平位置。

`idle_return` 的独立定时器负责启动和指令断流后的回水平：收到有效实测反馈后，从当前 pitch 平滑回到 `pitch_horizontal_rad`，保持进入回中时的实测 yaw。有效解算指令持续到达时，定时器不额外发送。单次短掉帧先保持；超过 `command_timeout_s` 未收到有效指令才回中，重复的无目标消息不会推迟该期限。回中途中重捕从最新实测角恢复跟随。串口反馈过期时停止发送，恢复后重新保持当前 yaw。禁用 `idle_return.enabled` 后，无目标时仅保持最后指令。

向上移动在画面上方时，应看到 `OBS optical_pitch_deg` 增大，随后解算仰角增大、电机绝对目标 pitch 减小。平滑只限制变化速度，不改变上下方向；方向异常应结合 solver 的 `OBS/AIM` 和实测反馈定位。

## CRC 配置

节点参数 `crc_profile` 和探针参数 `--crc-profile` 接受以下值。收发必须使用同一组已与电控约定的参数，解析器始终检查 CRC。

| 配置 | CRC8 | CRC16 |
| --- | --- | --- |
| `crc8_31_modbus`（节点与探针默认） | 初值 `00`，多项式 `31`，不反射 | 初值 `FFFF`，反射多项式 `A001` |
| `robomaster` | 初值 `FF`，反射多项式 `8C` | 初值 `FFFF`，反射多项式 `8408` |

两组均无最终异或。第一组由实板日志中的 CRC 值匹配得到，当前捕获的完整上行帧为：

```text
A5 08 00 F4 00 00 EC 42 E1 7A 14 3F C1 3D
```

该帧原始值为 `yaw=118.0°`、`pitch=0.58 rad`，按当前标定换算后 ROS pitch 接近 `0°`。仅一个不同内容的样本不足以唯一确认全部 CRC 参数；应再用变化的姿态或电控实际 CRC 源码核对。本地 `diankongSerial/seasky_protocol.c` 是先前整理的 `robomaster` CRC 样例，且发送 15 个 float，与此次实板固件不同；若烧录该样例，需改用 `robomaster`，不要将本地样例视为当前实板源码。

## 构建与运行

在挂载当前检出目录的开发容器中执行：

```bash
cd /ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to rm_serial_driver armor_solver rm_bringup --parallel-workers 4
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

`enable_data_print: true` 每秒最多各打印一次通过 CRC 的完整姿态和成功写入的角度。水平位置的 `pitch_deg` 应接近 0（float 舍入可能显示为约 `9.56e-7`）：

```text
RX Seasky OK: floats=2 yaw_deg=118 pitch_rad=0.58 pitch_deg=9.56226e-07 roll_deg=0
```

发送日志中的 `cmd_pitch_deg` 是解算器要求的仰角，`goal_pitch_rad` 是限位后的目标，`pitch_deg` 是平滑后实际写出的电机绝对角度数，`target_pitch_rad` 是电控换算后应执行的绝对角弧度。`feedback_pitch_rad`、`feedback_yaw_deg` 是最近一次有效反馈。例如已接近目标时：

```text
TX Seasky WRITTEN: pitch_deg=28.2316 yaw_deg=118 cmd_pitch_deg=5 target_pitch_rad=0.492734 goal_pitch_rad=0.492734 goal_yaw_deg=118 feedback_pitch_rad=0.492734 feedback_yaw_deg=118 distance_m=3 pitch_limited=0
```

`pitch_limited=1` 表示请求超出机械范围，实际发送值已限位。`idle_return=1` 表示当前正在回水平，`goal_pitch_rad` 应为水平零位，yaw 为回中时锁定的实测角。`WRITTEN` 仅代表本机串口写入完成；仍需观察电控接收变量验证下行接收。

接收器会拼接分段读取的数据并处理合包。旧版日志中的每行 `RX Seasky:` 只表示一次串口读取，不表示一个完整帧或校验成功。

可在另一个已 source 工作区的终端检查 ROS 消息：

```bash
ros2 topic echo /serial/receive rm_interfaces/msg/SerialReceiveData --qos-reliability best_effort
```

收到有效姿态后，节点才会发布 `serial/receive` 和 `odom → gimbal_link` TF。`hero`、`air`、`infantry`、`sentry`、`seasky` 均使用以上帧；机器人类型影响订阅和模式服务。`hero` / `air` 订阅 `armor_solver/cmd_gimbal`，其余类型还订阅 `rune_solver/cmd_gimbal`。

解算器用 `distance < 0` 表示无有效目标；两 float 协议不传目标状态。串口会先短时保持，再按 `idle_return` 配置回水平；下发的是标定后的水平绝对角和保持的 yaw，而非消息中的零角度。

日志中的 `Lookup would require extrapolation into the future` 表示图像请求时刻超出 TF 中最新的姿态时间，不能凭此认定 PnP 或弹道算法有错。本次标定修正不解决 TF 时间延迟；如果该错误持续出现，需要继续核对姿态上报频率、时间戳和线程调度。

## 验证电控接收

先停止占用同一串口的节点或工具，再发送一次两 float 测试：

```bash
# 只查看帧，不打开串口
ros2 run rm_serial_driver serial_probe --crc-profile crc8_31_modbus --values 33.231552 118 --dry-run

# 波特率使用实板当前配置
ros2 run rm_serial_driver serial_probe --port /dev/ttyACM0 --baud 115200 --crc-profile crc8_31_modbus --values 33.231552 118

# 每秒一次，共 10 次
ros2 run rm_serial_driver serial_probe --port /dev/ttyACM0 --baud 115200 --crc-profile crc8_31_modbus --values 33.231552 118 --count 10 --interval-ms 1000
```

示例中的 yaw `118°` 来自上面的抓包，应换成希望保持的当前 yaw 绝对角。电控调试器中观察接收变量，成功时应得到 `pitch≈33.231552°`、`yaw=118°`，执行 `pitch * DEG_TO_RAD` 后目标为水平位置 `0.58 rad`。探针参数是线上原始值，不经过 ROS 零位转换或机械限位。默认值为 `[33.231552°, -2.5°]`，其中 pitch 对应当前标定下的水平位置；输出标签为 `PITCH_DEG` 和 `YAW_DEG`。对实板验证请像示例一样显式填写 `--values`，以保持当前 yaw。该选项恰好接受两个有限 float；其他数量会在打开串口前报错。

`TX_WRITTEN` 只表示本机写入成功。`DELIVERY_UNCONFIRMED` 表示未收到相同帧回显，采用观察变量方式验证时正常；电控的姿态上行也不证明下行已经接收。`RX` 显示返回字节。只有电控增加原帧回显后，`--expect-echo` 和 `ECHO_CONFIRMED` 才能用于自动确认。退出码 0 表示本机发送成功或要求的回显成功；1 为参数/串口错误；2 为要求回显但未收到。
