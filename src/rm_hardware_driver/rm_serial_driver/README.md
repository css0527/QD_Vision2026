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

视觉 → 电控：`[pitch_rad, yaw_deg]`，数据段 8 字节，完整帧 14 字节。

电控 → 视觉：实板当前发送 `[yaw_deg, pitch_rad]`，完整帧同样为 14 字节。接收器也兼容本地样例的 15 个 float 遥测帧（66 字节）：

```text
yaw, pitch, roll, control_id, game_progress, current_HP,
current_base_hp, allow_fire_amount, current_outpost_hp,
current_enemy_base_hp, current_enemy_outpost_hp,
hero_x, hero_y, cmd_x, cmd_y
```

电控 pitch 收发均为电机绝对角（弧度），yaw 收发均为度。实机水平 pitch 为 `0.58 rad`，向上抬时数值减小，最高为 `0.4 rad`，最低为 `0.75 rad`。ROS pitch 使用以水平为零、抬头为正的仰角（度）：

```text
接收：pitch_deg = (0.58 - pitch_rad) × 180/π
发送：pitch_rad = 0.58 - GimbalCmd.pitch × π/180
```

发送的 pitch 限制在 `[0.4, 0.75] rad`，yaw 原样传递。对应参数位于 `rm_bringup/config/node_params/serial_driver_params.yaml`：

```yaml
pitch_horizontal_rad: 0.58
pitch_up_sign: -1.0
pitch_min_rad: 0.4
pitch_max_rad: 0.75
```

通用接收公式为 `(pitch_rad - pitch_horizontal_rad) × pitch_up_sign × 180/π`；方向参数只接受 `+1.0` 或 `-1.0`。更换机械零位时需同步核对零位、方向及上下限，重启节点生效。

| 云台位置 | 电控绝对 pitch | ROS 仰角 |
| --- | --- | --- |
| 水平 | 0.58 rad | 约 0° |
| 最高 | 0.4 rad | +10.3132° |
| 最低 | 0.75 rad | −9.7403° |

TF 绕 Y 的角度取 ROS 仰角的负值，因此抬头时 TF 的 pitch 为负，这是坐标轴约定。不能直接把电机绝对角取负作为 TF 角度。两个 float 上行没有 roll，因此设置为 0。帧中没有视觉模式、实际弹速和 MCU 时间戳，这些消息字段暂设为 0；模式 0 表示红色自瞄，解算器继续使用已有弹速配置。

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
colcon build --symlink-install --packages-up-to rm_serial_driver --parallel-workers 4
colcon build --symlink-install --packages-select rm_bringup
source install/setup.bash
ros2 launch rm_bringup bringup_SingleProcess.launch.py
```

`enable_data_print: true` 每秒最多各打印一次通过 CRC 的完整姿态和成功写入的角度。水平位置的 `pitch_deg` 应接近 0（float 舍入可能显示为约 `9.56e-7`）：

```text
RX Seasky OK: floats=2 yaw_deg=118 pitch_rad=0.58 pitch_deg=9.56226e-07 roll_deg=0
```

发送日志中的 `cmd_pitch_deg` 是解算器要求的仰角，`pitch_rad` 是转换并限位后实际写出的电控绝对角。例如：

```text
TX Seasky WRITTEN: pitch_rad=0.492734 yaw_deg=118 cmd_pitch_deg=5 distance_m=3 pitch_limited=0
```

`pitch_limited=1` 表示请求超出机械范围，实际发送值已限位。`WRITTEN` 仅代表本机串口写入完成；仍需观察电控接收变量验证下行接收。

接收器会拼接分段读取的数据并处理合包。旧版日志中的每行 `RX Seasky:` 只表示一次串口读取，不表示一个完整帧或校验成功。

可在另一个已 source 工作区的终端检查 ROS 消息：

```bash
ros2 topic echo /serial/receive rm_interfaces/msg/SerialReceiveData --qos-reliability best_effort
```

收到有效姿态后，节点才会发布 `serial/receive` 和 `odom → gimbal_link` TF。`hero`、`air`、`infantry`、`sentry`、`seasky` 均使用以上帧；机器人类型影响订阅和模式服务。`hero` / `air` 订阅 `armor_solver/cmd_gimbal`，其余类型还订阅 `rune_solver/cmd_gimbal`。

解算器用 `distance < 0` 表示无有效目标；两 float 协议不传目标状态，此时串口节点不发送零角度指令。

日志中的 `Lookup would require extrapolation into the future` 表示图像请求时刻超出 TF 中最新的姿态时间，不能凭此认定 PnP 或弹道算法有错。本次标定修正不解决 TF 时间延迟；如果该错误持续出现，需要继续核对姿态上报频率、时间戳和线程调度。

## 验证电控接收

先停止占用同一串口的节点或工具，再发送一次两 float 测试：

```bash
# 只查看帧，不打开串口
ros2 run rm_serial_driver serial_probe --crc-profile crc8_31_modbus --dry-run

# 波特率使用实板当前配置
ros2 run rm_serial_driver serial_probe --port /dev/ttyACM0 --baud 115200 --crc-profile crc8_31_modbus --values 0.58 -2.5

# 每秒一次，共 10 次
ros2 run rm_serial_driver serial_probe --port /dev/ttyACM0 --baud 115200 --crc-profile crc8_31_modbus --count 10 --interval-ms 1000
```

电控调试器中观察接收变量，成功时应得到 `pitch=0.58 rad`、`yaw=-2.5°`。探针参数是线上原始值，不经过 ROS 零位转换或机械限位；其中 `0.58 rad` 对应当前云台水平位置。默认值为 `[0.58 rad, -2.5°]`。`--values` 恰好接受两个有限 float；其他数量会在打开串口前报错。

`TX_WRITTEN` 只表示本机写入成功。`DELIVERY_UNCONFIRMED` 表示未收到相同帧回显，采用观察变量方式验证时正常；电控的姿态上行也不证明下行已经接收。`RX` 显示返回字节。只有电控增加原帧回显后，`--expect-echo` 和 `ECHO_CONFIRMED` 才能用于自动确认。退出码 0 表示本机发送成功或要求的回显成功；1 为参数/串口错误；2 为要求回显但未收到。
