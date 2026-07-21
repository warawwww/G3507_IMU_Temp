# IMU 对外 UART/CAN 通信接口调研记录

> 本文只记录调研和可借鉴点，不定义本项目最终协议。
> 这里先按“UART/CAN 两个对外接口”理解；CAN 暂时待定，本轮只看 UART。若后续确实还要做 HART，需要单独补充 HART 物理层、地址、带宽和命令模型约束。

## 当前目标

- 为后续确定本项目 IMU 对外通信接口做准备。
- 先调研 JY901/WIT Motion 的 UART 命令、数据反馈和寄存器模型。
- 暂不确定本项目的帧头、字段、校验、命令号、单位缩放和 CAN 映射。

## 参考资料

- WIT Standard Communication Protocol PDF  
  `https://cdn.robotshop.com/rbm/f83835f4-5e29-4ee0-9cc2-e49300031503/6/681c0521-11c2-4e47-99b0-d336be43f142/8ab75072_wit-standard-communication-protocol.pdf`
- WITMOTION JY901 标准协议示例仓库  
  `https://github.com/WITMOTION/WitStandardProtocol_JY901`
- WITMOTION Linux C SDK 示例  
  `https://github.com/WITMOTION/WitStandardProtocol_JY901/tree/main/Linux_C/normal`

## JY901/WIT UART 协议总体形态

JY901/WIT 标准协议不是 ASCII 文本协议，而是十六进制二进制协议。它大体分成两类：

1. 连续输出的数据帧：设备按配置好的内容和输出频率，主动往串口推送姿态、角速度、加速度等数据。
2. 主机发起的命令帧：主机用固定 5 字节格式读写寄存器，修改输出内容、波特率、校准模式等。

这个设计的重点是“上位机持续解析数据流”，命令本身更像寄存器访问。它不强调每条写命令都有一个完整 ACK/NACK 反馈帧，配置是否生效通常靠读回寄存器、观察输出流变化或观察校准状态/数据变化确认。

## 连续输出帧

标准数据输出帧长度固定为 11 字节：

```text
0x55 TYPE DATA1L DATA1H DATA2L DATA2H DATA3L DATA3H DATA4L DATA4H SUM
```

- `0x55`：帧头。
- `TYPE`：数据类型。
- `DATAxL/DATAxH`：小端 16-bit 数据，低字节先发，高字节后发。
- `SUM`：前 10 字节逐字节求和后取低 8 位。

常见 `TYPE`：

| TYPE | 含义 |
|---|---|
| `0x50` | 时间 |
| `0x51` | 加速度 |
| `0x52` | 角速度 |
| `0x53` | 角度 |
| `0x54` | 磁场 |
| `0x55` | 端口状态 |
| `0x56` | 气压/高度 |
| `0x57` | 经纬度 |
| `0x58` | GPS 地速 |
| `0x59` | 四元数 |
| `0x5A` | GPS 精度 |
| `0x5F` | 寄存器读回 |

几个和 IMU 最相关的帧：

```text
加速度: 0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM
角速度: 0x55 0x52 WxL WxH WyL WyH WzL WzH VolL VolH SUM
角度:   0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
四元数: 0x55 0x59 Q0L Q0H Q1L Q1H Q2L Q2H Q3L Q3H SUM
读回:   0x55 0x5F REG1L REG1H REG2L REG2H REG3L REG3H REG4L REG4H SUM
```

典型缩放关系：

- 加速度：`acc = raw / 32768 * 16 g`
- 角速度：`gyro = raw / 32768 * 2000 deg/s`
- 角度：`angle = raw / 32768 * 180 deg`
- 四元数：`q = raw / 32768`
- 温度：`temperature = raw / 100 deg C`

## 命令写入帧

标准写寄存器命令为 5 字节：

```text
0xFF 0xAA ADDR DATAL DATAH
```

- `0xFF 0xAA`：命令帧头。
- `ADDR`：寄存器地址。
- `DATAL/DATAH`：写入值，小端。
- 数据是十六进制二进制，不是 ASCII 字符串。

资料里强调，修改配置通常需要三步：

```text
1. 解锁: 0xFF 0xAA 0x69 0x88 0xB5
2. 发送要修改的寄存器命令
3. 保存: 0xFF 0xAA 0x00 0x00 0x00
```

解锁后需要在约定时间内完成后续命令，否则会自动重新锁定。这个机制可以防止误写配置，但也让“写命令”带有全局状态，主机端需要处理好时序。

## 寄存器读回

SDK 示例里的普通串口读寄存器命令是：

```text
0xFF 0xAA 0x27 REGL REGH
```

读回帧固定返回 4 个连续寄存器：

```text
0x55 0x5F REG1L REG1H REG2L REG2H REG3L REG3H REG4L REG4H SUM
```

所以 JY901/WIT 的“命令反馈”更多是读回型反馈：主机发读命令后，设备把寄存器值作为一种 `0x55 0x5F` 数据帧塞回同一条串口数据流。解析器需要把普通 telemetry 帧和读回帧放在同一个流里处理。

## 常见控制寄存器

| 寄存器 | 地址 | 用途 |
|---|---:|---|
| `SAVE` | `0x00` | 保存参数、软件重启、恢复出厂 |
| `CALSW` | `0x01` | 校准模式 |
| `RSW` | `0x02` | 输出内容选择 |
| `RRATE` | `0x03` | 输出频率 |
| `BAUD` | `0x04` | 串口波特率 |
| `AXOFFSET~HZOFFSET` | `0x05~0x0D` | 零偏/偏置设置 |
| `KEY` | `0x69` | 解锁 |

`SAVE` 常见值：

```text
0x0000: 保存参数
0x00FF: 软件重启
0x0001: 恢复出厂
```

`CALSW` 常见值：

```text
0x00: 正常工作
0x01: 自动加速度校准
0x03: 高度清零
0x04: 航向角清零
0x07: 磁场球面拟合校准
0x08: 设置角度参考
0x09: 磁场双平面校准
```

`RSW` 用 bitmask 选择主动输出哪些数据帧，例如打开角速度、角度、加速度等。`RRATE` 用枚举值设置输出频率，常见为 0.2 Hz 到 200 Hz，部分型号支持 500 Hz 和 1000 Hz。`BAUD` 也是枚举值，覆盖 4800 到 230400 bps，部分型号支持 460800/921600 bps。

零偏寄存器中，陀螺仪零偏的标称单位是：

```text
gyro_offset_deg_s = GXOFFSET / 10000
```

也就是寄存器 LSB 对应 `0.0001 deg/s`。这一点对我们后面讨论“标定值显示几位小数是否丢精度”很有参考价值：对外显示可以少几位，但内部协议字段应该明确实际缩放，不能靠 UI 小数位决定精度。

## SDK 侧的解析思路

WIT 示例 SDK 的普通串口协议解析思路大致是：

- 输入数据按字节喂给 parser。
- 找到 `0x55` 帧头后，收满 11 字节。
- 计算前 10 字节低 8 位和，与第 11 字节比较。
- 校验失败时丢掉当前缓冲区第一个字节，继续滑动重同步。
- 校验成功后按 `TYPE` 更新对应寄存器数组，再通过回调通知上层。

这个策略实现很轻，但依赖固定长度帧和简单帧头。串口有丢字节、插字节、噪声时，能靠滑动同步恢复；但 `SUM8` 校验能力偏弱，误检能力不如 CRC16/CRC32。

## JY901/WIT 做法值得借鉴的点

- 固定长度 telemetry 帧很容易在 MCU 和上位机中解析，内存占用可控。
- 帧头、类型、payload、校验的结构清楚，字节流重同步简单。
- telemetry 和 register readback 复用同一条输出流，上位机只需要一个 parser。
- 寄存器模型扩展性不错，后续增加参数时不用重新设计命令帧。
- 输出内容 bitmask 和输出频率寄存器很实用，可以按场景降低带宽。
- 解锁/保存机制可以避免误改永久参数。

## JY901/WIT 做法的不足

- `SUM8` 校验太弱，适合短帧低风险场景，不适合我们后续要做高可靠控制命令时直接照搬。
- 写命令没有显式 ACK/NACK/result code，主机不容易区分“命令已收到但未生效”和“串口丢包”。
- 读回帧固定返回 4 个寄存器，简单但不够灵活。
- 没有明显的协议版本、能力查询、错误码、忙状态、校准进度字段。
- `0x55` 单字节帧头较短，靠固定长度和 checksum 补救，抗误同步能力有限。
- unlock/save 是全局状态，主机断连、重连、并发发送命令时需要格外小心。
- telemetry 帧没有统一序号或统一时间戳，不利于严格诊断丢包、延迟和采样抖动。

## 对本项目 UART 协议的启发，但不是决定

后续我们设计自己的 UART 时，可以重点讨论这些方向：

- 是否采用二进制协议作为正式协议，ASCII 只保留为调试/兼容模式。
- telemetry 是否也做固定长度短帧，便于高频低开销上报。
- 命令响应是否必须有明确 `ACK/NACK + command_id + result_code`。
- 校验是否至少用 CRC16，而不是简单 `SUM8`。
- 是否加入 `protocol_version`、`device_id`、`capability`、`fw_version` 查询命令。
- 是否把 streaming 数据和 command response 共用一个 parser，但用不同 frame type 区分。
- 是否每帧加入序号或时间戳，用于判断丢帧和延迟。
- 是否把“写 RAM 参数”和“保存到 Flash”明确拆成两个命令。
- 校准命令是否需要返回 `accepted/running/progress/done/failed`，而不是只靠状态流推断。
- 单轴 IMU 是否只暴露 Z 轴角速度/角度，还是保留三轴兼容格式。

## 后续待讨论问题

- UART 物理层：TTL UART、USB CDC、RS485，还是其他封装。
- 默认波特率和最高可选波特率。
- 上报数据集：raw、bias、corrected gyro、angle、temperature、state、calibration quality 是否都要出。
- 上报频率：默认频率、最高频率、是否允许主机订阅不同数据流。
- 命令分类：查询类、流控类、校准类、参数类、持久化类、诊断类。
- 失败反馈：CRC 错、命令不存在、参数非法、设备忙、校准失败、Flash 保存失败如何表达。
- CAN 接口是否复用 UART 的命令语义，还是做完全独立的 CAN ID 映射。

## 暂不定稿声明

本文件只用于“先看别人怎么做”。本项目最终协议还需要结合：

- 目标上位机/主控是谁。
- 是否需要兼容现有调试 HTML。
- 实际带宽和实时性要求。
- 标定流程是否要可交互显示进度。
- CAN 后续是否需要和电控系统已有协议共存。

下一步可以在本文基础上新开“UART 协议草案”，再把 telemetry、command、ACK、错误码和校准状态逐项定下来。
