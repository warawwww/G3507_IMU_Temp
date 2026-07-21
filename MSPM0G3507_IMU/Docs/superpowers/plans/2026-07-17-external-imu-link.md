# 外部 IMU 通信任务实现计划

> **给后续执行者：** 按任务逐项实现和验证。每个任务都应能单独构建检查，避免一次性把协议、任务调度、app 接入全部揉在一起。

**目标：** 新建一个面向外部 MCU 的 IMU 通信任务。先做 UART，CAN 暂不定；UART 同时支持一套最小 JY901 兼容数据流，以及一套我们自己的命令/响应/数据协议。

**架构：** 现有 `HostLink` 继续只服务 Type-C/HTML 调试页面，不参与这套外部 MCU 协议。新增 `ExternalIMULink` 任务，使用 `BSP_UART_PORT_EXTERNAL`，从 `IMU_Task_GetSample()` 获取 IMU 数据，并把外部 MCU 发来的命令转成 app 层已有的零飘、AutoC、1080 标定、角度归零请求。协议编解码独立放在 `external_imu_protocol` 模块中，后续 CAN 可以复用同一套语义。

## 全局约束

- 不改变当前 Type-C/HTML 使用的 `HostLink` 行为。
- 新外部通信任务只使用 `BSP_UART_PORT_EXTERNAL`。
- JY901 兼容模式先只做输出数据流，不实现 JY901 寄存器读写、修改配置、保存参数等命令。
- 我们自己的协议必须包含三类逻辑帧：
  - 上位机/外部 MCU 发给下位机的命令帧。
  - 下位机发给上位机/外部 MCU 的响应帧。
  - 下位机持续发送的数据帧。
- 自定义 UART 帧要适合逐字节解析，方便移植到其他 MCU。
- 自定义协议使用 CRC16 校验，不使用简单 SUM8。
- 自定义协议帧头不能和 JY901 的 `0x55` 数据帧冲突。
- 自定义协议数据帧默认 `100 Hz`，每帧反馈状态和已解算数据，不发送陀螺仪原始 raw。
- JY901 兼容输出流按 WIT/JY901 常见默认回传频率 `10 Hz` 推送。
- CAN 本轮只保留语义边界，不确定 CAN ID 和具体帧格式。
- 单帧长度必须小于 `BSP_UART_TX_BUFFER_SIZE`，当前为 `256` 字节。

---

## 文件划分

- 新建 `User/external_imu_protocol.h`：协议常量、命令枚举、状态枚举、payload 定义、编码/解析接口。
- 新建 `User/external_imu_protocol.c`：CRC16、JY901 最小帧编码、自定义协议帧编码、自定义协议逐字节解析器。
- 新建 `User/external_imu_link.h`：外部通信任务 API，以及供 app 层读取命令请求的接口。
- 新建 `User/external_imu_link.c`：基于 `BSP_UART_PORT_EXTERNAL` 的轮询任务、数据发送调度、命令解析和响应发送。
- 修改 `User/app.c`：初始化和运行 `ExternalIMULink`，并把外部 MCU 命令并入现有 IMU 操作流程。
- 修改 `User/app_config.h`：增加 0/1 宏，控制外部通信任务、JY901 兼容流、自定义协议流。
- 修改 `Docs/IMU_EXTERNAL_UART_CAN_INTERFACE_RESEARCH.md`：实现方向确认后，把最终选型补进调研文档。

---

## 协议草案

### JY901 兼容输出流

JY901 兼容模式只输出两类 11 字节帧：

```text
55 52 WxL WxH WyL WyH WzL WzH VolL VolH SUM
55 53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
```

本项目是单轴陀螺仪，所以映射规则先定为：

```text
Wx = 0
Wy = 0
Wz = 当前校正后角速度
Roll = 0
Pitch = 0
Yaw = 当前归一化角度
Vol/Version = 0
SUM = 前 10 字节低 8 位求和
```

JY901 兼容模式不接收 JY901 寄存器命令，也不承诺可以被 JY901 工具完整配置。它的目的只是让已有 WIT/JY901 风格解析器能看到基本角速度和角度。我们的 Native `A5 5A` 命令帧仍然会被解析，用于调试页触发零飘、AutoC、1080 标定和角度归零。

### 自定义 UART 帧

自定义协议使用固定帧头：

```text
A5 5A LEN TYPE MSG_ID PAYLOAD... CRC_L CRC_H
```

字段规则：

```text
LEN     = 从 TYPE 到 PAYLOAD 结束的字节数
TYPE    = 帧类型
MSG_ID  = 命令号、响应对应命令号，或数据消息号
PAYLOAD = 0 到 64 字节
CRC16   = CRC16-CCITT，覆盖 LEN、TYPE、MSG_ID、PAYLOAD
CRC     = 小端发送，低字节在前
```

帧类型：

```c
typedef enum {
    EXTERNAL_IMU_FRAME_COMMAND  = 0x01,
    EXTERNAL_IMU_FRAME_RESPONSE = 0x02,
    EXTERNAL_IMU_FRAME_DATA     = 0x03,
} ExternalIMUProtocol_FrameType;
```

命令号：

```c
typedef enum {
    EXTERNAL_IMU_CMD_PING            = 0x01,
    EXTERNAL_IMU_CMD_ZERO_CAL        = 0x10,
    EXTERNAL_IMU_CMD_AUTOC           = 0x11,
    EXTERNAL_IMU_CMD_CAL1080         = 0x12,
    EXTERNAL_IMU_CMD_ANGLE_ZERO      = 0x13,
} ExternalIMUProtocol_Command;
```

响应状态：

```c
typedef enum {
    EXTERNAL_IMU_STATUS_OK           = 0x00,
    EXTERNAL_IMU_STATUS_BAD_CRC      = 0x01,
    EXTERNAL_IMU_STATUS_BAD_FRAME    = 0x02,
    EXTERNAL_IMU_STATUS_BAD_COMMAND  = 0x03,
    EXTERNAL_IMU_STATUS_BAD_PARAM    = 0x04,
    EXTERNAL_IMU_STATUS_BUSY         = 0x05,
    EXTERNAL_IMU_STATUS_FAILED       = 0x06,
} ExternalIMUProtocol_Status;
```

响应 payload 先保持 4 字节：

```text
payload[0] = status
payload[1] = 当前 IMU state
payload[2] = 当前 calibration result
payload[3] = 保留，填 0
```

这样外部 MCU 每发一个命令，都能知道下位机是收到了、忙、命令不存在、参数非法，还是校验错。

---

## 任务 1：建立协议模块和 CRC16

**文件：**

- 新建 `User/external_imu_protocol.h`
- 新建 `User/external_imu_protocol.c`

**产出接口：**

```c
uint16_t ExternalIMUProtocol_Crc16Ccitt(
    const uint8_t *data, size_t length);

size_t ExternalIMUProtocol_EncodeNativeFrame(
    uint8_t frameType, uint8_t msgId,
    const uint8_t *payload, uint8_t payloadLength,
    uint8_t *out, size_t outSize);

bool ExternalIMUProtocol_ParseNativeByte(
    ExternalIMUProtocol_Parser *parser,
    uint8_t byte,
    ExternalIMUProtocol_Frame *frame);
```

步骤：

- [ ] 定义自定义协议帧头、最大 payload 长度、帧类型、命令号、响应状态。
- [ ] 实现 CRC16-CCITT 软件版本，使用多项式 `0x1021`，初值 `0xFFFF`，无 final XOR。
- [ ] 实现自定义帧编码函数，buffer 太小时返回 `0`。
- [ ] 实现逐字节 parser 状态机，状态包括：

```c
typedef enum {
    EXTERNAL_IMU_PARSE_SYNC_0 = 0,
    EXTERNAL_IMU_PARSE_SYNC_1,
    EXTERNAL_IMU_PARSE_LEN,
    EXTERNAL_IMU_PARSE_BODY,
    EXTERNAL_IMU_PARSE_CRC_L,
    EXTERNAL_IMU_PARSE_CRC_H,
} ExternalIMUProtocol_ParseState;
```

- [ ] 长度非法、CRC 错误、同步失败时回到 `SYNC_0`，保证串口乱流后能重新同步。
- [ ] 跑一次 `eide build`，确认新增模块能编译。

---

## 任务 2：实现 JY901 兼容输出编码

**文件：**

- 修改 `User/external_imu_protocol.h`
- 修改 `User/external_imu_protocol.c`

**产出接口：**

```c
size_t ExternalIMUProtocol_EncodeJY901GyroFrame(
    int32_t wzMilliDps, uint8_t *out, size_t outSize);

size_t ExternalIMUProtocol_EncodeJY901AngleFrame(
    int32_t yawMilliDeg, uint8_t *out, size_t outSize);
```

步骤：

- [ ] 实现 `SUM8` helper：前 10 字节求和取低 8 位。
- [ ] 实现 `0x55 0x52` 角速度帧编码。
- [ ] 实现 `0x55 0x53` 角度帧编码。
- [ ] 角速度换算：

```text
raw = clamp(wz_dps / 2000 * 32768, -32768, 32767)
```

- [ ] 角度换算：

```text
raw = clamp(yaw_deg / 180 * 32768, -32768, 32767)
```

- [ ] buffer 足够时固定返回 `11`，buffer 不足时返回 `0`。

---

## 任务 3：实现自定义数据帧 payload

**文件：**

- 修改 `User/external_imu_protocol.h`
- 修改 `User/external_imu_protocol.c`

**产出接口：**

```c
size_t ExternalIMUProtocol_EncodeNativeIMUData(
    const ExternalIMUProtocol_NativeDataPayload *payload,
    uint8_t *out,
    size_t outSize);
```

数据 payload 先用固定字段：

```c
typedef struct {
    uint32_t timeMs;
    int32_t angularRateMilliDps;
    int32_t angleMilliDeg;
    int32_t normalizedAngleMilliDeg;
    uint8_t imuState;
    uint8_t appState;
    uint8_t calResult;
    uint8_t flags;
} ExternalIMUProtocol_NativeDataPayload;
```

字段说明：

- payload 固定 `20 bytes`，完整帧固定 `27 bytes`。
- `angularRateMilliDps`、`angleMilliDeg`、`normalizedAngleMilliDeg` 都是下位机完成零飘、比例标定和解算后的结果。
- 高频实时帧不发送 `bias`、温度、比例修正等内部诊断参数；这些数据后续如有需要，用单独诊断帧或查询命令扩展。
- `flags bit0` 表示正在静止自学习 bias。
- `flags bit1` 表示外部 UART RX 发生过溢出，发送后清除。
- `flags bit2` 表示角度数据有效；本次启动校准成功前，Native 状态帧继续发送，但角速度和角度字段填 0。
- JY901 兼容流没有状态/flags 字段，角度有效前不发送 JY901 数据帧。

步骤：

- [ ] 不直接 `memcpy struct`，避免不同编译器的结构体对齐差异。
- [ ] 实现显式小端写入 helper：

```c
static void ExternalIMUProtocol_WriteU32LE(uint8_t *out, uint32_t value);
static void ExternalIMUProtocol_WriteI32LE(uint8_t *out, int32_t value);
```

- [ ] 用 `TYPE = EXTERNAL_IMU_FRAME_DATA`、`MSG_ID = 0x01` 编码 IMU 数据帧。
- [ ] 确认完整数据帧长度小于 `256` 字节。

---

## 任务 4：新建外部 IMU 通信任务

**文件：**

- 新建 `User/external_imu_link.h`
- 新建 `User/external_imu_link.c`

**产出接口：**

```c
void ExternalIMULink_Init(void);
void ExternalIMULink_Run(uint8_t appState);

bool ExternalIMULink_TakeZeroCalibrationRequest(void);
bool ExternalIMULink_TakeAutoCRequest(void);
bool ExternalIMULink_TakeRotationCalibrationRequest(void);
bool ExternalIMULink_TakeAngleResetRequest(void);
```

步骤：

- [ ] `ExternalIMULink_Init()` 中清空 `BSP_UART_PORT_EXTERNAL` RX，重置 parser、请求标志和上次发送时间。
- [ ] `ExternalIMULink_Run()` 每次读取 `BSP_UART_PORT_EXTERNAL` 中所有可读字节。
- [ ] 收到自定义 `COMMAND` 帧后执行：

```text
PING       -> 立即回复 OK
ZERO_CAL   -> 设置零飘请求标志，回复 OK
AUTOC      -> 设置 AutoC 请求标志，回复 OK
CAL1080    -> 设置 1080 标定请求标志，回复 OK
ANGLE_ZERO -> 设置角度归零请求标志，回复 OK
未知命令   -> 回复 BAD_COMMAND
```

- [ ] 如果 IMU 正在标定，后续可以把不允许打断的命令回复 `BUSY`。第一版可先复用 app 层现有优先级，保证不直接打断旋转标定。
- [ ] 实现响应帧发送。UART TX 忙时，允许丢弃本次非关键连续数据帧；命令响应尽量重试一次或排到下一轮发送。
- [ ] 初始数据发送周期：

```c
#define EXTERNAL_IMU_LINK_NATIVE_PERIOD_MS (10U)
#define EXTERNAL_IMU_LINK_JY901_PERIOD_MS  (100U)
```

- [ ] JY901 兼容流启用时，每 `100 ms` 连续发送一帧角速度和一帧角度。
- [ ] 自定义数据流启用时，发送完整 IMU 数据帧。

---

## 任务 5：接入 app 主流程

**文件：**

- 修改 `User/app.c`
- 修改 `User/app_config.h`

步骤：

- [ ] 在 `User/app_config.h` 增加宏：

```c
#define APP_ENABLE_EXTERNAL_IMU_LINK         (1)
#define APP_EXTERNAL_IMU_ENABLE_NATIVE_LINK  (1)
#define APP_EXTERNAL_IMU_ENABLE_JY901_STREAM (0)
```

- [ ] 给这三个宏加 `#error` 检查，只允许 `0` 或 `1`。
- [ ] 在 `User/app.c` include：

```c
#include "external_imu_link.h"
```

- [ ] 在 `APP_Init()` 中，`HostLink_Init()` 后初始化：

```c
#if APP_ENABLE_EXTERNAL_IMU_LINK
    ExternalIMULink_Init();
#endif
```

- [ ] 在 `APP_Run()` 中，`HostLink_Run()` 后运行：

```c
#if APP_ENABLE_EXTERNAL_IMU_LINK
    ExternalIMULink_Run((uint8_t)g_appState);
#endif
```

- [ ] 在 `APP_HandleKeyEvents()` 中合并外部 MCU 请求：

```c
bool rotationRequested = KEY_WasLongPressed() ||
                         HostLink_TakeRotationCalibrationRequest() ||
                         ExternalIMULink_TakeRotationCalibrationRequest();
bool zeroRequested = KEY_WasShortPressed() ||
                     HostLink_TakeZeroCalibrationRequest() ||
                     ExternalIMULink_TakeZeroCalibrationRequest();
bool autoCRequested = HostLink_TakeAutoCRequest() ||
                      ExternalIMULink_TakeAutoCRequest();
bool angleResetRequested = HostLink_TakeAngleResetRequest() ||
                           ExternalIMULink_TakeAngleResetRequest();
```

- [ ] 保持现有优先级不变：

```text
1080 标定 > AutoC > 零飘 > 角度归零
```

---

## 任务 6：验证和文档同步

**文件：**

- 修改 `Docs/IMU_EXTERNAL_UART_CAN_INTERFACE_RESEARCH.md`
- 可选新建 `Docs/IMU_EXTERNAL_LINK_PROTOCOL_DRAFT.md`

步骤：

- [ ] 在调研文档中补充最终采用的高层方向：

```text
JY901 兼容模式：
- 只输出数据，不接收 JY901 配置命令。
- 输出 0x55 0x52 角速度帧。
- 输出 0x55 0x53 角度帧。

自定义模式：
- A5 5A 二进制帧。
- command / response / data 三类帧。
- CRC16-CCITT。
- 默认 100 Hz 发送轻量状态/解算数据帧，不发送陀螺仪 raw。
- UART 先实现，CAN 后续映射。
```

- [ ] 构建验证：

```powershell
eide build
```

期望：构建成功。

- [ ] 外部 UART 烟雾测试：

```text
PB15 = UART_EXTERNAL_TX
PB16 = UART_EXTERNAL_RX
baud = 460800
format = 8N1
```

期望：

```text
启用 JY901 兼容流后，能看到 55 52 和 55 53 JY901 兼容帧，SUM 正确，频率约 10 Hz。
能看到 A5 5A 自定义数据帧，CRC16 正确。
发送 PING，收到 RESPONSE/OK。
发送 ZERO_CAL，收到 RESPONSE/OK，并进入零飘状态。
发送 ANGLE_ZERO，收到 RESPONSE/OK，角度归零但标定值不归零。
```

---

## 执行建议

建议按任务顺序做，不要先把 `app.c` 接进去：

1. 先完成协议编码/解析。
2. 再完成 JY901 兼容输出。
3. 再完成自定义数据帧。
4. 再新建外部通信任务。
5. 最后接入 app 主流程。

这样每一步都比较容易回滚和验证，也不会影响现在 HTML 调试链路。
