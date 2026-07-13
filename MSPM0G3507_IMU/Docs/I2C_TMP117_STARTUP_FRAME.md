# TMP117 启动时 Device ID 读取帧

`TMP117_Init()` 读取 TMP117 的 Device ID 寄存器 `0x0F`。正常读取结果为
`0x0117`。

I2C 采用开漏输出：发送方发送 `0` 时主动把 SDA 拉低；发送 `1` 时释放
SDA，由上拉电阻将其拉高。除 START 和 STOP 外，SDA 在 SCL 低电平时变化，
在 SCL 高电平时由接收方采样。

## 帧总览

```text
MCU:    S  0x90  ──>  0x0F  ──>  Sr  0x91  ──>
TMP117:       ACK          ACK          ACK  0x01  ──>  0x17  ──>
MCU:                                                  ACK          NACK  P
```

其中：

- `S`：START；`Sr`：Repeated START；`P`：STOP。
- `0x90` 是七位设备地址 `0x48` 加写方向位（`W = 0`）。
- `0x91` 是七位设备地址 `0x48` 加读方向位（`R = 1`）。
- `0x0F` 是 TMP117 内部的 Device ID 寄存器地址。
- `0x01 0x17` 是 TMP117 返回的 16 位 Device ID。

## 每一段由谁控制

| 阶段 | SDA 在 SCL 高电平时的位值 | SDA 驱动方 | 含义 |
|---|---|---|---|
| 总线空闲 | `1` | 无主动驱动，上拉电阻 | SDA、SCL 都为高电平 |
| START | SDA 在 SCL 为高时由 `1 -> 0` | MCU | 开始一次 I2C 传输 |
| 地址 + 写 | `1 0 0 1 0 0 0 0` | MCU | `0x90`：地址 `0x48`，写方向 |
| 地址 ACK | `0` | TMP117 | TMP117 确认自己响应地址 `0x48` |
| 寄存器地址 | `0 0 0 0 1 1 1 1` | MCU | `0x0F`：选择 TMP117 内部 Device ID 寄存器 |
| 寄存器 ACK | `0` | TMP117 | TMP117 确认已接收寄存器地址 |
| Repeated START | SDA 在 SCL 为高时由 `1 -> 0` | MCU | 切换为读操作，期间不释放总线 |
| 地址 + 读 | `1 0 0 1 0 0 0 1` | MCU | `0x91`：地址 `0x48`，读方向 |
| 读地址 ACK | `0` | TMP117 | TMP117 确认准备发送数据 |
| ID 高字节 | `0 0 0 0 0 0 0 1` | TMP117 | `0x01` |
| 高字节 ACK | `0` | MCU | MCU 确认已收到高字节，要求继续发送 |
| ID 低字节 | `0 0 0 1 0 1 1 1` | TMP117 | `0x17` |
| 低字节 NACK | `1` | MCU（释放 SDA） | MCU 表示已经读完，不再请求后续字节；这是正常结束信号 |
| STOP | SDA 在 SCL 为高时由 `0 -> 1` | MCU | 结束传输并释放总线 |

## 示波器判断要点

1. `0x90` 后第 9 个时钟必须看到 TMP117 将 SDA 拉低（ACK）。若为高电平，表示地址阶段未收到应答。
2. `0x0F` 后也应看到 TMP117 ACK。
3. `0x91` 后应再次看到 TMP117 ACK。
4. 两个数据字节应为 `0x01`、`0x17`。
5. 最后一个字节后的 NACK 是 MCU 发出的正常结束信号，不能作为 TMP117 未应答的证据。

## 代码对应关系

```text
APP_Init()
  -> TMP117_Init()
     -> TMP117_ReadRegister(0x0F)
        -> BSP_I2C_WriteRead(0x48, &reg, 1, data, 2)
           -> BSP_I2C_Transfer()
              -> I2C0_IRQHandler()
```

`BSP_I2C_Transfer()` 由 MCU 发出写阶段。发送完成的 `TX_DONE` 中断到来后，
`I2C0_IRQHandler()` 再由 MCU 发出读阶段的 Repeated START。TMP117 只在其地址
被选中后，对 ACK 位和返回数据字节进行驱动。
