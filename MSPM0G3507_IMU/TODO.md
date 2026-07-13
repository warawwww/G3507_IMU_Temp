# TODO

## 当前状态

- [ ] I2C 尚未完成实物联调：TMP117 是否应答、设备 ID 是否能读到 `0x0117` 均未确认。
- [ ] SPI 尚未完成实物联调：XV7011 初始化与角速度数据读取结果未确认。

## 下一步：示波器分析 I2C

- [ ] 观察 SDA（PA0）和 SCL（PA1）空闲时是否均为高电平，确认外部上拉、供电和共地正常。
- [ ] 捕获 TMP117 初始化的读 Device ID 时序：
  `START -> 0x90 + ACK -> 0x0F + ACK -> Repeated START -> 0x91 + ACK -> 0x01 + ACK -> 0x17 + NACK -> STOP`。
- [ ] 若地址字节 `0x90` 或 `0x91` 后没有从机 ACK，检查 TMP117 的 ADD0 地址配置（`0x48` 至 `0x4B`）、连线和供电。
- [ ] 记录 `TMP117_Init()` 返回值，区分 `NOT_FOUND`、`TIMEOUT`、`BUS_BUSY` 和 `DEVICE_ID_MISMATCH`。

## 后续：DMA

- [ ] I2C/SPI 基础收发和示波器波形确认正常后，再评估 DMA 方案。
- [ ] SPI 优先改为 DMA：适合周期性读取陀螺仪数据，降低 CPU 在收发过程中的等待时间。
- [ ] 评估 I2C DMA：保留 START、Repeated START、STOP 和 NACK 错误处理；DMA 仅负责 FIFO 数据搬运。
- [ ] 为 DMA 完成、超时、NACK/仲裁丢失等状态设计统一的回调或状态机接口。
