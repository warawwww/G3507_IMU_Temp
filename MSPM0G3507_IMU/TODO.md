# TODO

## Type-C 上位机通信开关

- [x] 默认静默：设备上电后不主动通过 Type-C UART 连续发送 TMP、heater control、heater state 等调试帧。
- [x] 保持 TMP 采样和温控控制周期不变：TMP 仍按 125 ms 采样，heater 仍按控制周期运行，但串口上报开关独立控制。
- [x] 增加上位机握手命令：设备只有收到 `START`/`HELLO`/`ON` 后，才开始连续发送调试数据。
- [ ] 支持按数据类型订阅：例如 TMP 温度、heater 状态、heater 控制样本、后续陀螺仪数据分别开关。
- [ ] 支持上报频率配置：例如 `SUB TMP 1000` 表示 TMP 每 1000 ms 上报一次，而不是每次采样都上报。
- [x] 支持停止上报和超时静默：收到 `STOP`/`OFF` 后停止发送；长时间没有上位机 `PING` 时自动回到静默模式。

## 算法性能优化

- [ ] 后续陀螺仪姿态解算、滤波、三角函数、平方根、atan2、定点矩阵/向量等复杂计算，优先评估使用 MSPM0 MATHACL 或 TI IQmath mathacl 库加速；当前温控 PID 计算量很小，暂不为它专门改 MATHACL。

## 当前状态

- [ ] SPI 尚未完成实物联调：XV7021BB 初始化与角速度数据读取结果未确认。


## 后续：DMA

- [x] SPI 陀螺仪收发已改为 DMA 搬运，当前保持同步等待接口，适合现阶段周期性读取 XV7021BB。
- [ ] SPI DMA 仍需示波器/逻辑分析仪确认：`5 MHz` SCLK、CS 时序、MOSI/MISO 边沿、DMA 完成时刻和 `500 us` 采样抖动。
- [ ] 评估 I2C DMA：保留 START、Repeated START、STOP 和 NACK 错误处理；DMA 仅负责 FIFO 数据搬运。
- [ ] 为 DMA 完成、超时、NACK/仲裁丢失等状态设计统一的回调或状态机接口。
