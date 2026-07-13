# Algorithm

存放滤波、姿态解算、传感器融合和 CMSIS-DSP 算法代码。

算法层只处理数据，不直接访问 GPIO、SPI、I2C 等硬件外设。

## PID

`pid.c/.h` 是对 CMSIS-DSP `arm_pid_f32` 的浮点 PID 封装，提供：

- 连续时间 `Kp/Ki/Kd` 到离散 PID 参数的采样周期换算。
- 输出限幅，可直接对应加热 PWM 的 `0~100%` 占空比。
- 限幅后的状态跟踪，减少积分饱和。
- 复位和非法数值检查。

温控调用示例（PID 参数需要实机整定）：

```c
static PID_Controller g_temperaturePid;

static const PID_Config g_temperaturePidConfig = {
    .kp = TEMP_PID_KP,
    .ki = TEMP_PID_KI,
    .kd = TEMP_PID_KD,
    .samplePeriodS = 0.125f,
    .outputMin = 0.0f,
    .outputMax = 100.0f,
};

PID_Init(&g_temperaturePid, &g_temperaturePidConfig);

/* Call once every 125 ms. */
float dutyPercent;
if (PID_Compute(&g_temperaturePid, targetC, measuredC, &dutyPercent)) {
    Heater_SetDutyPercent((uint8_t) (dutyPercent + 0.5f));
}
```
