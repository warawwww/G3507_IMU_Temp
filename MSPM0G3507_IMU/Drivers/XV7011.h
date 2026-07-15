#ifndef DRIVERS_XV7011_H
#define DRIVERS_XV7011_H

#include <stdbool.h>
#include <stdint.h>

#define XV7011_ANGULAR_RATE_LSB_PER_DPS (280.0f)
#define XV7011_TEMPERATURE_LSB_PER_C    (16.0f)

typedef enum {
    XV7011_STATUS_NOT_INITIALIZED = 0,
    XV7011_STATUS_OK,
    XV7011_STATUS_INVALID_ARGUMENT,
    XV7011_STATUS_NOT_FOUND,
    XV7011_STATUS_BUS_BUSY,
    XV7011_STATUS_BUS_TIMEOUT,
    XV7011_STATUS_BUS_ERROR,
    XV7011_STATUS_CONFIGURATION_ERROR,
} XV7011_Status;

typedef enum {
    XV7011_LPF_ORDER_2 = 0,
    XV7011_LPF_ORDER_3,
    XV7011_LPF_ORDER_4,
} XV7011_LpfOrder;

typedef enum {
    XV7011_LPF_10_HZ = 0,
    XV7011_LPF_35_HZ,
    XV7011_LPF_45_HZ,
    XV7011_LPF_50_HZ,
    XV7011_LPF_70_HZ,
    XV7011_LPF_85_HZ,
    XV7011_LPF_100_HZ,
    XV7011_LPF_140_HZ,
    XV7011_LPF_175_HZ,
    XV7011_LPF_200_HZ,
    XV7011_LPF_285_HZ,
    XV7011_LPF_345_HZ,
    XV7011_LPF_400_HZ,
    XV7011_LPF_500_HZ,
} XV7011_LpfFrequency;

typedef enum {
    XV7011_HPF_0_01_HZ = 0,
    XV7011_HPF_0_03_HZ,
    XV7011_HPF_0_1_HZ,
    XV7011_HPF_0_3_HZ,
    XV7011_HPF_1_HZ,
    XV7011_HPF_3_HZ,
    XV7011_HPF_10_HZ,
} XV7011_HpfFrequency;

/*
 * 初始化陀螺仪接口和输出格式。
 * 当前配置为 4 线 SPI、16 位角速度输出、12 位温度输出，
 * 最后等待手册要求的 200 ms 角速度启动时间。
 */
XV7011_Status XV7011_Init(void);

XV7011_Status XV7011_ReadRegister(uint8_t reg, uint8_t *value);
XV7011_Status XV7011_WriteRegister(uint8_t reg, uint8_t value);

XV7011_Status XV7011_ReadStatus(uint8_t *status);
XV7011_Status XV7011_ReadAngularRateRaw(int16_t *rawAngularRate);
XV7011_Status XV7011_ReadAngularRateDps(float *angularRateDps);
XV7011_Status XV7011_ReadTemperatureRaw(int16_t *rawTemperature);
XV7011_Status XV7011_ReadTemperatureC(float *temperatureC);
/* 返回当前板子固定使用的 multi-slave SPI 地址位 A[6:5]。 */
uint8_t XV7011_GetSpiAddressBits(void);

XV7011_Status XV7011_SetLowPassFilter(
    XV7011_LpfOrder order, XV7011_LpfFrequency frequency);
XV7011_Status XV7011_SetHighPassFilter(
    bool enable, XV7011_HpfFrequency frequency);

/*
 * 发送 SlpIn 命令，让陀螺仪进入 sleep 模式以降低功耗。
 * 再次读取角速度前需要先调用 XV7011_Wake() 唤醒。
 */
XV7011_Status XV7011_Sleep(void);

/*
 * 发送 Stby 命令，让陀螺仪进入 standby 模式。
 * 再次读取角速度前需要先调用 XV7011_Wake() 唤醒。
 */
XV7011_Status XV7011_Standby(void);

/*
 * 发送 SlpOut 命令，让陀螺仪退出 sleep/standby，并复位内部 DSP 通路。
 * 驱动会等待手册要求的启动时间后再返回。
 */
XV7011_Status XV7011_Wake(void);

/*
 * 发送 SWRst 命令，复位用户命令寄存器。
 * 命令完成后驱动会重新执行 XV7011_Init()，恢复接口和输出格式配置。
 */
XV7011_Status XV7011_SoftwareReset(void);

/*
 * 使能并发送 AutoC 零速率校准命令。
 * 调用时需要保持板子静止，让陀螺仪更新零速率参考值。
 */
XV7011_Status XV7011_CalibrateZeroRate(void);

#endif
