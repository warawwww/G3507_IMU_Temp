#ifndef DRIVERS_XV7021_H
#define DRIVERS_XV7021_H

#include <stdbool.h>
#include <stdint.h>

#define XV7021_ANGULAR_RATE_16BIT_LSB_PER_DPS (70.0f)
#define XV7021_ANGULAR_RATE_LSB_PER_DPS       XV7021_ANGULAR_RATE_16BIT_LSB_PER_DPS
#define XV7021_ANGULAR_RATE_24BIT_LSB_PER_DPS (17920.0f)
#define XV7021_TEMPERATURE_LSB_PER_C          (16.0f)
#define XV7021_RATE_RANGE_DPS                 (400.0f)

typedef enum {
    XV7021_STATUS_NOT_INITIALIZED = 0,
    XV7021_STATUS_OK,
    XV7021_STATUS_INVALID_ARGUMENT,
    XV7021_STATUS_NOT_FOUND,
    XV7021_STATUS_BUS_BUSY,
    XV7021_STATUS_BUS_TIMEOUT,
    XV7021_STATUS_BUS_ERROR,
    XV7021_STATUS_CONFIGURATION_ERROR,
} XV7021_Status;

typedef enum {
    XV7021_LPF_ORDER_2 = 0,
    XV7021_LPF_ORDER_3,
    XV7021_LPF_ORDER_4,
} XV7021_LpfOrder;

typedef enum {
    XV7021_LPF_10_HZ = 0,
    XV7021_LPF_35_HZ,
    XV7021_LPF_45_HZ,
    XV7021_LPF_50_HZ,
    XV7021_LPF_70_HZ,
    XV7021_LPF_85_HZ,
    XV7021_LPF_100_HZ,
    XV7021_LPF_140_HZ,
    XV7021_LPF_175_HZ,
    XV7021_LPF_200_HZ,
    XV7021_LPF_285_HZ,
    XV7021_LPF_345_HZ,
    XV7021_LPF_400_HZ,
    XV7021_LPF_500_HZ,
} XV7021_LpfFrequency;

typedef enum {
    XV7021_HPF_0_01_HZ = 0,
    XV7021_HPF_0_03_HZ,
    XV7021_HPF_0_1_HZ,
    XV7021_HPF_0_3_HZ,
    XV7021_HPF_1_HZ,
    XV7021_HPF_3_HZ,
    XV7021_HPF_10_HZ,
} XV7021_HpfFrequency;

typedef enum {
    XV7021_RATE_DATA_FORMAT_16BIT = 0,
    XV7021_RATE_DATA_FORMAT_24BIT,
} XV7021_RateDataFormat;

/*
 * 初始化 XV7021BB 的接口和输出格式。
 * 当前配置为 4 线 SPI、24 位角速度输出、12 位温度输出，
 * 最后等待手册要求的 200 ms 角速度启动时间。
 */
XV7021_Status XV7021_Init(void);

XV7021_Status XV7021_ReadRegister(uint8_t reg, uint8_t *value);
XV7021_Status XV7021_WriteRegister(uint8_t reg, uint8_t value);

XV7021_Status XV7021_ReadStatus(uint8_t *status);
XV7021_Status XV7021_SetAngularRateDataFormat(
    XV7021_RateDataFormat format);
XV7021_Status XV7021_ReadAngularRateRaw(int16_t *rawAngularRate);
XV7021_Status XV7021_ReadAngularRateRaw24(int32_t *rawAngularRate);
XV7021_Status XV7021_ReadAngularRateDps(float *angularRateDps);
XV7021_Status XV7021_ReadAngularRateDps24(float *angularRateDps);
XV7021_Status XV7021_ReadTemperatureRaw(int16_t *rawTemperature);
XV7021_Status XV7021_ReadTemperatureC(float *temperatureC);

/* 返回当前板子固定使用的 multi-slave SPI 地址位 A[6:5]。 */
uint8_t XV7021_GetSpiAddressBits(void);

XV7021_Status XV7021_SetLowPassFilter(
    XV7021_LpfOrder order, XV7021_LpfFrequency frequency);
XV7021_Status XV7021_SetHighPassFilter(
    bool enable, XV7021_HpfFrequency frequency);

/*
 * 发送 SlpIn 命令，让陀螺仪进入 sleep 模式以降低功耗。
 * 再次读取角速度前需要先调用 XV7021_Wake() 唤醒。
 */
XV7021_Status XV7021_Sleep(void);

/*
 * 发送 Stby 命令，让陀螺仪进入 standby 模式。
 * 再次读取角速度前需要先调用 XV7021_Wake() 唤醒。
 */
XV7021_Status XV7021_Standby(void);

/*
 * 发送 SlpOut 命令，让陀螺仪退出 sleep/standby，并复位内部 DSP 通路。
 * 驱动会等待手册要求的启动时间后再返回。
 */
XV7021_Status XV7021_Wake(void);

/*
 * 发送 SWRst 命令，复位用户命令寄存器。
 * 命令完成后驱动会重新执行 XV7021_Init()，恢复接口和输出格式配置。
 */
XV7021_Status XV7021_SoftwareReset(void);

/*
 * 使能并发送 AutoC 零速率校准命令。
 * 调用时需要保持板子静止，让陀螺仪更新零速率参考值。
 */
XV7021_Status XV7021_CalibrateZeroRate(void);

#endif
