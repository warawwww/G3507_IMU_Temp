#include "bsp_pwm.h"

#include "ti_msp_dl_config.h"

#define BSP_PWM_MAX_DUTY_PERCENT (100U)

static uint8_t g_dutyPercent;

static void BSP_PWM_ForceOutputLow(void)
{
    DL_TimerG_setCCPOutputDisabled(PWM_OUTPUT_INST,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

void BSP_PWM_Init(void)
{
    BSP_PWM_ForceOutputLow();
    DL_TimerG_stopCounter(PWM_OUTPUT_INST);
    g_dutyPercent = 0U;
}

bool BSP_PWM_SetDutyPercent(uint8_t dutyPercent)
{
    uint32_t periodTicks;
    uint32_t highTicks;
    uint32_t compareValue;

    if (dutyPercent > BSP_PWM_MAX_DUTY_PERCENT) {
        return false;
    }

    if (dutyPercent == 0U) {
        BSP_PWM_ForceOutputLow();
        DL_TimerG_stopCounter(PWM_OUTPUT_INST);
        g_dutyPercent = 0U;
        return true;
    }

    periodTicks = DL_TimerG_getLoadValue(PWM_OUTPUT_INST) + 1U;
    highTicks = (periodTicks * dutyPercent) / BSP_PWM_MAX_DUTY_PERCENT;
    compareValue = periodTicks - highTicks;

    DL_TimerG_setCaptureCompareValue(
        PWM_OUTPUT_INST, compareValue, GPIO_PWM_OUTPUT_C0_IDX);
    DL_TimerG_startCounter(PWM_OUTPUT_INST);
    DL_TimerG_setCCPOutputDisabled(PWM_OUTPUT_INST,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);

    g_dutyPercent = dutyPercent;
    return true;
}

uint8_t BSP_PWM_GetDutyPercent(void)
{
    return g_dutyPercent;
}
