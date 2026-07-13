#include "bsp_pwm.h"

#include "ti_msp_dl_config.h"

#define BSP_PWM_MAX_DUTY_PERMILLE (1000U)
#define BSP_PWM_PERMILLE_PER_PERCENT (10U)

static uint16_t g_dutyPermille;

static void BSP_PWM_ForceOutputLow(void)
{
    DL_TimerG_setCCPOutputDisabled(PWM_OUTPUT_INST,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

void BSP_PWM_Init(void)
{
    BSP_PWM_ForceOutputLow();
    DL_TimerG_stopCounter(PWM_OUTPUT_INST);
    g_dutyPermille = 0U;
}

bool BSP_PWM_SetDutyPercent(uint8_t dutyPercent)
{
    if (dutyPercent > 100U) {
        return false;
    }

    return BSP_PWM_SetDutyPermille(
        (uint16_t) dutyPercent * BSP_PWM_PERMILLE_PER_PERCENT);
}

uint8_t BSP_PWM_GetDutyPercent(void)
{
    return (uint8_t) ((g_dutyPermille +
        (BSP_PWM_PERMILLE_PER_PERCENT / 2U)) /
        BSP_PWM_PERMILLE_PER_PERCENT);
}

bool BSP_PWM_SetDutyPermille(uint16_t dutyPermille)
{
    uint32_t periodTicks;
    uint32_t highTicks;
    uint32_t compareValue;

    if (dutyPermille > BSP_PWM_MAX_DUTY_PERMILLE) {
        return false;
    }

    if (dutyPermille == 0U) {
        BSP_PWM_ForceOutputLow();
        DL_TimerG_stopCounter(PWM_OUTPUT_INST);
        g_dutyPermille = 0U;
        return true;
    }

    periodTicks = DL_TimerG_getLoadValue(PWM_OUTPUT_INST) + 1U;
    highTicks =
        (periodTicks * dutyPermille) / BSP_PWM_MAX_DUTY_PERMILLE;
    compareValue = periodTicks - highTicks;

    DL_TimerG_setCaptureCompareValue(
        PWM_OUTPUT_INST, compareValue, GPIO_PWM_OUTPUT_C0_IDX);
    DL_TimerG_startCounter(PWM_OUTPUT_INST);
    DL_TimerG_setCCPOutputDisabled(PWM_OUTPUT_INST,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);

    g_dutyPermille = dutyPermille;
    return true;
}

uint16_t BSP_PWM_GetDutyPermille(void)
{
    return g_dutyPermille;
}
