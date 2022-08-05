/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-31     Misth       the first version
 */
#include "LD_25MG.h"


#define PWM_DEV_NAME_level          "pwm3"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL_level       2       /* PWM通道 */

#define PWM_DEV_NAME_vertical       "pwm3"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL_vertical    1       /* PWM通道 */

#define LD_25MG_period              20000000



#define THREAD_PRIORITY_25mg            25
#define THREAD_STACK_SIZE_25mg          512
#define THREAD_TIMESLICE_25mg           10

rt_uint32_t KalmanAngle_25mg_X;
rt_uint32_t KalmanAngle_25mg_Y;

rt_int16_t KalmanAngle_25mg_X_16;
rt_int16_t KalmanAngle_25mg_Y_16;

void LD_25MG_set(rt_uint32_t pulse_level, rt_uint32_t pulse_vertical, struct rt_device_pwm *pwm_dev_level, struct rt_device_pwm *pwm_dev_vertical)//us
{
    pulse_level *= 1000;
    pulse_vertical *= 1000;
    rt_pwm_set(pwm_dev_level, PWM_DEV_CHANNEL_level, LD_25MG_period, pulse_level);
    rt_pwm_set(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical, LD_25MG_period, pulse_vertical);
}

void LD_25MG_enable(struct rt_device_pwm *pwm_dev_level, struct rt_device_pwm *pwm_dev_vertical)
{
    rt_pwm_enable(pwm_dev_level, PWM_DEV_CHANNEL_level);
    rt_pwm_enable(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical);
}

void LD_25MG_disable(struct rt_device_pwm *pwm_dev_level, struct rt_device_pwm *pwm_dev_vertical)
{
    rt_pwm_disable(pwm_dev_level, PWM_DEV_CHANNEL_level);
    rt_pwm_disable(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical);
}
