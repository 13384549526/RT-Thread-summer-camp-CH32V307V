/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-31     ZangCH       the first version
 */
#ifndef APPLICATIONS_LD_25MG_H_
#define APPLICATIONS_LD_25MG_H_

#include "kalman.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include <stdint.h>
#include "sensor_inven_mpu6xxx.h"
#include "mpu6xxx.h"

void LD_25MG_set(rt_uint32_t pulse_level, rt_uint32_t pulse_vertical, struct rt_device_pwm *pwm_dev_level, struct rt_device_pwm *pwm_dev_vertical);
void LD_25MG_enable(struct rt_device_pwm *pwm_dev_level, struct rt_device_pwm *pwm_dev_vertical);
void LD_25MG_disable(struct rt_device_pwm *pwm_dev_level, struct rt_device_pwm *pwm_dev_vertical);


#endif /* APPLICATIONS_LD_25MG_H_ */
