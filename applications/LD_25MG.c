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

static rt_thread_t tid_6050 = RT_NULL;
static rt_thread_t tid_25mg = RT_NULL;

struct rt_device_pwm *pwm_dev_level = RT_NULL;        /* PWM设备句柄 */
struct rt_device_pwm *pwm_dev_vertical = RT_NULL;     /* PWM设备句柄 */

//extern static rt_mailbox_t mailbox_X;
//extern static rt_mailbox_t mailbox_Y;

rt_uint32_t KalmanAngle_25mg_X;
rt_uint32_t KalmanAngle_25mg_Y;

rt_int16_t KalmanAngle_25mg_X_16;
rt_int16_t KalmanAngle_25mg_Y_16;

//static int LD_25MG_init()
//{
//    /* 查找设备level */
//    pwm_dev_level = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_level);
//    if (pwm_dev_level == RT_NULL)
//    {
//        rt_kprintf("LD_25MG level run failed! can't find %s device!\n", PWM_DEV_NAME_vertical);
//        return RT_ERROR;
//    }
//    /* 查找设备vertical */
//    pwm_dev_vertical = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_level);
//    if (pwm_dev_vertical == RT_NULL)
//    {
//        rt_kprintf("LD_25MG level run failed! can't find %s device!\n", PWM_DEV_NAME_vertical);
//        return RT_ERROR;
//    }
//}
//MSH_CMD_EXPORT(LD_25MG_init, LD_25MG_init);

void LD_25MG_set(rt_uint32_t pulse_level, rt_uint32_t pulse_vertical)//us
{
    pulse_level *= 1000;
    pulse_vertical *= 1000;
    rt_pwm_set(pwm_dev_level, PWM_DEV_CHANNEL_level, LD_25MG_period, pulse_level);
    rt_pwm_set(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical, LD_25MG_period, pulse_vertical);
}

//void LD_25MG_set(rt_uint32_t num, rt_uint32_t pulse)//us
//{
//    pulse *= 1000;
//    if(num == 1)
//        rt_pwm_set(pwm_dev_level, PWM_DEV_CHANNEL_level, LD_25MG_period, pulse);
//    else
//        rt_pwm_set(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical, LD_25MG_period, pulse);
//}

void LD_25MG_enable()
{
    rt_pwm_enable(pwm_dev_level, PWM_DEV_CHANNEL_level);
    rt_pwm_enable(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical);
}

void LD_25MG_disable()
{
    rt_pwm_disable(pwm_dev_level, PWM_DEV_CHANNEL_level);
    rt_pwm_disable(pwm_dev_vertical, PWM_DEV_CHANNEL_vertical);
}

//static void pwm_test()
//{
//    rt_uint32_t i = 500, a = 1;
//    LD_25MG_init();
//    LD_25MG_set(1500, 1500);
//    LD_25MG_enable();
//    while (1)
//    {
//        LD_25MG_set(i, i);
//        if(a)
//        {
//            i+=30;
//            if(i>=2500)
//            {
//                i=2500;
//                a = 0;
//            }
//
//        }
//        else
//        {
//            i-=30;
//            if(i<=500)
//            {
//                i = 500;
//                a = 1;
//            }
//        }
//
//        rt_thread_mdelay(200);
//    }
//}
//MSH_CMD_EXPORT(pwm_test, pwm pwm_test);

///* 线程 25mg 的入口函数 */
//static void thread_25mg_entry(void *parameter)
//{
//    rt_uint32_t i = 500, a = 1;
//    LD_25MG_init();
//    LD_25MG_set(1500, 1500);
//    LD_25MG_enable();
//    while (1)
//    {
//        rt_mb_recv (mailbox_X, KalmanAngle_25mg_X, RT_WAITING_FOREVER);
//        rt_mb_recv (mailbox_Y, KalmanAngle_25mg_Y, RT_WAITING_FOREVER);
//        KalmanAngle_25mg_X_16 = (rt_int16_t)KalmanAngle_25mg_X;
//        KalmanAngle_25mg_Y_16 = (rt_int16_t)KalmanAngle_25mg_Y;
//        rt_kprintf("x:%d y:%d\n", KalmanAngle_25mg_X_16, KalmanAngle_25mg_Y_16);
//    }
//    LD_25MG_disable();
//}
//
///* 线程示例 */
//int thread_sam25mg(void)
//{
//
//    /* 创建线程 2，名称是 thread_6050，入口是 thread_6050_entry*/
//    tid_25mg = rt_thread_create("thread_25mg",
//                            thread_25mg_entry, RT_NULL,
//                            THREAD_STACK_SIZE_25mg,
//                            THREAD_PRIORITY_25mg, THREAD_TIMESLICE_25mg);
//
//    /* 如果获得线程控制块，启动这个线程 */
//
//    if (tid_25mg != RT_NULL)
//            rt_thread_startup(tid_25mg);
//    else
//        rt_kprintf("thread_25mg open fail");
//
//
//    return 0;
//}
//
///* 导出到 msh 命令列表中 */
//MSH_CMD_EXPORT(thread_sam25mg, thread_sample_25mg);
