/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include <rtthread.h>
#include <rthw.h>
#include <board.h>
#include <rtdevice.h>
#include "ch32v30x.h"
#include "drivers/pin.h"

#include "kalman.h"
#include "LD_25MG.h"
#include "sensor_inven_mpu6xxx.h"
/* Global typedef */

/* Global define */

/* Global Variable */
#define THREAD_PRIORITY         25
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        10

static rt_sem_t dynamic_sem = RT_NULL;      /* 信号量 */

/*********************************************************************/
/* 线程 6050 的入口函数 */
static void thread_6050_entry(void *parameter)
{
    struct mpu6xxx_device *dev;
    struct IMU_Parameter IMU_Data1, IMU_Data2, IMU_Data3, IMU_Data4;
    struct rt_device_pwm *pwm_dev_level, *pwm_dev_vertical;
    static rt_err_t result;
    rt_uint32_t i, i1, i2;

    dev = mpu6xxx_init("i2c1", RT_NULL);
    pwm_dev_level = (struct rt_device_pwm *)rt_device_find("pwm3");
    pwm_dev_vertical = (struct rt_device_pwm *)rt_device_find("pwm3");

    if (dev == RT_NULL)
        rt_kprintf("mpu6xxx init failed\n");
    else
        rt_kprintf("mpu6xxx init succeed\n");
    if (pwm_dev_level == RT_NULL)
    {
        rt_kprintf("LD_25MG level run failed!\n");
    }
    if (pwm_dev_vertical == RT_NULL)
    {
        rt_kprintf("LD_25MG level run failed!\n");
    }

    Kalman_get_value(dev, &IMU_Data1);
    Kalman_get_value(dev, &IMU_Data2);
    LD_25MG_set(1500, 1500, pwm_dev_level, pwm_dev_vertical);
    LD_25MG_enable(pwm_dev_level, pwm_dev_vertical);
    while (1)
    {
        IMU_Data2 = IMU_Data1;
        Kalman_get_value(dev, &IMU_Data1);
        result = rt_sem_trytake(dynamic_sem);
        if (result == RT_EOK &&
            (IMU_Data1.KalmanAngleX == IMU_Data2.KalmanAngleX) &&
            (IMU_Data1.KalmanAngleY == IMU_Data2.KalmanAngleY))
        {
            i1 = 1500 + ((2000 * IMU_Data1.KalmanAngleX) / 270);
            if(i1>2500)i1=2500;
            if(i1<500)i1=500;
            if(IMU_Data1.KalmanAngleX < 15 && IMU_Data1.KalmanAngleX > -15)
                i1 = 1500;
            i2 = 1500 + ((2000 * IMU_Data1.KalmanAngleY) / 270);
            if(i2>2500)i2=2500;
            if(i2<500)i2=500;
            if(IMU_Data1.KalmanAngleY < 15 && IMU_Data1.KalmanAngleY > -15)
                i2 = 1500;
            rt_kprintf("");
            rt_kprintf("KalmanX = %4d, KalmanY = %4d, i1 = %4d, i2 = %4d\n", IMU_Data1.KalmanAngleX, IMU_Data1.KalmanAngleY, i1, i2);

            LD_25MG_set(i1, i2, pwm_dev_level, pwm_dev_vertical);
         }
    }
}
/* 线程 25mg 的入口函数 */
static void thread_25mg_entry(void *parameter)
{

    while(1)
    {
        rt_thread_mdelay(1000);
        rt_sem_release(dynamic_sem);
    }
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    static rt_thread_t tid_6050 = RT_NULL, tid_25mg = RT_NULL;

    rt_kprintf("MCU: CH32V307\n");
	rt_kprintf("SysClk: %dHz\n",SystemCoreClock);
    rt_kprintf("www.wch.cn\n");

    dynamic_sem = rt_sem_create("dsem", 1, RT_IPC_FLAG_PRIO);

    tid_6050 = rt_thread_create("thread_6050", thread_6050_entry, RT_NULL,
                            1024, 25, THREAD_TIMESLICE);
    tid_25mg = rt_thread_create("thread_25mg", thread_25mg_entry, RT_NULL,
                            1024, 25, THREAD_TIMESLICE);

    /* 如果获得线程控制块，启动这个线程 */
    if (tid_6050 != RT_NULL)
    {
        rt_thread_startup(tid_6050);
        rt_kprintf("thread_6050 open success");
    }
    else
        rt_kprintf("thread_6050 open fail");

    if (tid_25mg != RT_NULL)
        rt_thread_startup(tid_25mg);
    else
        rt_kprintf("thread_25mg open fail");
}




int rt_hw_mpu6xxx_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = "i2c1";
    cfg.intf.user_data = (void *)MPU6XXX_ADDR_DEFAULT;
    cfg.irq_pin.pin = RT_PIN_NONE;
    rt_hw_mpu6xxx_init("mpu", &cfg);
    return 0;
}
INIT_APP_EXPORT(rt_hw_mpu6xxx_port);
