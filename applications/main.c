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

#define MP_BLOCK_SIZE       RT_ALIGN(sizeof(rt_int16_t)*9, sizeof(intptr_t)) /* 为了字节对齐 */
#define MB_LEN              (4)
#define MP_LEN              MB_LEN

IMU_Parameter IMU_Data;
static rt_thread_t tid_6050 = RT_NULL;
static rt_thread_t tid_25mg = RT_NULL;
rt_uint32_t i1 = 500, i2 = 500;


static rt_mailbox_t tmp_msg_mb;           /* 邮箱 */
static rt_mp_t tmp_msg_mp;                /* 内存池 */
static struct rt_sensor_data sensor_data; /* sensor结构体 */

//extern struct rt_device_pwm *pwm_dev_level;        /* PWM设备句柄 */
//extern struct rt_device_pwm *pwm_dev_vertical;     /* PWM设备句柄 */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    rt_kprintf("MCU: CH32V307\n");
	rt_kprintf("SysClk: %dHz\n",SystemCoreClock);
    rt_kprintf("www.wch.cn\n");
	LED1_BLINK_INIT();

	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	while(1)
	{
	    GPIO_SetBits(GPIOA,GPIO_Pin_0);
	    rt_thread_mdelay(500);
	    GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	    rt_thread_mdelay(500);
	}
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





/* 线程 6050 的入口函数 */
static void thread_6050_entry(void *parameter)
{

    struct mpu6xxx_device *dev;
    struct mpu6xxx_3axes accel, gyro;
    dev = mpu6xxx_init("i2c1", RT_NULL);
    if (dev == RT_NULL)
        rt_kprintf("mpu6xxx init failed\n");
    else
        rt_kprintf("mpu6xxx init succeed\n");
    while (1)
    {
        mpu6xxx_get_accel(dev, &accel);
        mpu6xxx_get_gyro(dev, &gyro);

        IMU_Data.Accel_X = accel.x;
        IMU_Data.Accel_Y = accel.y;
        IMU_Data.Accel_Z = accel.z;
        IMU_Data.Gyro_X = gyro.x;
        IMU_Data.Gyro_Y = gyro.y;
        IMU_Data.Gyro_Z = gyro.z;
        MPU6050_Read_All(&IMU_Data);

        i1 = 1500 - ((2000 * IMU_Data.KalmanAngleX) / 270);
        if(i1>2500)i1=2500;
        if(i1<500)i1=500;
        if(IMU_Data.KalmanAngleX < 15 && IMU_Data.KalmanAngleX > -15)
            i1 = 1500;

        i2 = 1500 - ((2000 * IMU_Data.KalmanAngleY) / 270);
        if(i2>2500)i2=2500;
        if(i2<500)i2=500;
        if(IMU_Data.KalmanAngleY < 15 && IMU_Data.KalmanAngleY > -15)
            i2 = 1500;

        rt_kprintf("KalmanAngleX = %3d, KalmanAngleY = %3d, i1 = %5d, i2 = %5d\n", IMU_Data.KalmanAngleX, IMU_Data.KalmanAngleY, i1, i2);

        rt_thread_mdelay(2);
    }
}
/* 线程 25mg 的入口函数 */
static void thread_25mg_entry(void *parameter)
{
    rt_uint32_t a = 1;

    pwm_dev_level = (struct rt_device_pwm *)rt_device_find("pwm3");
    if (pwm_dev_level == RT_NULL)
    {
        rt_kprintf("LD_25MG level run failed! can't find %s device!\n", "pwm3");
        return RT_ERROR;
    }
    /* 查找设备vertical */
    pwm_dev_vertical = (struct rt_device_pwm *)rt_device_find("pwm3");
    if (pwm_dev_vertical == RT_NULL)
    {
        rt_kprintf("LD_25MG level run failed! can't find %s device!\n", "pwm3");
        return RT_ERROR;
    }

    LD_25MG_set(1500, 1500);
    LD_25MG_enable();
    while (1)
    {
//        i1 = 1500 - ((2000 * IMU_Data.KalmanAngleX) / 270);
//        if(i1>2500)i1=2500;
//        if(i2>2500)i1=2500;
//        if(IMU_Data.KalmanAngleX < 15 && IMU_Data.KalmanAngleX > -15)
//            i1 = 1500;
//
//        i2 = 1500 - ((2000 * IMU_Data.KalmanAngleY) / 270);
//        if(i1<500)i1=500;
//        if(i2<500)i1=500;
//        if(IMU_Data.KalmanAngleY < 15 && IMU_Data.KalmanAngleY > -15)
//            i2 = 1500;

        LD_25MG_set(i1, i2);
        //LD_25MG_set(2500, 1500);
//        if(i1 != 1500)
//            LD_25MG_set(1, i1);
//        if(i2 != 1500)
//            LD_25MG_set(2, i2);

//        if(a)
//        {
//            i+=30;
//            if(i>=2500)
//            {
//                i=2500;
//                a = 0;
//            }
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
        rt_thread_mdelay(5000);
    }
    LD_25MG_disable();
}

/* 线程示例 */
void thread_sample(void)
{
    tmp_msg_mb = rt_mb_create("temp_mb0", MB_LEN, RT_IPC_FLAG_FIFO); /* 创建邮箱 */
    tmp_msg_mp = rt_mp_create("temp_mp0", MP_LEN, MP_BLOCK_SIZE);    /* 创建内存池 */
    /* 创建线程 1，名称是 thread_6050，入口是 thread_6050_entry*/
    tid_6050 = rt_thread_create("thread_6050",
                            thread_6050_entry, RT_NULL,
                            THREAD_STACK_SIZE,
                            THREAD_PRIORITY, THREAD_TIMESLICE);
    /* 创建线程 2，名称是 thread_6050，入口是 thread_6050_entry*/
    tid_25mg = rt_thread_create("thread_25mg",
                            thread_25mg_entry, RT_NULL,
                            THREAD_STACK_SIZE,
                            THREAD_PRIORITY, THREAD_TIMESLICE);

    /* 如果获得线程控制块，启动这个线程 */
    if (tid_6050 != RT_NULL)
        rt_thread_startup(tid_6050);
    else
        rt_kprintf("thread_6050 open fail");

    if (tid_25mg != RT_NULL)
            rt_thread_startup(tid_25mg);
    else
        rt_kprintf("thread_25mg open fail");
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(thread_sample, thread sample);
//INIT_APP_EXPORT(thread_sample);




