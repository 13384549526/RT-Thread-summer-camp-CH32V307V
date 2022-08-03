/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-01     Misth       the first version
 */
#include "kalman.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
#define MPU6XXX_DEVICE_NAME  "i2c1"

//#define THREAD_PRIORITY         25
//#define THREAD_STACK_SIZE       512
//#define THREAD_TIMESLICE        10

#define THREAD_PRIORITY_6050            25
#define THREAD_STACK_SIZE_6050          512
#define THREAD_TIMESLICE_6050           10

static rt_thread_t tid_6050 = RT_NULL;
//static rt_mailbox_t mailbox_X = RT_NULL;
//static rt_mailbox_t mailbox_Y = RT_NULL;

//IMU_Parameter IMU_Data;



uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

void MPU6050_Read_All(IMU_Parameter *IMU_Data)
{
    double dt = (double)(rt_tick_get() - timer) / 1000;
    timer = rt_tick_get();
    double roll;
    double roll_sqrt = sqrt(IMU_Data->Accel_X * IMU_Data->Accel_X + IMU_Data->Accel_Z * IMU_Data->Accel_Z);
    if (roll_sqrt != 0.0)
    {
        roll = atan(IMU_Data->Accel_Y / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-IMU_Data->Accel_X, IMU_Data->Accel_Z) * RAD_TO_DEG;
    if ((pitch < -90 && IMU_Data->KalmanAngleY > 90) || (pitch > 90 && IMU_Data->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        IMU_Data->KalmanAngleY = pitch;
    }
    else
    {
        IMU_Data->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, IMU_Data->Gyro_Y, dt);
    }
    if (fabs(IMU_Data->KalmanAngleY) > 90)
        IMU_Data->Gyro_X = -IMU_Data->Gyro_X;
    IMU_Data->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, IMU_Data->Gyro_Y, dt);
}
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};


///* Test function */
//static int mpu6xxx_test()
//{
//    struct mpu6xxx_device *dev;
//    struct mpu6xxx_3axes accel, gyro;
//
//    /* Initialize mpu6xxx, The parameter is RT_NULL, means auto probing for i2c*/
//    dev = mpu6xxx_init(MPU6XXX_DEVICE_NAME, RT_NULL);
//
//    if (dev == RT_NULL)
//    {
//        rt_kprintf("mpu6xxx init failed\n");
//        return -1;
//    }
//    rt_kprintf("mpu6xxx init succeed\n");
//
//    while(1)
//    {
//        mpu6xxx_get_accel(dev, &accel);
//        mpu6xxx_get_gyro(dev, &gyro);
//
//        IMU_Data.Accel_X = accel.x;
//        IMU_Data.Accel_Y = accel.y;
//        IMU_Data.Accel_Z = accel.z;
//        IMU_Data.Gyro_X = gyro.x;
//        IMU_Data.Gyro_Y = gyro.y;
//        IMU_Data.Gyro_Z = gyro.z;
//
//        rt_kprintf("accel.x = %3d, accel.y = %3d, accel.z = %3d ", accel.x, accel.y, accel.z);
//        rt_kprintf("gyro.x = %3d gyro.y = %3d, gyro.z = %3d\n", gyro.x, gyro.y, gyro.z);
//
//        MPU6050_Read_All(&IMU_Data);
//
//        rt_kprintf("KalmanAngleX = %3d, KalmanAngleY = %3d, KalmanAngleZ = %3d ", IMU_Data.KalmanAngleX, IMU_Data.KalmanAngleY, IMU_Data.KalmanAngleZ);
//
//        rt_thread_mdelay(100);
//    }
//
//    mpu6xxx_deinit(dev);
//
//    return 0;
//}
//MSH_CMD_EXPORT(mpu6xxx_test, mpu6xxx sensor test function);

///* 线程 6050 的入口函数 */
//static void thread_6050_entry(void *parameter)
//{
//    struct mpu6xxx_device *dev;
//    struct mpu6xxx_3axes accel, gyro;
//
//    /* Initialize mpu6xxx, The parameter is RT_NULL, means auto probing for i2c*/
//    dev = mpu6xxx_init(MPU6XXX_DEVICE_NAME, RT_NULL);
//
//    if (dev == RT_NULL)
//    {
//        rt_kprintf("mpu6xxx init failed\n");
//        return -1;
//    }
//    rt_kprintf("mpu6xxx init succeed\n");
//    while (1)
//    {
//        mpu6xxx_get_accel(dev, &accel);
//        mpu6xxx_get_gyro(dev, &gyro);
//
//        IMU_Data.Accel_X = accel.x;
//        IMU_Data.Accel_Y = accel.y;
//        IMU_Data.Accel_Z = accel.z;
//        IMU_Data.Gyro_X = gyro.x;
//        IMU_Data.Gyro_Y = gyro.y;
//        IMU_Data.Gyro_Z = gyro.z;
//
//        MPU6050_Read_All(&IMU_Data);
//
//        rt_kprintf("KalmanAngleX = %3d, KalmanAngleY = %3d\n", IMU_Data.KalmanAngleX, IMU_Data.KalmanAngleY);
//        rt_mb_send (mailbox_X, (rt_uint32_t)&IMU_Data.KalmanAngleX);
//        rt_mb_send (mailbox_Y, (rt_uint32_t)&IMU_Data.KalmanAngleY);
//
//
//        rt_thread_mdelay(100);
//    }
//    mpu6xxx_deinit(dev);
//}
//int thread_sam6050(void)
//{
//    mailbox_X = rt_mb_create ("mailbox_X", 5, RT_IPC_FLAG_FIFO);
//    mailbox_Y = rt_mb_create ("mailbox_Y", 5, RT_IPC_FLAG_FIFO);
//
//    /* 创建线程 1，名称是 thread_6050，入口是 thread_6050_entry*/
//    tid_6050 = rt_thread_create("thread_6050",
//                            thread_6050_entry, RT_NULL,
//                            THREAD_STACK_SIZE_6050,
//                            THREAD_PRIORITY_6050, THREAD_TIMESLICE_6050);
//
//    /* 如果获得线程控制块，启动这个线程 */
//    if (tid_6050 != RT_NULL)
//        rt_thread_startup(tid_6050);
//    else
//        rt_kprintf("thread_6050 open fail");
//
//    return 0;
//}
//
///* 导出到 msh 命令列表中 */
//MSH_CMD_EXPORT(thread_sam6050, thread_sample_6050);
