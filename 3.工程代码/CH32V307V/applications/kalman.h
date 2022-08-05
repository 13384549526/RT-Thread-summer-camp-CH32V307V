/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-01     Misth       the first version
 */
#ifndef APPLICATIONS_KALMAN_H_
#define APPLICATIONS_KALMAN_H_

#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include "sensor_inven_mpu6xxx.h"
#include "mpu6xxx.h"

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

struct IMU_Parameter
{
    rt_int16_t Accel_X;
    rt_int16_t Accel_Z;
    rt_int16_t Accel_Y;
    rt_int16_t Gyro_X;
    rt_int16_t Gyro_Y;
    rt_int16_t Gyro_Z;
    rt_int16_t KalmanAngleX;
    rt_int16_t KalmanAngleY;
    rt_int16_t KalmanAngleZ;
};

void Kalman_get_value(struct mpu6xxx_device *dev, struct IMU_Parameter *IMU_Data);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
void Kalman_get_value(struct mpu6xxx_device *dev, struct IMU_Parameter *IMU_Data);


#endif /* APPLICATIONS_KALMAN_H_ */
