#ifndef __MPU6050_PROCESS_H__
#define __MPU6050_PROCESS_H__

#include  "main.h"


typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角速度，
    float Gyro_Y;   //转换成实际的Y轴角速度，
    float Gyro_Z;   //转换成实际的Z轴角速度
}MPU6050_REAL_DATA;

typedef struct __ACCEL_AVERAGE_DATA__
{
    float X;
    float Y;
    float Z;
}ACCEL_AVERAGE_DATA;

typedef struct __GYRO_RADIAN_DATA__
{
    float X;
    float Y;
    float Z;
}GYRO_RADIAN_DATA;

typedef struct __MPU6050_ANGLE__
{
    float Pitch;
    float Rool;
    float Yaw;
}MPU6050_ANGLE;

int MPU6050_Initialization(void);
int MPU6050_ReadData(void);
void MPU6050_Gyro_calibration(void);
void MPU6050_Data_Filter(void);
void MPU6050_Angle_Calculate( float gyro_x,float gyro_y,float gyro_z,float accel_x,float accel_y,float accel_z);
#endif

