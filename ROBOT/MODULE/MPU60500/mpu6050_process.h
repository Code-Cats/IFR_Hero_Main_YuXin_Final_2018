#ifndef __MPU6050_PROCESS_H__
#define __MPU6050_PROCESS_H__

#include  "main.h"


typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
    short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
    short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    float Gyro_X;   //ת����ʵ�ʵ�X����ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y����ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z����ٶ�
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

