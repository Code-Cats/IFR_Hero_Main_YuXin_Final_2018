#include "mpu6050_process.h"

#define	SMPLRT_DIV		 0x19	//�����ǲ����� ����ֵ 0X07 125Hz
#define	CONFIG			   0x1A	//��ͨ�˲�Ƶ�� ����ֵ 0x00 
#define	GYRO_CONFIG	   0x1B	//�������Լ켰������Χ                 ����ֵ 0x18 ���Լ� 2000deg/s
#define	ACCEL_CONFIG	 0x1C	//���ٶȼ��Լ켰������Χ����ͨ�˲�Ƶ�� ����ֵ 0x01 ���Լ� 2G 5Hz

#define INT_PIN_CFG    0x37
#define INT_ENABLE     0x38
#define INT_STATUS     0x3A    //ֻ��

#define	ACCEL_XOUT_H	 0x3B
#define	ACCEL_XOUT_L	 0x3C

#define	ACCEL_YOUT_H	 0x3D
#define	ACCEL_YOUT_L	 0x3E

#define	ACCEL_ZOUT_H	 0x3F
#define	ACCEL_ZOUT_L	 0x40

#define	TEMP_OUT_H		 0x41
#define	TEMP_OUT_L		 0x42

#define	GYRO_XOUT_H		 0x43
#define	GYRO_XOUT_L		 0x44	

#define	GYRO_YOUT_H		 0x45
#define	GYRO_YOUT_L		 0x46

#define	GYRO_ZOUT_H		 0x47
#define	GYRO_ZOUT_L		 0x48

#define	PWR_MGMT_1		 0x6B	//��Դ���� ����ֵ 0x00 ��������
#define	WHO_AM_I		   0x75	//ֻ��  Ĭ�϶���Ӧ���� MPU6050_ID = 0x68


#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   //�������ݴ�ŵ�ַ�������ģ�����һ������

ACCEL_AVERAGE_DATA   Accel_Raw_Average_Data; 
GYRO_RADIAN_DATA     Gyro_Radian_Data;
MPU6050_ANGLE        MPU6050_Angle;

MPU6050_RAW_DATA    MPU6050_Raw_Data; 
MPU6050_REAL_DATA   MPU6050_Real_Data;	//������̨�����������������ο��������ã����ٶȻ�һ�£��������

int gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0;

//MPU6050 ��ʼ�����ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_Initialization(void)
{
    unsigned char temp_data = 0x00;

    IIC_GPIO_Init();  //��ʼ��IIC�ӿ�
    HEAT_Configuration();
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I ,&temp_data ,1)==0) //ȷ��IIC�����Ϲҽӵ��Ƿ���MPU6050
    {
        if(temp_data != MPU6050_ID)
        {
//            printf("error 1A\r\n");
            return 0xfc; //У��ʧ�ܣ�����0xff
        }
    }
    else
    {
//        printf("error 1B\r\n");
        return 0xcc; //��ȡʧ�� ����0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //�������״̬
    {
//        printf("error 1C\r\n");
        return 0xcf;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x07) == 0xff)//cyq��07 ����Ƶ��1khz
    {
//        printf("error 1D\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x00) == 0xff)
    {
//        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x08) == 0xff)
    {
//        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x08) == 0xff)
    {
//        printf("error 1G\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x00) == 0xff)
    {
//        printf("error 1H\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)
    {
//        printf("error 1I\r\n");
        return 0xff;
    }
    
    //MPU6050_Interrupt_Configuration(); //MPU6050�жϳ�ʼ��
    
    return 0;
}

//MPU6050  ���ݶ�ȡ���ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_ReadData(void)
{
    u8 buf[14];
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14) == 0xff)
    {
//        printf("error 1J\r\n");
        return 0xff;
    }
    else
    {
        //��ȡ�Ĵ���ԭ������
           
        MPU6050_Raw_Data.Accel_X = (buf[0]<<8 | buf[1]);
        MPU6050_Raw_Data.Accel_Y = (buf[2]<<8 | buf[3]);
        MPU6050_Raw_Data.Accel_Z = (buf[4]<<8 | buf[5]); 
        MPU6050_Raw_Data.Temp =    (buf[6]<<8 | buf[7]);  
        MPU6050_Raw_Data.Gyro_X = (buf[8]<<8 | buf[9]);
        MPU6050_Raw_Data.Gyro_Y = (buf[10]<<8 | buf[11]);
        MPU6050_Raw_Data.Gyro_Z = (buf[12]<<8 | buf[13]);

       
        //��ԭ������ת��Ϊʵ��ֵ�����㹫ʽ���Ĵ����������й�
        MPU6050_Real_Data.Accel_X = -(float)(MPU6050_Raw_Data.Accel_X)/8192.0f; //��datasheet 30 of 47
        MPU6050_Real_Data.Accel_Y = -(float)(MPU6050_Raw_Data.Accel_Y)/8192.0f; //��datasheet 30 of 47
        MPU6050_Real_Data.Accel_Z = (float)(MPU6050_Raw_Data.Accel_Z)/8192.0f; //��datasheet 30 of 47
        MPU6050_Real_Data.Temp =   (float)(MPU6050_Raw_Data.Temp)/340.0f+36.53f;//��datasheet 31 of 47
        MPU6050_Real_Data.Gyro_X = (float)(MPU6050_Raw_Data.Gyro_X - gyroADC_X_offset)/65.5f;     //��datasheet 32 of 47	//ԭ�и��ţ�������̨���������������
        MPU6050_Real_Data.Gyro_Y = -(float)(MPU6050_Raw_Data.Gyro_Y - gyroADC_Y_offset)/65.5f;     //��datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Z = -(float)(MPU6050_Raw_Data.Gyro_Z - gyroADC_Z_offset)/65.5f;     //��datasheet 32 of 47	//ԭ�޸��ţ�������̨���������������
    } 
    
    return 0;
}

void MPU6050_Gyro_calibration(void)
{
	u16 i;
	float x_temp=0,y_temp=0,z_temp=0;
	
	for(i=0; i<2000; i++)
	{
		MPU6050_ReadData();
		delay_ms(1);
		x_temp=x_temp+MPU6050_Raw_Data.Gyro_X;
		y_temp=y_temp+MPU6050_Raw_Data.Gyro_Y;
		z_temp=z_temp+MPU6050_Raw_Data.Gyro_Z;
	}			
	
	x_temp=x_temp/2000.00f;
	y_temp=y_temp/2000.00f;	
	z_temp=z_temp/2000.00f;

	gyroADC_X_offset=(int)x_temp;
	gyroADC_Y_offset=(int)y_temp;
	gyroADC_Z_offset=(int)z_temp;
	
}

void MPU6050_Data_Filter(void)  // this*0.01 + (��һ���õ�)last *0.99 Ҫ��
{
    unsigned int i=0;
    static unsigned int first_flag = 0;
    static unsigned int filter_cnt = 0;    //������ٶȼ��˲��Ĵ���
    
    long temp_accel_x = 0; //������ż��ٶȼ�X��ԭ�����ݵ��ۼӺ�
    long temp_accel_y = 0; //������ż��ٶȼ�Y��ԭ�����ݵ��ۼӺ�
    long temp_accel_z = 0; //������ż��ٶȼ�Z��ԭ�����ݵ��ۼӺ�
    
    static short accel_x_buffer[10] = {0}; //������ż��ٶȼ�X�����10�����ݵ�����
    static short accel_y_buffer[10] = {0}; //������ż��ٶȼ�Y�����10�����ݵ�����
    static short accel_z_buffer[10] = {0}; //������ż��ٶȼ�Z�����10�����ݵ�����
    
    if(first_flag == 0) //�����һ�ν����ú��������������ƽ����������г�ʼ��
    {
        first_flag = 1; //�Ժ��ٽ���
        for(i=0;i<10;i++)
        {
            accel_x_buffer[i] = MPU6050_Raw_Data.Accel_X;
            accel_y_buffer[i] = MPU6050_Raw_Data.Accel_Y;
            accel_z_buffer[i] = MPU6050_Raw_Data.Accel_Z;
        }
    }
    else  //������ǵ�һ����	//�ƶ�ƽ���˲�
    {
        accel_x_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_X;
        accel_y_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Y;
        accel_z_buffer[filter_cnt] = MPU6050_Raw_Data.Accel_Z;   
        
        filter_cnt ++;
        if(filter_cnt == 10)
        {
            filter_cnt = 0;
        }        
    }
    
    for(i=0;i<10;i++)
    {
        temp_accel_x += accel_x_buffer[i];
        temp_accel_y += accel_y_buffer[i];
        temp_accel_z += accel_z_buffer[i];
    }
    
    Accel_Raw_Average_Data.X = (float)temp_accel_x / 10.0f;
    Accel_Raw_Average_Data.Y = (float)temp_accel_y / 10.0f;
    Accel_Raw_Average_Data.Z = (float)temp_accel_z / 10.0f;
    
    Gyro_Radian_Data.X = (float)(MPU6050_Real_Data.Gyro_X  * (3.14159265/180.0));
    Gyro_Radian_Data.Y = (float)(MPU6050_Real_Data.Gyro_Y  * (3.14159265/180.0));
    Gyro_Radian_Data.Z = (float)(MPU6050_Real_Data.Gyro_Z  * (3.14159265/180.0));
}


void MPU6050_Angle_Calculate( float gyro_x,float gyro_y,float gyro_z,float accel_x,float accel_y,float accel_z)
{
    static float q0 = 1;
    static float q1 = 0;
    static float q2 = 0;
    static float q3 = 0;
    
    static float exInt = 0;
    static float eyInt = 0;
    static float ezInt = 0;
    
    const float kp = 0.3; //
    const float ki = 0.00; //0.0;
    const float halfT = 0.001; //�������ڵ�һ��ֵ
    
    float norm; //������ģ
    float vx,vy,vz;
    float ex,ey,ez;

    float ax,ay,az; //���ٶ�������ģ�ı�ֵ 
    float gx,gy,gz; //������

    static float pre_ax = 0;
    static float pre_ay = 0;
    static float pre_az = 0;
    //���ٶ��˲�
    accel_x = accel_x *0.02f + pre_ax * 0.98f;//cyq:0.02
    pre_ax = accel_x;
    
    accel_y = accel_y *0.02f + pre_ay * 0.98f;
    pre_ay = accel_y;

    accel_z = accel_z *0.02f + pre_az * 0.98f;
    pre_az = accel_z;    
    
    norm = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    ax = accel_x / norm;
    ay = accel_y / norm;
    az = accel_z / norm;
    
    vx = 2 * (q1*q3 - q0*q2);
    vy = 2 * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    exInt += ki*ex;
    eyInt += ki*ey;
    ezInt += ki*ez;
    
    gx = gyro_x + kp*ex + exInt;
    gy = gyro_y + kp*ey + eyInt;
    gz = gyro_z + kp*ez + ezInt;
    
    q0 += (      - q1*gx - q2*gy - q3*gz)*halfT;
    q1 += (q0*gx +         q2*gz - q3*gy)*halfT;
    q2 += (q0*gy - q1*gz +         q3*gx)*halfT;
    q3 += (q0*gz + q1*gy - q2*gx        )*halfT;

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    
    MPU6050_Angle.Rool = asin(-2 * q1 * q3 + 2 * q0* q2) * (180.0f/3.14159265f); 
    MPU6050_Angle.Pitch  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * (180.0f/3.14159265f); 
    MPU6050_Angle.Yaw = atan2( 2 * q1 * q2 + 2 * q0 * q3,1.0f - 2.0f * ( q2 * q2 + q3 * q2 ) ) * (180.0f/3.14159265f);//��׼

}

