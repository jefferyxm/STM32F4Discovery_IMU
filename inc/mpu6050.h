

#include"stm32f4xx.h"
#ifndef __MPU6050_H__
#define __MPU6050_H__


#define     MPU6050_DEVICE          I2C2        //����MPU6050 ���õĽӿ� Ϊ I2C0

/* mpu6050 common Register Address ------------------------------------------------------------*/
#define	SMPLRT_DIV		(0x19)	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			(0x1A)	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		(0x1B)	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	(0x1C)	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define	ACCEL_XOUT_H    (0x3B)  //X����ٶ�  ���ٶ�
#define	ACCEL_XOUT_L	(0x3C)	
#define	ACCEL_YOUT_H	(0x3D)  //Y����ٶ�
#define	ACCEL_YOUT_L	(0x3E)
#define	ACCEL_ZOUT_H	(0x3F)  //Z����ٶ�
#define	ACCEL_ZOUT_L	(0x40)

#define	TEMP_OUT_H		(0x41)
#define	TEMP_OUT_L		(0x42)

#define	GYRO_XOUT_H		(0x43)  //X����ٶ�  ������
#define	GYRO_XOUT_L		(0x44)	
#define	GYRO_YOUT_H		(0x45)  //Y����ٶ�
#define	GYRO_YOUT_L		(0x46)
#define	GYRO_ZOUT_H		(0x47)  //Z����ٶ�
#define	GYRO_ZOUT_L		(0x48)

#define	PWR_MGMT_1		(0x6B)	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		(0x75)	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SlaveAddress	(0xd0)	//Ӳ��I2C��ַ0x68 ���I2C��ַ0xD0




/*  interrupt register */
#define INT_PIN_CFG     (0x37)   //  �ⲿ�ж���������
#define INT_ENABLE      (0x38)   //�ⲿ�ж��ź�ʹ��
#define INT_STATUS      (0x39)   //�ж�״̬�Ĵ���

/*FIFO*/

#define FIFO_EN         (0x23)
#define FIFO_COUNT_H    (0x72)   // FIFO����
#define FIFO_COUNT_L    (0x73) 
#define FIFO_R_W        (0X74)
#define USER_CTRL       (0x6A)



//function declartion
void  mpu6050_init(void);                        //��ʼ��MPU6050
void mpu6050_writeByte(uint8_t regAddr,uint8_t value);
uint8_t mpu6050_readByte(uint8_t regAddr);
void read_6050();
void get_mpuinfo();
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void k_Filter();

// global var
extern int16_t g_i16_accX,g_i16_accY,g_i16_accZ;
extern int16_t g_i16_gyroX,g_i16_gyroY,g_i16_gyroZ;

extern float g_f32_accX,g_f32_accY,g_f32_accZ;
extern float g_f32_gyroX,g_f32_gyroY,g_f32_gyroZ;

extern float Roll,Pit,Yaw;
extern float K_Roll,K_Pit;
extern int16_t g_i16_buffer[12];  

extern uint8_t g_u8_info;


#endif  //__FIRE_MPU6050_H__