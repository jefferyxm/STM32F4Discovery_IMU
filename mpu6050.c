/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "mpu6050.h"
#include "perip_func.h"
#include "math.h"

//�궨����õײ��I2C�ӿ�
#define MPU6050_WR(reg,value)   i2c_writeByte(I2C2,SlaveAddress,reg,value)  //MPU6050 д�Ĵ���
#define MPU6050_RD(reg)         i2c_readByte(I2C2,SlaveAddress,reg)             //MPU6050 ���Ĵ���

int16_t g_i16_accX,g_i16_accY,g_i16_accZ;
int16_t g_i16_gyroX,g_i16_gyroY,g_i16_gyroZ;

float g_f32_accX,g_f32_accY,g_f32_accZ;
float g_f32_gyroX,g_f32_gyroY,g_f32_gyroZ;

int16_t g_i16_buffer[12];   //int16_data of  acc/gyro/mag/angle

float Roll,Pit,Yaw;

uint8_t g_u8_info;


void mpu6050_init(void)
{
    MPU6050_WR(PWR_MGMT_1, 0x00);	//�������״̬
    MPU6050_WR(SMPLRT_DIV, 0x07);	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
    MPU6050_WR(CONFIG, 0x06);		//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
    MPU6050_WR(GYRO_CONFIG, 0x18);	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
    MPU6050_WR(ACCEL_CONFIG, 0x01);	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
    //MPU6050_WR(INT_PIN_CFG,0x00);
    //MPU6050_WR(INT_ENABLE,0x01);
}

int32_t start2 = 0;
void read_6050()
{
    //start2 = SysTick->VAL;
    g_i16_accX = MPU6050_RD(ACCEL_XOUT_H);			//���ټƶ�����
    g_i16_accX = (g_i16_accX << 8) | MPU6050_RD(ACCEL_XOUT_L);
    //start2 = start2 - SysTick->VAL;
    
    g_i16_accY = MPU6050_RD(ACCEL_YOUT_H);
    g_i16_accY = (g_i16_accY << 8) | MPU6050_RD(ACCEL_YOUT_L);
    g_i16_accZ = MPU6050_RD(ACCEL_ZOUT_H);
    g_i16_accZ = (g_i16_accZ << 8) | MPU6050_RD(ACCEL_ZOUT_L);

    g_i16_gyroX = MPU6050_RD(GYRO_XOUT_H);				//����������
    g_i16_gyroX = (g_i16_gyroX << 8) | MPU6050_RD(GYRO_XOUT_L);
    g_i16_gyroY = MPU6050_RD(GYRO_YOUT_H);
    g_i16_gyroY = (g_i16_gyroY << 8) | MPU6050_RD(GYRO_YOUT_L);
    g_i16_gyroZ = MPU6050_RD(GYRO_ZOUT_H);
    g_i16_gyroZ = (g_i16_gyroZ << 8) | MPU6050_RD(GYRO_ZOUT_L);
    
    //���ٶȼ�У׼
    g_i16_buffer[0] = g_i16_accX -550;
    g_i16_buffer[1] = g_i16_accY +500;
    g_i16_buffer[2] = g_i16_accZ +3500;
    
    g_f32_accX = g_i16_buffer[0]/1.0;
    g_f32_accY = g_i16_buffer[1]/1.0;
    g_f32_accZ = g_i16_buffer[2]/1.02;
    
    g_i16_buffer[3] = g_i16_gyroX + 21;     //������������Ư    
    g_i16_buffer[4] = g_i16_gyroY - 18;
    g_i16_buffer[5] = g_i16_gyroZ + 11;

    g_f32_gyroX = (float)(g_i16_buffer[3]/16.4);
    g_f32_gyroY = (float)(g_i16_buffer[4]/16.4);
    g_f32_gyroZ = (float)(g_i16_buffer[5]/16.4);

}

/**��̬����**/
#define Kp 30.0f			//���Ƽ��ٶȼ����������ǻ�����̬���ٶ�
#define Ki 0.01f
#define halfT 0.0015f		// ��̬����ʱ���һ�롣
#define Gyro_G 		0.017453f //	0.0010642f

//��ʼ����Ԫ��
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

//������̬�������Ļ���
float exInt = 0, eyInt = 0, ezInt = 0;

//����Ϊ��̬���㺯����
//����gx��gy��gz�ֱ��Ӧ������Ľ��ٶȣ���λ�ǻ��� / ��;
//����ax��ay��az�ֱ��Ӧ������ļ��ٶ�ԭʼ����
//���ڼ��ٶȵ������ϴ󣬴˴�Ӧ�����˲��������

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	//�����ٶȵ�ԭʼ���ݣ���һ�����õ���λ���ٶ�
	norm = sqrt(ax*ax + ay*ay + az*az);
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	//����Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء��������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء����������vx��vy��vz����ʵ���ǵ�ǰ�Ļ����������ϵ�ϣ����������������λ������(�ñ�ʾ������̬����Ԫ�����л���)
	vx = 2 * (q1*q3 - q0*q2);
	vy = 2 * (q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	/*����˵��һ�㣬���ٶȼ����������Ƚϴ󣬶����ڷ��й����У��ܻ�����Ӱ������������ԣ���ʱ���ڵĿɿ��Բ��ߡ�����������С���������ڻ�������ɢ�ģ���ʱ��Ļ��ֻ����Ư�Ƶ�����������Ҫ���ü��ٶȼ���õ���̬�����������ǻ�����̬��Ư�ơ�
	�ڻ����������ϵ�ϣ����ٶȼƲ����������������ax��ay��az; ���ݻ��ֺ����̬�������������������vx��vy��vz; ����֮�������������������ݻ��ֺ����̬�ͼ��ٶȼƲ��������̬֮�����
	���������������������(Ҳ����������)����ʾ��ex��ey��ez�����������������Ĳ���������������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
	�������ѧ�������ٶȰٿ�������ϸ���͡�*/
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	//����������л���
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//�ò���������PI����������ƫ��ͨ������Kp��Ki�������������Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶ�
	gx = gx* Gyro_G;
	gy = gy* Gyro_G;
	gz = gz* Gyro_G;

	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	/*��Ԫ��΢�ַ��̣����ת����ŷ���Ǽ�����.*/
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//��Ԫ����λ��
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	//ת��Ϊŷ����
	//Roll = atan(2.0*(q2*q3 + q0*q1) / (q0*q0 - q1*q1 - q2*q2 + q3*q3)) * 57.3;
	//Pit = asin(-2.0*q1*q3 + 2 * q0*q2) * 57.3;
	//Yaw = atan(2.0*(2*q0*q1+2*q2*q3))*57.3;
	
	//ת��Ϊŷ����
	Roll = 57.3*atan2((2 * q0*q1 + 2 * q2*q3), (1 - 2 * q1*q1 - 2 * q2*q2));
	Pit = 57.3*asin(2 * q0*q2 - 2 * q1*q3);
	Yaw = 57.3*atan2((2 * q0*q3 + 2 * q1*q2), (1 - 2 * q2*q2 - 2 * q3*q3));
        
        g_i16_buffer[9] = (int16_t)(Roll *100);
        g_i16_buffer[10] = (int16_t)(Pit *100);
        g_i16_buffer[11] = 0;
	//Yaw += gz*0.1f;
}


/******kalmsn�˲�********/

float angle_dotx, angle_doty, angle_dotz;
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.03, dt = 0.01;
//0.0001         //0.00015        //1.2
//ע��:dt��ȡֵkalman�˲�������ʱ��;         //0.8
float P[2][2] = {
	{ 1, 0 },
	{ 0, 1 }
};

float Pdot[4] = { 0, 0, 0, 0 };

const char C_0 = 1;

float  PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

float q_biasx, q_biasy, q_biasz;
float angle_errx, angle_erry, angle_errz;

float angle_x, angle_y, angle_z;
float gyro_x, gyro_y, gyro_z;

#define GYRO_G_X	0.01745  //20.95	
#define GYRO_G_Y	0.01745  //10.75	
#define GYRO_G_Z	0.01745  //0.0010642f	

float K_Roll, K_Pit;

void k_Filter()
{

	gyro_x = g_f32_gyroX * GYRO_G_X;
	gyro_y = g_f32_gyroY * GYRO_G_Y;
	//gyro_z = g3 * GYRO_G_Z;

	angle_x += (gyro_y - q_biasx) * dt;
	angle_y += (gyro_x - q_biasy) * dt;
	//angle_z += (gyro_z - q_biasz) * dt;
        
        angle_errx = Roll*GYRO_G_X - angle_x;
	angle_erry = Pit*GYRO_G_Y - angle_y;

	Pdot[0] = Q_angle - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;

	angle_x += K_0 * angle_errx;
	angle_y += K_0 * angle_erry;
	//angle_z += K_0 * angle_errz;

	q_biasx += K_1 * angle_errx;
	q_biasy += K_1 * angle_erry;
	//q_biasz += K_1 * angle_errz;

	angle_dotx = gyro_x - q_biasx;
	angle_doty = gyro_y - q_biasy;
	//angle_dotz = gyro_z - q_biasz;

	K_Roll = angle_x*57.3;
	K_Pit = angle_y*57.3;

	//g_f32_gyroX = angle_dotx;
	//g_f32_gyroY = angle_doty;
	//g3 = angle_dotz;
}


void get_mpuinfo()
{
    g_u8_info = MPU6050_RD(WHO_AM_I);
}

void mpu6050_writeByte(uint8_t regAddr,uint8_t value)
{
    MPU6050_WR(regAddr,value);
}

uint8_t mpu6050_readByte(uint8_t regAddr)
{
    return MPU6050_RD(regAddr);
}


