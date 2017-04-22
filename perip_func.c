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

#include"stm32f4xx.h"
static uint32_t Exception_callback(char *str);

/***********************   usart   ******************************/

void usart_sendByte(USART_TypeDef *USARTx, uint8_t data)
{
    	USART_SendData(USARTx, data);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
}

void usart_sendString(USART_TypeDef *USARTx,char*str)
{
	uint8_t k = 0;
	do
	{
		usart_sendByte(USARTx,*(str+k));
		k++;
	}while(*(str+k)!='\0');
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TC));	
}

void usart_sendMessage(USART_TypeDef *USARTx, uint8_t *p_data, uint8_t by_num)
{
   for(uint8_t i = 0; i<by_num; i++)
   {
      usart_sendByte(USARTx,*(p_data+i));
   }
   while(USART_GetFlagStatus(USARTx,USART_FLAG_TC));
}




/***********************   I2C   ******************************/
#define I2C_COM_TIMEOUT ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT ((uint32_t)(I2C_FLAG_TIMEOUT*10))

void i2c_writeByte(I2C_TypeDef * I2Cx, uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    //i2c 起始信号
    I2C_GenerateSTART(I2Cx,ENABLE);
    uint32_t i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((i2c_timeout--)==0) Exception_callback("i2c start");
    }
    
    //设置地址
    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if((i2c_timeout--)==0) Exception_callback("i2c selectSlaveDevice");
    }

    //设置寄存器
    I2C_SendData(I2Cx,regAddr);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((i2c_timeout--)==0) Exception_callback("i2c configRegister");
    }
    
    //发送数据
    I2C_SendData(I2Cx,data);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((i2c_timeout--)==0) Exception_callback("i2c sendingData");
    }
    
    //i2c 停止信号
    I2C_GenerateSTOP(I2Cx,ENABLE);
}

uint8_t i2c_readByte(I2C_TypeDef * I2Cx, uint8_t slaveAddr, uint8_t regAddr)
{
    //i2c 起始信号
    I2C_GenerateSTART(I2Cx,ENABLE);
    uint32_t i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((i2c_timeout--)==0) Exception_callback("4");
    }

    //设置地址
    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if((i2c_timeout--)==0) Exception_callback("5");
    }
    I2C_Cmd(I2Cx, ENABLE);
	
    //设置寄存器
    I2C_SendData(I2Cx,regAddr);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((i2c_timeout--)==0) Exception_callback("6");
    }
    
//换方向，需要重新发送起始信号
    I2C_GenerateSTART(I2Cx,ENABLE);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((i2c_timeout--)==0) Exception_callback("7");
    }

    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);
    i2c_timeout = I2C_COM_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
    {
        if((i2c_timeout--)==0) Exception_callback("8");
    }

    i2c_timeout = I2C_LONG_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
	if((i2c_timeout--)==0)Exception_callback("9");
    }
    uint8_t u8_result = I2C_ReceiveData(I2Cx);
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx,ENABLE);
    return u8_result;
}


/***********************   common   ******************************/
static uint32_t Exception_callback(char *str)
{
  usart_sendString(USART2, "Exception generate at");
  usart_sendString(USART2, str);
  return 0;
}
