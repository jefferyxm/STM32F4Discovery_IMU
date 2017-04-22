#include "main.h"

#define DISCOVERY 0
void bsp_led_init()
{
  /*    FOR discovery
        led3 ----pd13
  	led4 ----pd12
  	led5 ----pd14
  	led6 ----pd15

        For ZG
        PF9  led2;
        PF10 led3;
  */  
#if DISCOVERY
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  GPIO_InitTypeDef GPIO_Init_Struct;
  GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init_Struct.GPIO_OType= GPIO_OType_PP;
  GPIO_Init_Struct.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_Init_Struct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init_Struct.GPIO_Speed = GPIO_High_Speed;

  GPIO_Init(GPIOD,&GPIO_Init_Struct);
#else
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
  GPIO_InitTypeDef GPIO_Init_Struct;
  GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init_Struct.GPIO_OType= GPIO_OType_PP;
  GPIO_Init_Struct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
  GPIO_Init_Struct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init_Struct.GPIO_Speed = GPIO_Fast_Speed;

  GPIO_Init(GPIOF,&GPIO_Init_Struct);
  led3off();
  led2off();
#endif  
}

void perip_uart_init()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  
  //A2--uart2_TX .  A3  uart2_Rx
  GPIO_InitTypeDef GPIO_Init_Struct;
  GPIO_Init_Struct.GPIO_OType =GPIO_OType_PP;
  GPIO_Init_Struct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init_Struct.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init_Struct.GPIO_Pin =GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_Init_Struct);
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);


  // uart configuration
  USART_InitTypeDef UsartInit_Struct;
  UsartInit_Struct.USART_BaudRate = 460800;
  UsartInit_Struct.USART_HardwareFlowControl =  USART_HardwareFlowControl_None;
  UsartInit_Struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  UsartInit_Struct.USART_Parity = USART_Parity_No;
  UsartInit_Struct.USART_StopBits = USART_StopBits_1;
  UsartInit_Struct.USART_WordLength = USART_WordLength_8b;
  USART_Init(USART2,&UsartInit_Struct);

  NVIC_InitTypeDef NVICInit_Struct;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVICInit_Struct.NVIC_IRQChannel = USART2_IRQn;
  NVICInit_Struct.NVIC_IRQChannelPreemptionPriority = 1;
  NVICInit_Struct.NVIC_IRQChannelSubPriority = 1;
  NVICInit_Struct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVICInit_Struct);
  

  USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

  USART_Cmd(USART2,ENABLE);
}

void perip_I2C_init()
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

//  b10-- i2c_2_sck .   b11--i2c_2 sda
   GPIO_InitTypeDef GPIOB_Init_Struct;
   GPIOB_Init_Struct.GPIO_Mode = GPIO_Mode_AF;
   GPIOB_Init_Struct.GPIO_OType = GPIO_OType_OD;
   GPIOB_Init_Struct.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
   GPIOB_Init_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIOB_Init_Struct.GPIO_Speed = GPIO_Fast_Speed;
   
   GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_I2C2);
   GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_I2C2);
   GPIO_Init(GPIOB,&GPIOB_Init_Struct);

   I2C_InitTypeDef i2c2_InitStruct;
   i2c2_InitStruct.I2C_Mode = I2C_Mode_I2C;
   i2c2_InitStruct.I2C_ClockSpeed = 300000;
   i2c2_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
   i2c2_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
   i2c2_InitStruct.I2C_Ack = I2C_Ack_Enable;
   i2c2_InitStruct.I2C_OwnAddress1 = 0;
   I2C_Init(I2C2,&i2c2_InitStruct);
   I2C_Cmd(I2C2, ENABLE);
}

/***********************   timer   ******************************/
void TIM_init()
{
      RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM6EN,ENABLE);
      
      TIM_TimeBaseInitTypeDef TimInitStuct;   
      TimInitStuct.TIM_Period = 4999;     //3ms
      TimInitStuct.TIM_Prescaler = 167;

      TIM_TimeBaseInit(TIM6,&TimInitStuct);
      TIM_ClearFlag(TIM6, TIM_FLAG_Update);
      TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
      

      NVIC_InitTypeDef NVIC_InitStruct;
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
      NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
      NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
      NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStruct);
}

/***********************   EXIT   ******************************/
void EXTI_config()
{
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
      GPIO_InitTypeDef GPIO_InitStruct;
      GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
      GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
      GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
      GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
      GPIO_Init(GPIOA,&GPIO_InitStruct);

      RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
      SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource0);

      NVIC_InitTypeDef NVIC_InitStruct;
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
      NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
      NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
      NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStruct);

      EXTI_InitTypeDef EXTI_InitStruct;
      EXTI_InitStruct.EXTI_Line = EXTI_Line0;
      EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
      EXTI_InitStruct.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStruct);	
}


/***********************   MCO   ******************************/
void MCO_init()
{
      GPIO_InitTypeDef GPIO_InitStructure;

      /* Output HSE clock on MCO1 pin(PA8) ****************************************/ 
      /* Enable the GPIOA peripheral */ 
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

      /* Configure MCO1 pin(PA8) in alternate function */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* HSE clock selected to output on MCO1 pin(PA8)*/
      RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);

      /* Output SYSCLK/4 clock on MCO2 pin(PC9) ***********************************/ 
      /* Enable the GPIOACperipheral */ 
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

      /* Configure MCO2 pin(PC9) in alternate function */
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
      GPIO_Init(GPIOC, &GPIO_InitStructure);

      /* SYSCLK/4 clock selected to output on MCO2 pin(PC9)*/
      RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_4);
}
    