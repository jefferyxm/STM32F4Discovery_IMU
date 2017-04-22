/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

void USART1_IRQHandler(void)
{
#ifndef STM32F40_41xxx
  if(USART_GetFlagStatus(USART1,USART_SR_RXNE)!= RESET)
  {
     if(USART_ReceiveData(USART1)==4)
  	{
  	  GPIO_SetBits(GPIOD,GPIO_Pin_15);
  	}
  }
 #endif
}

int32_t start = 0;
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
            start = SysTick->VAL;
            read_6050();
            IMUupdate(g_f32_gyroX, g_f32_gyroY, g_f32_gyroZ, g_f32_accX, g_f32_accY, g_f32_accZ);
            k_Filter();
            //ano_sendIMU();
            /*
            f32_cache[0] = g_f32_gyroX;
            f32_cache[1] = g_f32_gyroY;
            f32_cache[2] = g_f32_gyroZ;
            f32_cache[3] = g_f32_accX;
            f32_cache[4] = g_f32_accY;
            f32_cache[5] = g_f32_accZ;
            f32_cache[6] = Roll;
            f32_cache[7] = Pit;
            f32_cache[8] = Yaw;*/
            f32_cache[0] = Roll;
            f32_cache[1] = Pit;
            f32_cache[2] = K_Roll;
            f32_cache[3] = K_Pit;
            
            SendTO_ANO(4,0,0);
            start = start - SysTick->VAL;
            TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
            
            led2toggle();
	}
}



void EXTI0_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line0)!= RESET)
	{
		led2toggle();
                //START_SYSTICK();
                start = SysTick->VAL;
                read_6050();      //4.76ms 100khz iic frequence
                start = start - SysTick->VAL;
                i16_cache[0] = g_i16_accX;
                i16_cache[1] = g_i16_accY;
                i16_cache[2] = g_i16_accZ;
                i16_cache[3] = g_i16_gyroX;
                i16_cache[4] = g_i16_gyroY;
                i16_cache[5] = g_i16_gyroZ;
                SendTO_ANO(0,0,6);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
