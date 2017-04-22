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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "perip_func.h"

static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);
int16_t a1 = 0;

int main(void)
{
    SystemInit();
    bsp_led_init();
    perip_uart_init();
    START_SYSTICK();
    perip_I2C_init();
    TIM_init();
    //GPIO_SetBits(GPIOD,GPIO_Pin_12);
    
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency/100);

    //GPIO_Init(GPIOB,&GPIO_Init_Struct);
    
    //ano_sendDebug(0,6);
    //usart_sendString(USART2,"helloworld");
    //GPIO_SetBits(GPIOB,GPIO_Pin_4);

    MARK_SYSTICK();WAIT_SYSTICK(84000);
    mpu6050_init();
    MARK_SYSTICK();WAIT_SYSTICK(84000);
    get_mpuinfo();

    //EXTI_config();
    TIM_Cmd(TIM6,ENABLE);
    while(1)
    {        
        GPIO_SetBits(GPIOB,GPIO_Pin_4);
        //led3on();
	MARK_SYSTICK();
	WAIT_SYSTICK(8400);
        //led3off();
        GPIO_ResetBits(GPIOB,GPIO_Pin_4);
        MARK_SYSTICK();
	WAIT_SYSTICK(8400);
    }
}




void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

void testSystick()
{
  START_SYSTICK();
  SystemInit();
  bsp_led_init();
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency/100);
  //tim = RCC_Clocks.HCLK_Frequency;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
