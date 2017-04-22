#ifndef __BOARD_INIT_H
#define __BOARD_INIT_H

void bsp_led_init();
void perip_uart_init();
void perip_I2C_init();
void TIM_init();
void EXTI_config();
void MCO_init();
#define DISCOVERY 0

#if DISCOVERY
#define led3on() GPIOD->BSRRL = GPIO_Pin_13
#define led4on() GPIOD->BSRRL = GPIO_Pin_12
#define led5on() GPIOD->BSRRL = GPIO_Pin_14
#define led6on() GPIOD->BSRRL = GPIO_Pin_15

#define led3off() GPIOD->BSRRH = GPIO_Pin_13
#define led4off() GPIOD->BSRRH = GPIO_Pin_12
#define led5off() GPIOD->BSRRH = GPIO_Pin_14
#define led6off() GPIOD->BSRRH = GPIO_Pin_15

#else
#define led2off() GPIOF->BSRRL=GPIO_Pin_9
#define led3off() GPIOF->BSRRL=GPIO_Pin_10

#define led2on() GPIOF->BSRRH=GPIO_Pin_9
#define led3on() GPIOF->BSRRH=GPIO_Pin_10

#define led2toggle() GPIOF->ODR^=GPIO_Pin_9
#define led3toggle() GPIOF->ODR^=GPIO_Pin_10
#endif

#endif