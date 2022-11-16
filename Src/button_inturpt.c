/*
 * button_inturpt.c
 *
 *  Created on: Oct 27, 2022
 *      Author: sozib
 */


/*
 * button_interupt.c
 *
 *  Created on: 24 Oct 2022
 *      Author: sozib
 */
#include "stm32f446xx.h"
#include "string.h"

void EXTI15_10_IRQHandler(void);

void delay(uint8_t value){
	for(uint32_t i=0;i<50000/value;i++);
}
int main(void){


	GPIO_Handle_t Led_cinfig , Button_config;

	memset(&Led_cinfig,0,sizeof(Led_cinfig));
	memset(&Button_config,0,sizeof(Button_config));

	/* output config */
	Led_cinfig.pGPIOx=GPIOA;
	Led_cinfig.pGPIO_pin_Config.GPIO_PIN_NO = GPIO_PIN_6;// pin number
	Led_cinfig.pGPIO_pin_Config.GPIO_PIN_MODE = GPIO_MODE_OUT;// output type
	Led_cinfig.pGPIO_pin_Config.GPIO_PIN_SPEED = GPIO_SPEED_FREQ_VERY_HIGH;// speed
	Led_cinfig.pGPIO_pin_Config.GPIO_PIN_OPT_TYPE = GPIO_OUT_TYPE_PP;//open drain
	Led_cinfig.pGPIO_pin_Config.GPIO_PIN_PUPD = GPIO_PIN_NO_PUPD;// no pull up/down

	GPIO_Peripheral_Clk_Contrl(GPIOA, ENABLE);
	GPIO_Int(&Led_cinfig);
	/*output config end*/


	// button
	Button_config.pGPIOx=GPIOB;
	Button_config.pGPIO_pin_Config.GPIO_PIN_NO = GPIO_PIN_13;// pin number
	Button_config.pGPIO_pin_Config.GPIO_PIN_MODE = GPIO_MODE_IN_FALLING;
	Button_config.pGPIO_pin_Config.GPIO_PIN_SPEED = GPIO_SPEED_FREQ_VERY_HIGH;// speed
	Button_config.pGPIO_pin_Config.GPIO_PIN_PUPD = GPIO_PIN_PU;//pull up

	GPIO_Peripheral_Clk_Contrl(GPIOB, ENABLE);
	GPIO_Int(&Button_config);

	// IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO_15);
	GPIO_IRQ_interupt_Config(IRQ_NO_EXTI15_10, ENABLE);

	while(1)
	{


	}
	return 0;
}


void EXTI15_10_IRQHandler(void){
	delay(2);
	GPIO_IRQ_Handling(GPIO_PIN_13);
	GPIO_Toggole_To_Output_Pin(GPIOA, GPIO_PIN_6);

}
