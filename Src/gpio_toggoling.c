/*
 * gpio_toggoling.c
 *
 *  Created on: 13 Oct 2022
 *      Author: sozib
 */

#include "stm32f446xx.h"
#include "stdint.h"

void delay(void){
	for(uint32_t i=0;i<500000;i++);
}


int main(void){

	GPIO_Handle_t Gpio_led;
	GPIO_Handle_t sw;

/* output config */
	Gpio_led.pGPIOx=GPIOA;
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_NO = GPIO_PIN_5;// pin number

	Gpio_led.pGPIO_pin_Config.GPIO_PIN_MODE = GPIO_MODE_OUT;// output type
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_SPEED = GPIO_SPEED_FREQ_HIGH;// speed

//	Gpio_led.pGPIO_pin_Config.GPIO_PIN_OPT_TYPE = GPIO_OUT_TYPE_PP;//push pull
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_OPT_TYPE = GPIO_OUT_TYPE_OD;//open drain

//	Gpio_led.pGPIO_pin_Config->GPIO_PIN_PUPD = GPIO_PIN_NO_PUPD;// no pull up/down
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_PUPD = GPIO_PIN_PU;// no pull up/down
/*output config end*/


	GPIO_Peripheral_Clk_Contrl(GPIOA, GPIO_PIN_SET);
	GPIO_Int(&Gpio_led);

	while(1){
/*
 * for toggoling led use push pull
 * for opendrain  use external pullup
 *
 * */

		GPIO_Toggole_To_Output_Pin(GPIOA, GPIO_PIN_5);
		delay();



	}
	return 0;
}
