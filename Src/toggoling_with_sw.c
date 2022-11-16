/*
 * toggoling_with_sw.c
 *
 *  Created on: 21 Oct 2022
 *      Author: sozib
 */


/*
 * gpio_toggoling.c
 *
 *  Created on: 13 Oct 2022
 *      Author: sozib
 */

#include "stm32f446xx.h"
#include "stdint.h"

#define LOW 0
#define HIGH 1

#define BUTTON_PRESSED LOW


void delay(uint8_t value){
	for(uint32_t i=0;i<500000/value;i++);
}


int main(void){

	GPIO_Handle_t Gpio_led,Switch;

	/* output config */
	Gpio_led.pGPIOx=GPIOA;
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_NO = GPIO_PIN_5;// pin number
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_MODE = GPIO_MODE_OUT;// output type
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_SPEED = GPIO_SPEED_FREQ_HIGH;// speed
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_OPT_TYPE = GPIO_OUT_TYPE_PP;//push pull
	Gpio_led.pGPIO_pin_Config.GPIO_PIN_PUPD = GPIO_PIN_NO_PUPD;// no pull up/down

	GPIO_Peripheral_Clk_Contrl(GPIOA, ENABLE);
	GPIO_Int(&Gpio_led);
	/*output config end*/


	/* Switch config */

	Switch.pGPIOx=GPIOC;
	Switch.pGPIO_pin_Config.GPIO_PIN_NO = GPIO_PIN_13;// pin number
	Switch.pGPIO_pin_Config.GPIO_PIN_MODE = GPIO_MODE_IN;// output type
	Switch.pGPIO_pin_Config.GPIO_PIN_SPEED = GPIO_SPEED_FREQ_VERY_HIGH;// speed
	Switch.pGPIO_pin_Config.GPIO_PIN_PUPD = GPIO_PIN_NO_PUPD;// no pull up/down
	GPIO_Peripheral_Clk_Contrl(GPIOC, ENABLE);

	GPIO_Int(&Switch);




	while(1)
	{
		if(GPIO_Read_Form_Input_Pin(GPIOC,GPIO_PIN_13)==BUTTON_PRESSED)
		{
			delay(2);
			GPIO_Toggole_To_Output_Pin(GPIOA, GPIO_PIN_5);
		}else{
			delay(5);
			GPIO_Toggole_To_Output_Pin(GPIOA, GPIO_PIN_5);
		}
	}
	return 0;
}
