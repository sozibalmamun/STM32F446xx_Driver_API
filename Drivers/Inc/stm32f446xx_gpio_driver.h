/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 3 Oct 2022
 *      Author: sozib
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include "stdint.h"


typedef struct {

	__v	uint8_t GPIO_PIN_NO;			/* possible value from @PIN_NO_MACROS*/
	__v	uint8_t GPIO_PIN_MODE;		    /* possible value from @PIN_MODE_MACROS*/
	__v	uint8_t GPIO_PIN_SPEED; 		/* possible value from @PIN_SPEED_MACROS*/
	__v	uint8_t GPIO_PIN_PUPD;			/* possible value from @PIN_PUPD_MACROS*/
	__v	uint8_t GPIO_PIN_OPT_TYPE;		/* possible value from @PIN_OPT_TYPE_MACROS*/
	__v	uint8_t GPIO_PIN_ALF;			/* possible value from @PIN_ALF_MACROS*/

}GPIO_Config_t;                     /* GPIO config types structure */


/*
 * handale structure for GPIO pin
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx;           /* This pointer hold the base address  of GPIOx register */
	GPIO_Config_t pGPIO_pin_Config;

}GPIO_Handle_t;           /* using this structure handle GPIO int  */

/*
 * @PIN_MODE_MACROS
 * gpio pin INPUT  possible moods
 */

#define GPIO_MODE_IN 			0x0
#define GPIO_MODE_OUT 			0x1
#define GPIO_MODE_AL_FUNC 		0x2
#define GPIO_MODE_ANALOG 		0x3

#define GPIO_MODE_IN_FALLING 	0x4
#define GPIO_MODE_IN_RISING		0x5
#define GPIO_MODE_IN_RF      	0x6    /*GPIO INPUT RISING_FALLING AGE*/

/*
 * @PIN_OPT_TYPE_MACROS
 * gpio pin OUTPUT TYPE
 */

#define GPIO_OUT_TYPE_PP	    0x0 // PUSH-PULL
#define GPIO_OUT_TYPE_OD 		0x1 // OPEN DRAIN

/*
 * @PIN_SPEED_MACROS
 * gpio pin OUTPUT POSIBLE SPEED
 */
#define  GPIO_SPEED_FREQ_LOW         0x0  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM      0x1  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH        0x2  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define  GPIO_SPEED_FREQ_VERY_HIGH   0x3  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */

/*
 * @PIN_PUPD_MACROS
 * gpio pin PULLUP/PULLDOWN CONFIG MACROS
 */

#define GPIO_PIN_NO_PUPD	  0
#define GPIO_PIN_PU	 	     1
#define GPIO_PIN_PD		     2


/*
 * @PIN_NO_MACROS
 * gpio POSSIBLE pin NUMBER
 */
#define GPIO_PIN_0                 0  /* Pin 0 selected    */
#define GPIO_PIN_1                 1  /* Pin 1 selected    */
#define GPIO_PIN_2                 2  /* Pin 2 selected    */
#define GPIO_PIN_3                 3  /* Pin 3 selected    */
#define GPIO_PIN_4                 4  /* Pin 4 selected    */
#define GPIO_PIN_5                 5  /* Pin 5 selected    */
#define GPIO_PIN_6                 6  /* Pin 6 selected    */
#define GPIO_PIN_7                 7  /* Pin 7 selected    */
#define GPIO_PIN_8                 8 /* Pin 8 selected    */
#define GPIO_PIN_9                 9  /* Pin 9 selected    */
#define GPIO_PIN_10                10  /* Pin 10 selected   */
#define GPIO_PIN_11                11  /* Pin 11 selected   */
#define GPIO_PIN_12                12  /* Pin 12 selected   */
#define GPIO_PIN_13                13  /* Pin 13 selected   */
#define GPIO_PIN_14                14  /* Pin 14 selected   */
#define GPIO_PIN_15                15  /* Pin 15 selected   */


/*
 * prototype API
 */

// peripheral clock control
void GPIO_Peripheral_Clk_Contrl(GPIO_RegDef_t *pGPIOx , uint8_t En_Disable );

// int and deint
void GPIO_Int(GPIO_Handle_t *pGOIO_Handle);
void GPIO_DeInt(GPIO_RegDef_t *pGPIOx);

//data read Write
uint8_t GPIO_Read_Form_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);
uint16_t GPIO_Read_Form_Input_Port(GPIO_RegDef_t *pGPIOx);
void GPIO_Write_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t State);
void GPIO_Write_To_Output_Port(GPIO_RegDef_t *pGPIOx ,uint16_t State);
void GPIO_Toggole_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);

// IRQ Configuration and ISR handle

void GPIO_IRQ_interupt_Config(uint8_t IRQ_Number, uint8_t En_Disable);
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void GPIO_IRQ_Handling( uint8_t Pin);
#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */














