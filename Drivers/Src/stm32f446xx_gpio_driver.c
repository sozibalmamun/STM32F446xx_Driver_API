/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 3 Oct 2022
 *      Author: sozib
 */

//#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"
#include "stdio.h"

/*
 *  Peripheral clock control
 *
 * @function name - GPIO_Peripheral_Clk_Contrl
 * @brif          - this function enables or disables Peripheral clock for the giver GPIO port
 *
 * @param-1       -  Base address of the GPIO peripheral
 * @peram-2       -  ENABLE  or DISABLE macros
 *
 * @return        - none
 * @note          - none
 */
void GPIO_Peripheral_Clk_Contrl(GPIO_RegDef_t *pGPIOx , uint8_t En_Disable ){

	if (En_Disable==ENABLE)
	{

		if(pGPIOx==GPIOA)
		{

			GPIOA_PCLK_EN();

		}else if(pGPIOx==GPIOB){

			GPIOB_PCLK_EN();

		}else if(pGPIOx==GPIOC)
		{

			GPIOC_PCLK_EN();

		}else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();

		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();

		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();

		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();

		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}

	}
	else{
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx==GPIOC){
			GPIOC_PCLK_DI();

		}else if(pGPIOx==GPIOD){
			GPIOD_PCLK_DI();

		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_DI();

		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_DI();

		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_DI();

		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_DI();

		}
	}
}

/*
 * @function name - GPIO_Int
 * @brif          - get value =(GPIO config type << which gpio pin number need to assign)
 *                  get the base addr from  GPIO_RegDef_t structure and set the value
 *
 * @param-1       - GPIO_Handle_t    type data given by user
 * @return        - none
 * @note          - none
 */
void GPIO_Int(GPIO_Handle_t *pGOIO_Handle){
	// Enable peripheral Clock
	GPIO_Peripheral_Clk_Contrl(pGOIO_Handle->pGPIOx ,ENABLE);

	uint32_t temp=0;
	//configure mood of GPIO pin
	if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE <= GPIO_MODE_ANALOG  ){

		temp=(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE<<(2* pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO));

		pGOIO_Handle->pGPIOx->MODER &=~ (0x3 <<pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);  /* clear 2 bit */
		pGOIO_Handle->pGPIOx->MODER |= temp;   /* Set bit */
	}
	else
	{
		if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE == GPIO_MODE_IN_FALLING)// inturupt falling adge
		{
			// 1 configer the FTSR
			EXTI->FTSR |= (1<<pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
			//CLear the RTSR
			EXTI->RTSR &=~ (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);

		}else if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE == GPIO_MODE_IN_RISING)// inturupt rising adge
		{
			// 1 configer the RTSR
			EXTI->RTSR |= (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
			//CLear the FTSR
			EXTI->FTSR &=~ (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);

		}else if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE==GPIO_MODE_IN_RF)// inturupt falling/rising both adge
		{
			// 1 configer the FTSR AND RTSR
			EXTI->FTSR|=(1<<  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
			EXTI->RTSR|=(1<<  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
		}

		//2 Configure the gpio port selection in SYSCFG_EXTICR

		uint8_t temp1=  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO / 4;
		uint8_t temp2=  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO % 4;
		uint8_t PortCode=GPIO_BASEADDR_TO_CODE(pGOIO_Handle->pGPIOx);

		SYSCFG_PCLK_EN(); //before Configure the GPIO port selection in SYSCFG_EXTICR enable SYSCFG PCLK

		SYSCFG->EXTICR[temp1]=PortCode << (temp2*4);

		//3 enable the EXTI Inturput delevery using IMR(inturpt musk resgister)
		EXTI->IMR|=(1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
	}

	//configure speed
	temp=0;
	temp=(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_SPEED<<(2*pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO));
	pGOIO_Handle->pGPIOx->OSPEEDR &=~ (0x3<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);/* clear 2 bit */


	pGOIO_Handle->pGPIOx->OSPEEDR|=temp;

	//configure PULL UP / PULL DOWN settings

	temp=0;
	temp=(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_PUPD)<<(2*pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
	pGOIO_Handle->pGPIOx->PUPDR &=~ (0x3<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);/* clear 2 bit */

	pGOIO_Handle->pGPIOx->PUPDR|=temp;

	// configure output type
	temp=0;
	temp=(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_OPT_TYPE)<<(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
	pGOIO_Handle->pGPIOx->OTYPER &=~ (0x1<<pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);/* clear 2 bit */

	pGOIO_Handle->pGPIOx->OTYPER|=temp;


	//configure alternate functionality
	/*
	 * temp1 is number alternate function  0 or 1
	 * and temp2 is number of alternate function range is [ AF0- AF15 ]
	 * 4* temp2   reason each AF takes  4bit
	 *
	 *
	 * */
	if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE == GPIO_MODE_AL_FUNC ){
		uint8_t temp1, temp2;
		temp1=pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO / 8;
		temp2=pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO % 8;
		pGOIO_Handle->pGPIOx->AFR[temp1] &=~ (0xF<<(4*temp2));
		pGOIO_Handle->pGPIOx->AFR[temp1] |=(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_ALF<<(4*temp2));
	}
}

/*
 *
 * @function name - GPIO_DeInt
 * @brif          -
 *
 *
 * @param-1       - GPIO_RegDef_t type de-intit GPIO Port
 *
 * @return        - none
 * @note          - none
 */

void GPIO_DeInt(GPIO_RegDef_t *pGPIOx){


	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();

	}else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();

	}else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();

	}else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();

	}else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();

	}else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}
}


/*
 *
 * @function name - GPIO_Read_Form_Input_Pin
 * @brif          - (uint8_t)((pGPIOx->IDR>>pin)   & (0x00000001) );
 *                  (uint8_t typecasted)(get Input Data Register base address right shiffted by pin number(1 to 15) and musk(&) with 0x00000001
 *                  here 0x00000001 means only care about least significant bit)
 *
 * @param-1       - GPIO_RegDef_t    type data given by user
 * @param-2       - uint8_t          type Pin number
 * @return        - value = 0 or 1
 * @note          - none
 */
//data read Write
uint8_t GPIO_Read_Form_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t Pin){

	uint8_t value;

	value =(uint8_t)((pGPIOx->IDR >> Pin) & 0x00000001);

	return value;
}

/*
 * @function name - GPIO_Read_Form_Input_Port
 * @brif          -
 *
 * @param-1       - GPIO_RegDef_t    type data given by user
 * @return        - 16 bit
 * @note          - none
 */


uint16_t GPIO_Read_Form_Input_Port(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_Write_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t State){

	if(State==GPIO_PIN_SET){

		pGPIOx->ODR |= (1<<Pin);

	}else{

		pGPIOx->ODR &=~ (1<<Pin);
	}
}
void GPIO_Write_To_Output_Port(GPIO_RegDef_t *pGPIOx ,uint16_t State){

	pGPIOx->ODR= State;
}
void GPIO_Toggole_To_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t Pin){

	/*
	 * toggole work with X-OR( ^ )
	 */

	//	pGPIOx->ODR=pGPIOx->ODR^(1<<Pin);  /* or can use this code */
	pGPIOx->ODR ^=(1<<Pin);
}



/*
 * @function name - GPIO_IRQ_interupt_Config
 * @brif          -
 *
 * @param-1       - IRQ_Number
 * @param-2       - En_Disable
 * @return        -  None
 * @note          - none
 */

// IRQ Configuration and ISR handle
void GPIO_IRQ_interupt_Config(uint8_t IRQ_Number, uint8_t En_Disable){
	/*
	 * ISER0(interupt set enabel register)
	 * there are 8 ISERO (ISER0 to ISER7)
	 * each ISER 32 bit ( 0 to 31)
	 * total IRQ_Number is 81
	 */
	if(En_Disable == ENABLE){
		if(IRQ_Number<=31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1<<IRQ_Number);


		}
		else if(IRQ_Number>31 && IRQ_Number < 64)// 32 to 63
		{
			// program ISER1 register

			*NVIC_ISER1 |= (1<<IRQ_Number%32);


		}
		else if(IRQ_Number>= 64 && IRQ_Number<96)//64  to 95
		{
			// program ISER2 register

			*NVIC_ISER2|= (1<<IRQ_Number%64);

		}
	}else {
		if(IRQ_Number<=31)
		{
			// program ICER0 register

			*NVIC_ICER0|= (1<<IRQ_Number);

		}
		else if(IRQ_Number>31&& IRQ_Number<64)
		{
			// program ICER1 register
			*NVIC_ICER1|= (1<<IRQ_Number%32);

		}
		else if(IRQ_Number>=64 &&IRQ_Number<96)
		{
			// program ICER2 register
			*NVIC_ICER2|= (1<<IRQ_Number%64);
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
	/*
	 * total 60 IPR(0 to 59)
	 * each ipr 32 bit divided by 4 section(0 to 3)
	 * 59*4 =236 section
	 * Each section 8 bit
	 *
	 * so No of IPR= 236/4
	 * no of section= 236%4
	 * if user value is IRQ_Number= 15 then
	 *  IPR   15/4= 3
	 * and section is   15%4= 3
	 *
	 *
	 *
	 *  1.  first lets find out the ipr (interupt priority register)
	 */
	uint8_t iprx_no = IRQ_Number/4;
	uint8_t iprx_section = IRQ_Number%4;

	uint8_t shifting_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR+iprx_no) |= (IRQ_Priority << shifting_amount);


}



void Interupt_type(GPIO_Handle_t *pGOIO_Handle ){
	if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE == GPIO_MODE_IN_FALLING)// inturupt falling adge
	{
		// 1 configer the FTSR
		EXTI->FTSR |= (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
		//CLear the rtsr
		EXTI->RTSR &=~ (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);

	}else if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE == GPIO_MODE_IN_RISING)// inturupt rising adge
	{
		// 1 configer the RTSR
		EXTI->RTSR |= (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
		//CLear the rtsr
		EXTI->FTSR &=~ (1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);


	}else if(pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_MODE==GPIO_MODE_IN_RF)// inturupt falling/rising both adge
	{
		// 1 configer the FTSR AND RTSR
		EXTI->FTSR|=(1<<  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
		EXTI->RTSR|=(1<<  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);
	}

	//2 Configure the gpio port selection in SYSCFG_EXTICR

	uint8_t temp1=  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO / 4;
	uint8_t temp2=  pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO % 4;

	uint8_t PortCode=GPIO_BASEADDR_TO_CODE(pGOIO_Handle->pGPIOx);

	SYSCFG_PCLK_EN();//before Configure the gpio port selection in SYSCFG_EXTICR enable SYSCFG pclk

	SYSCFG->EXTICR[temp1] = PortCode << (temp2*4);


	//3 enable the EXTI Inturput delevery using IMR(inturpt musk resgister)
	//	EXTI->IMR|=(1<< pGpio->pGPIO_pin_Config->GPIO_PIN_NO);
	EXTI->IMR|=(1<< pGOIO_Handle->pGPIO_pin_Config.GPIO_PIN_NO);

}
void GPIO_IRQ_Handling( uint8_t Pin)
{
	//	 clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << Pin))
	{// clear
		EXTI->PR |= (1 << Pin);
	}
}






