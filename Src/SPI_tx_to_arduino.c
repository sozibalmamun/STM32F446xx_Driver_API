/*
 * SPI_tx_to_arduino.c
 *
 *  Created on: 15 Nov 2022
 *      Author: sozib
 */
#include "stm32f446xx.h"
#include "string.h"

/*
 * SPI2
 * ALT Function Mode : 5
 * MOSI-  PB15
 * MISO-  PB14
 * CLK -  PB13
 * NSS -  PB12
 */

void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx= GPIOB;
	SPIPins.pGPIO_pin_Config.GPIO_PIN_MODE=GPIO_MODE_AL_FUNC;
	SPIPins.pGPIO_pin_Config.GPIO_PIN_ALF=5;
	SPIPins.pGPIO_pin_Config.GPIO_PIN_OPT_TYPE=GPIO_OUT_TYPE_PP;
	SPIPins.pGPIO_pin_Config.GPIO_PIN_PUPD=GPIO_PIN_NO_PUPD;
	SPIPins.pGPIO_pin_Config.GPIO_PIN_SPEED=GPIO_SPEED_FREQ_VERY_HIGH;

	// SCLK
	SPIPins.pGPIO_pin_Config.GPIO_PIN_NO=GPIO_PIN_13;
	GPIO_Int(&SPIPins);

	// MOSI
	SPIPins.pGPIO_pin_Config.GPIO_PIN_NO=GPIO_PIN_15;
	GPIO_Int(&SPIPins);

	// MISO
	//	SPIPins.pGPIO_pin_Config.GPIO_PIN_NO=GPIO_PIN_14;
	//	GPIO_Int(&SPIPins);

	// NSS
	SPIPins.pGPIO_pin_Config.GPIO_PIN_NO=GPIO_PIN_12;
	GPIO_Int(&SPIPins);

}
void SPI2_Inits(void){
	SPI_Handle_t SPI2_Handle;
	SPI2_Handle.pSPIx=SPI2;
	SPI2_Handle.Spi_Config.SPI_Bus_Config=SPI_BUS_CONFIG_FD;
	SPI2_Handle.Spi_Config.SPI_Device_Mode=SPI_DEVICE_MODE_MASTER;

	SPI2_Handle.Spi_Config.SPI_SclkSpeed=SPI_CLK_SPE_DIV_8;// Generate 8MH Clock
	SPI2_Handle.Spi_Config.SPI_DFF=SPI_DFF_8BIT;
	SPI2_Handle.Spi_Config.SPI_CPOL=SPI_CPOL_LOW;
	SPI2_Handle.Spi_Config.SPI_CPHA=SPI_CPHA_LOW;
	SPI2_Handle.Spi_Config.SPI_SSM=SPI_SSM_DI; // Hardware slave management for NSS pin

	SPI_Int(&SPI2_Handle);

}


void GPIO_BUTTON_INT(void){
	GPIO_Handle_t Switch;
	Switch.pGPIOx=GPIOC;
	Switch.pGPIO_pin_Config.GPIO_PIN_NO = GPIO_PIN_13;// pin number
	Switch.pGPIO_pin_Config.GPIO_PIN_MODE = GPIO_MODE_IN;// output type
	Switch.pGPIO_pin_Config.GPIO_PIN_SPEED = GPIO_SPEED_FREQ_VERY_HIGH;// speed
	Switch.pGPIO_pin_Config.GPIO_PIN_PUPD = GPIO_PIN_NO_PUPD;// no pull up/down

	GPIO_Int(&Switch);
}
void delay(uint8_t value){
	for(uint32_t i=0;i<500000/value;i++);
}
int main(void){

	char user_data[]="This function is used to initialize the GPIO pins to behave as SPI2 pins";

	GPIO_BUTTON_INT();
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();
	SPI2_Inits();
	/*
	 * SSOE (Slave select output enable)
	 * making SSOE 1 does NSS output enable
	 * the NSS pin is automatically managed by the hardware
	 * when SPE=1, NSS will be puled low
	 * and SPE=0, NSS will be puled high
	 *
	 * this is required when we use SSM=0
	 * or SSM disable (Hardware slave management are used)
	 */
	SPI_SSOE_CONFIG( SPI2, ENABLE);

	while(1){

		while(!GPIO_Read_Form_Input_Pin(GPIOC,GPIO_PIN_13))
		{
			delay(5);
			/*
			 *  Enable the SPI2 Peripheral( SPE OR SPI enable) after all the setup done
			 *  because if it done before, SPI is busy in Data communication
			 */
			SPI_Peripheral_Control(SPI2, ENABLE);

			// first sent the data length information
			uint8_t data_length= strlen(user_data);
			SPI_Send_Data(SPI2, &data_length, 1);
			// sent the data
			SPI_Send_Data(SPI2, (uint8_t*)user_data, data_length);

			/*
			 * first let confirm the BUSY flag set or not
			 * if it is 1 then SPI is busy to communication
			 * if it is 0 then SPI is free and then we can disable the @SPE register
			 */
			while(Spi_Get_flagStatus( SPI2,SPI_BUSY_FLAG)==FLAG_SET);

			/*
			 *  disable the SPI2 Peripheral( SPE OR SPI disable) after sending done
			 *  @SPE
			 */
			SPI_Peripheral_Control(SPI2, DISABLE);

		}
	}
	return 0;
}
