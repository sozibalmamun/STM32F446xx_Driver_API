/*
 * stm32f446xx_SPI_driver.c
 *
 *  Created on: 31 Oct 2022
 *      Author: sozib
 */


#include "stm32f446xx.h"



// peripheral clock control
/*
 * @function name SPI_Peripheral_Clk_Contrl
 * @param 1       - SPI_RegDef_t type
 * @param 2       - uint8_t type
 * note- enable the rcc peripheral clock
 */
void SPI_Peripheral_Clk_Contrl(SPI_RegDef_t *pSPIx , uint8_t En_Disable ){

	if (En_Disable==ENABLE)
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_EN();

		}else if(pSPIx==SPI2){

			SPI2_PCLK_EN();

		}else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();

		}else if(pSPIx==SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else{
		if(pSPIx==SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx==SPI3)
		{
			SPI3_PCLK_DI();

		}else if(pSPIx==SPI4)
		{
			SPI4_PCLK_DI();

		}
	}
}

/*
 * function name SPI_Int
 * @param    -  SPI_Handle_t type
 * return none
 */
void SPI_Int(SPI_Handle_t *pSPIx_Handle){



	SPI_Peripheral_Clk_Contrl(pSPIx_Handle->pSPIx, ENABLE);
	uint32_t temp=0;
	// SPI Device Mode
	temp|=pSPIx_Handle->Spi_Config.SPI_Device_Mode<<SPI_CR1_MSTR;

	// SPI Bus Configuration
	if(pSPIx_Handle->Spi_Config.SPI_Bus_Config==SPI_BUS_CONFIG_FD){

		// BIDI Clear
		temp &=~ (1<<SPI_CR1_BIDI_MODE);

	}else if(pSPIx_Handle->Spi_Config.SPI_Bus_Config==SPI_BUS_CONFIG_HD){

		// BIDI Mode should be set
		temp |= (1<<SPI_CR1_BIDI_MODE);

	}else if(pSPIx_Handle->Spi_Config.SPI_Bus_Config==SPI_BUS_CONFIG_SIMPLEX_RX_ONLY){

		// BIDI Should be Clear
		temp &=~ (1<<SPI_CR1_BIDI_MODE);

		// RxONlY must be set
		temp |= (1<<SPI_CR1_RX_ONLY);
	}
	// Configure SPI baud rate
	temp |= pSPIx_Handle->Spi_Config.SPI_SclkSpeed<<SPI_CR1_BR;

	//Configure the DFF
	temp |= pSPIx_Handle->Spi_Config.SPI_DFF<<SPI_CR1_DFF;


	// Configure the CPOL
	temp |= pSPIx_Handle->Spi_Config.SPI_CPOL<<SPI_CR1_CPOL;


	// Configure the CPHA
	temp |= pSPIx_Handle->Spi_Config.SPI_CPHA<<SPI_CR1_CPHA;

	// Software slave management for NSS pin
	temp |= pSPIx_Handle->Spi_Config.SPI_SSM<<SPI_CR1_SSM;



	pSPIx_Handle->pSPIx->CR1=temp;

}
/*
 * function name SPI_DeInt
 * @param    -  SPI_Handle_t type
 * return none
 */
void SPI_DeInt(SPI_RegDef_t *pSPIx){

	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET() ;
	}else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET() ;
	}else if(pSPIx==SPI4)
	{
		SPI4_REG_RESET() ;
	}
}

/*
 * function name Spi_Get_flagStatus
 * @param 1    -  SPI_RegDef_t type
 * @param 2    -  uint32_t type
 * note        - this function used for check SR is set or not
 *
 * return none
 */
uint8_t Spi_Get_flagStatus( SPI_RegDef_t *pSPIx ,uint32_t FlagName ){

	if(pSPIx->SR &FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * Data send and Receive
 *@param 1  - *pSPIx (SPIx base address),
 *@param 2  -*pTxBuffer(Data),
 *@param 3  - data length
 *@return   - none
 *@Note     - This is blocking call
 */
void SPI_Send_Data(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Data_len){
	while(Data_len>0)
	{//1. wait until TXE is set
		while(Spi_Get_flagStatus(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);
		// 2 Check the DFF iS CR1
		if( pSPIx->CR1&(1<<SPI_CR1_DFF)){
			// DFF IS 16 BIT
			//1. load the data in Data Register

			pSPIx->DR=*((uint16_t*)pTxBuffer);
			Data_len--;
			Data_len--;
			(uint16_t*)pTxBuffer++;


		}else{
			// DFF IS 8 BIT
			pSPIx->DR=*pTxBuffer;
			Data_len--;
			pTxBuffer++;
		}

	}
}


/*
 * function name    - SPI_Receive_Data
 *@param 1  		- *pSPIx (SPIx base address),
 *@param 2  		-*pTxBuffer(Data),
 *@param 3  		- data length
 *@return   		- none
 *@Note    			-
 */
void SPI_Receive_Data(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Data_len){
	while(Data_len>0)
	{//1. wait until RXNE is set
		while(Spi_Get_flagStatus(pSPIx, SPI_RXNE_FLAG)==FLAG_RESET);
		// 2 Check the DFF iS CR1
		if( pSPIx->CR1&(1<<SPI_CR1_DFF)){
			// DFF IS 16 BIT
			//1. load the data to pRxBuffer

			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Data_len--;
			Data_len--;
			(uint16_t*)pRxBuffer++;


		}else{
			// DFF IS 8 BIT
			*pRxBuffer = pSPIx->DR;
			Data_len--;
			pRxBuffer++;
		}
	}
}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQ_interupt_Config(uint8_t IRQ_Number, uint8_t En_Disable);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void SPI_IRQ_Handling( SPI_Handle_t *pHandle);
/*
 * Other Peripheral Control APIs
 */





/*
 * @fnction name      -   SPI_Peripheral_Control
 * @param 1           -   SPI_RegDef_t type
 * @param 2           -   uint8_t type
 * NOTE               - befor sending data SPI_CR1_SPE hase to set or enable
 * return             - none
 */
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t En_Disable){
	if(En_Disable==ENABLE){
		// SPI enable  set
		pSPIx->CR1|= (1<<SPI_CR1_SPE);
	}else
	{// SPI enable clear
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

/*
 * @fnction name      -   SPI_SSI_CONFIG(Internal slave select)
 * @param 1            -   SPI_RegDef_t type
 * @param 2            -   uint8_t type
 * note - this function need wheen SSM=1 is enable
 *
 * return - none
 */
void SPI_SSI_CONFIG(SPI_RegDef_t *pSPIx, uint8_t En_Disable){

	if(En_Disable==ENABLE){
		pSPIx->CR1|= (1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}

/*
 * @fnction name      -   SPI_SSOE_CONFIG(Slave select output enable)
 * @param 1           -   SPI_RegDef_t type
 * @param 2           -   uint8_t type
 * note               - this function need wheen SSM=0 is disable
 *                      when(NSS) hardware slave management are use
 *
 * return             - none
 */

void SPI_SSOE_CONFIG(SPI_RegDef_t *pSPIx, uint8_t En_Disable){
	if(En_Disable==ENABLE){
			pSPIx->CR2|= (1<<SPI_CR2_SSOE);
		}else
		{
			pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
		}
}





