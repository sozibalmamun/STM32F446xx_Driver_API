/*
 * stm32f446xx_SPI_driver.h
 *
 *  Created on: 31 Oct 2022
 *      Author: sozib
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 * CONfigaration Structure for SPIx Peripheral
 */

typedef struct{
							// ALL  Possible values
uint8_t SPI_Device_Mode;	//@SPI_Device_Mode
uint8_t SPI_Bus_Config;		//@SPI_Bus_Config
uint8_t SPI_SclkSpeed;		//@SPI_SclkSpeed
uint8_t SPI_DFF;			//@SPI_DFF
uint8_t SPI_CPOL;			//@SPI_CPOL
uint8_t SPI_CPHA;			//@SPI_CPHA
uint8_t SPI_SSM;			//@SPI_SSM

}SPI_Config_t;

typedef struct{

	SPI_RegDef_t *pSPIx;
	SPI_Config_t Spi_Config;

}SPI_Handle_t;

/*
 * @SPI_Device_Mode
 */
#define SPI_DEVICE_MODE_MASTER 		1
#define SPI_DEVICE_MODE_SLAVE 		0

/*
 * @SPI_Bus_Config
 */
#define SPI_BUS_CONFIG_FD				        1	//  SPI BUS Configuration full-deplex
#define SPI_BUS_CONFIG_HD				        2	//  SPI BUS Configuration half-deplex
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY      	3	//  SPI BUS Configuration simplex rx

/*
 * @SPI_SclkSpeed
 */
#define SPI_CLK_SPE_DIV_2 			0
#define SPI_CLK_SPE_DIV_4 			1
#define SPI_CLK_SPE_DIV_8 			2
#define SPI_CLK_SPE_DIV_16 			3
#define SPI_CLK_SPE_DIV_32 			4
#define SPI_CLK_SPE_DIV_64 			5
#define SPI_CLK_SPE_DIV_128 		6
#define SPI_CLK_SPE_DIV_256 		7

/*
 * @SPI_DFF- SPI Data frame format
 */

#define SPI_DFF_8BIT 			0
#define SPI_DFF_16BIT 			1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH			HIGH
#define SPI_CPOL_LOW 			LOW

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH			HIGH
#define SPI_CPHA_LOW 			LOW

/*
 * @SPI_SSM (Software Slave Management)
 */
#define SPI_SSM_DI				0
#define SPI_SSM_EN				1

/*
 * SPI Related Status Flags Definitions
 */
#define SPI_RXNE_FLAG    	 (1<< SPI_SR_RXNE)
#define SPI_TXE_FLAG    	 (1<< SPI_SR_TXE)
#define SPI_CHSIDE_FLAG      (1<< SPI_SR_CHSIDE)
#define SPI_UDR_FLAG     	 (1<< SPI_SR_UDR)
#define SPI_CRC_ERR_FLAG     (1<< SPI_SR_CRC_ERR )
#define SPI_MODF_FLAG     	 (1<< SPI_SR_MODF)
#define SPI_OVR_FLAG     	 (1<< SPI_SR_OVR)
#define SPI_BUSY_FLAG    	 (1<< SPI_SR_BSY)
#define SPI_FRE_FLAG     	 (1<< SPI_SR_FRE)




/*
 * ********************************   **************************  ***********************************
 *                                    API support by this Driver
 * ********************************   **************************  ***********************************
 */

/*
 * Peripheral Clock Setup
 *
 */
void SPI_Peripheral_Clk_Contrl(SPI_RegDef_t *pSPIx , uint8_t En_Disable );

/*
 * Int and De-init
 */
void SPI_Int(SPI_Handle_t *pSPIx_Handle);
void SPI_DeInt(SPI_RegDef_t *pSPIx);

/*
 * Data send and Receive
 *
 */
void SPI_Send_Data(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Data_len);
void SPI_Receive_Data(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Data_len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQ_interupt_Config(uint8_t IRQ_Number, uint8_t En_Disable);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void SPI_IRQ_Handling( SPI_Handle_t *pHandle);



/*
 * Other Peripheral Control APIs
 */
void SPI_SSI_CONFIG(SPI_RegDef_t *pSPIx, uint8_t En_Disable);
void SPI_SSOE_CONFIG(SPI_RegDef_t *pSPIx, uint8_t En_Disable);

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t En_Disable);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
