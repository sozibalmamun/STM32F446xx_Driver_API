/*
 * stm32f446xx.h
 *
 *  Created on: Sep 28, 2022
 *      Author: sozib
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __v volatile


/*
 * Table 4-2 NVIC register summary
 * Nested Vectored Interrupt Controller(NVIC)
 * Processor Specific Details
 * Arm cortex Mx processor NIVIC ISERx(Interrupt Set-enable Registers) Register addresses
 */
#define NVIC_ISER0      ( (__v uint32_t*)0xE000E100)
#define NVIC_ISER1      ( (__v uint32_t*)0xE000E104)
#define NVIC_ISER2      ( (__v uint32_t*)0xE000E108)
#define NVIC_ISER3      ( (__v uint32_t*)0xE000E10c)
/*
 * Arm cortex Mx processor NIVIC ISERx Register addresses
 * Interrupt Clear-enable Registers
 *
 */
#define NVIC_ICER0      ((__v uint32_t*)0xE000E180)
#define NVIC_ICER1      ((__v uint32_t*)0xE000E184)
#define NVIC_ICER2      ((__v uint32_t*)0xE000E188)
#define NVIC_ICER3      ((__v uint32_t*)0xE000E18c)
/*
 * Arm cortex Mx processor NIVIC Priority Register base addresses
 * Interrupt Priority Registers
 */
#define NVIC_PR_BASE_ADDR ((__v uint32_t*)0xE000E400)
/*
 * Arm cortex Mx processor no priority register bits implemented
 * may be value not same for other Arm cortex family
 */
#define NO_PR_BITS_IMPLEMENTED 	   4

#define FLASH_BASEADDR             0x08000000U  //3.3 Embedded Flash memory   Sector 0
#define SRAM1_BASEADDR             0x20000000U // Memory mapping vs. Boot mode/physical remap in STM32F446xx
#define SRAM                       SRAM1_BASEADDR

#define SRAM2_BASEADDR             0x2001UC000 //Memory mapping vs. Boot mode/physical remap in STM32F446xx
#define RAM_BASEADDR               0x1FFF0000U    //Flash module organization    (sYSTEM MEMORY)






/*
 *  AHBx and APBx bus peripheral base addresses
 *
 */

#define PERIPHERAL_BASE_ADDR       0x40000000U
#define APB1_BASE_ADDR             PERIPHERAL_BASE_ADDR

#define APB2_BASE_ADDR             0x40010000U

#define AHB1_BASE_ADDR             0x40020000U

#define AHB2_BASE_ADDR             0x50000000U
#define AHB3_BASE_ADDR             0xA0001000U

/*
 * base addresses of peripheral which are hanging on APB1
 *
 */

#define TIM2_BASE_ADDR 		       (APB1_BASE_ADDR+0x0000)
#define TIM3_BASE_ADDR 			   (APB1_BASE_ADDR+0x0400)
#define TIM4_BASE_ADDR 			   (APB1_BASE_ADDR+0x0800)
#define TIM5_BASE_ADDR 			   (APB1_BASE_ADDR+0x0C00)
#define TIM6_BASE_ADDR 			   (APB1_BASE_ADDR+0x1000)
#define TIM7_BASE_ADDR             (APB1_BASE_ADDR+0x1400)
#define TIM12_BASE_ADDR            (APB1_BASE_ADDR+0x1800)
#define TIM13_BASE_ADDR            (APB1_BASE_ADDR+0x1C00)
#define TIM14_BASE_ADDR            (APB1_BASE_ADDR+0x2000)

#define RTC_BKP_REG_BASE_ADDR      (APB1_BASE_ADDR+0x2800)
#define WWDG_BASE_ADDR             (APB1_BASE_ADDR+0x2C00)
#define IWDG_BASE_ADDR             (APB1_BASE_ADDR+0x3000)

#define SPI2_I2S2_BASE_ADDR        (APB1_BASE_ADDR+0x3800)
#define SPI3_I2S3_BASE_ADDR        (APB1_BASE_ADDR+0x3C00)

#define SPDIF_RX_BASE_ADDR         (APB1_BASE_ADDR+0x4000)
#define USART2_BASE_ADDR           (APB1_BASE_ADDR+0x4400)
#define USART3_BASE_ADDR           (APB1_BASE_ADDR+0x4800)
#define UART4_BASE_ADDR            (APB1_BASE_ADDR+0x4C00)
#define UART5_BASE_ADDR            (APB1_BASE_ADDR+0x5000)

#define I2C1_BASE_ADDR             (APB1_BASE_ADDR+0x5400)
#define I2C2_BASE_ADDR             (APB1_BASE_ADDR+0x5800)
#define I2C3_BASE_ADDR             (APB1_BASE_ADDR+0x5C00)

#define CAN1_BASE_ADDR             (APB1_BASE_ADDR+0x6400)
#define CAN2_BASE_ADDR             (APB1_BASE_ADDR+0x6800)
#define HDMI_CEC_BASE_ADDR     	   (APB1_BASE_ADDR+0x6C00)
#define PWR_BASE_ADDR     		   (APB1_BASE_ADDR+0x4000)
#define DAC_BASE_ADDR              (APB1_BASE_ADDR+0x7400)

/*
 * base addresses of peripheral which are hanging on APB2
 */

#define TIM1_BASE_ADDR              (APB2_BASE_ADDR+0x0000)
#define TIM8_BASE_ADDR              (APB2_BASE_ADDR+0x0400)
#define USART1_BASE_ADDR            (APB2_BASE_ADDR+0x1000)
#define USART6_BASE_ADDR            (APB2_BASE_ADDR+0x1400)
#define ADC1_ADC2_ADC3_BASE_ADDR    (APB2_BASE_ADDR+0x2000)
#define SDMMC_BASE_ADDR             (APB2_BASE_ADDR+0x2C00)

#define SPI1_BASE_ADDR              (APB2_BASE_ADDR+0x3000)
#define SPI4_BASE_ADDR              (APB2_BASE_ADDR+0x3400)

#define SYSCFG_BASE_ADDR            (APB2_BASE_ADDR+0x3800)

#define EXTI_BASE_ADDR              (APB2_BASE_ADDR+0x3C00)

#define TIM9_BASE_ADDR              (APB2_BASE_ADDR+0x4000)
#define TIM10_BASE_ADDR             (APB2_BASE_ADDR+0x4400)
#define TIM11_BASE_ADDR             (APB2_BASE_ADDR+0x4800)
#define SAI1_BASE_ADDR              (APB2_BASE_ADDR+0x5800)
#define SAI2_BASE_ADDR              (APB2_BASE_ADDR+0x5C00)
/*
 * base addresses of peripheral which are hanging on AHB1
 */

#define GPIOA_BASE_ADDR            (AHB1_BASE_ADDR+0x0000)       /* HERE  0x0000 OFFSET OF GPIOA */

#define GPIOB_BASE_ADDR            (AHB1_BASE_ADDR+0x0400)       /* HERE  0x0400  OFFSET OF GPIOB */

#define GPIOC_BASE_ADDR            (AHB1_BASE_ADDR+0x0800)       /* HERE  0x0800 OFFSET OF GPIOC */

#define GPIOD_BASE_ADDR 		   (AHB1_BASE_ADDR+0x0C00)       /* HERE  0x0C00 OFFSET OF GPIOD */
#define GPIOE_BASE_ADDR 		   (AHB1_BASE_ADDR+0x1000)       /* HERE  0x1000 OFFSET OF GPIOE */
#define GPIOF_BASE_ADDR            (AHB1_BASE_ADDR+0x1400)       /* HERE  0x1400 OFFSET OF GPIOF */
#define GPIOG_BASE_ADDR            (AHB1_BASE_ADDR+0x1800)       /* HERE  0x1800 OFFSET OF GPIOG */
#define GPIOH_BASE_ADDR            (AHB1_BASE_ADDR+0x1C00)       /* HERE  0x1C00 OFFSET OF GPIOH */

#define CRC_BASE_ADDR              (AHB1_BASE_ADDR+0x3000)       /* HERE  0x3000 OFFSET OF CRC_BASE_ADDR */

#define RCC_BASE_ADDR              (AHB1_BASE_ADDR+0x3800)       /* HERE  0x3800 OFFSET OF RCC_BASE_ADDR */

#define FLASH_INT_REG_BASE_ADDR    (AHB1_BASE_ADDR+0x3C00)       /* HERE  0x3C00 OFFSET OF FLASH_INT_REG_BASE_ADDR */
#define BKPSRAM_BASE_ADDR          (AHB1_BASE_ADDR+0x4000)       /* HERE  0x4000 OFFSET OF BKPSRAM_BASE_ADDR */
#define DMA1_BASE_ADDR             (AHB1_BASE_ADDR+0x6000)       /* HERE  0x6000 OFFSET OF DMA1_BASE_ADDR */
#define DMA2_BASE_ADDR             (AHB1_BASE_ADDR+0x6400)       /* HERE  0x6400 OFFSET OF DMA2_BASE_ADDR */
//#define USB_OTG_HS_BASE_ADDR       (AHB1_BASE_ADDR+0x1000)       /* HERE  0x1000 OFFSET OF GPIOA */


/*
 * base addresses of peripheral which are hanging on AHB2
 */
#define USB_OTG_FS_BASE_ADDR       (AHB2_BASE_ADDR+0x0000)       /* HERE  0x0000 OFFSET OF USB_OTG_FS_BASE_ADDR */
#define DCMI_BASE_ADDR             (AHB2_BASE_ADDR+0x1000)       /* HERE  0x1000 OFFSET OF DCMI_BASE_ADDR */


/*
 * base addresses of peripheral which are hanging on AHB3
 */

#define FMC_CONTROL_REG_BASE_ADDR            (AHB3_BASE_ADDR+0x1000)       /* HERE  0x1000 OFFSET OF GPIOA */
#define QUADSPI_REG_BASE_ADDR                (AHB3_BASE_ADDR+0x1000)       /* HERE  0x1000 OFFSET OF GPIOA */

/*
 * peripheral register definition structure for GPIO
 */

typedef struct{

	__v	uint32_t MODER;            //GPIO port mode register                                       address offset  0x00
	__v	uint32_t OTYPER;           //GPIO port output type register                                address offset  0x04
	__v	uint32_t OSPEEDR;          //GPIO port output speed register                               address offset  0x08
	__v	uint32_t PUPDR;             //GPIO port pull-up/pull-down register                          address offset  0x0C
	__v	uint32_t IDR;              //GPIO port input data register                                 address offset  0x10
	__v	uint32_t ODR;              //GPIO port output data register                                address offset  0x14
	__v	uint32_t BSRR;             //GPIO port bit set/reset register                              address offset  0x18
	__v	uint32_t LCKR;             //GPIO port configuration lock register                         address offset  0x1C
	__v	uint32_t AFR[2];           //GPIO alternate function  register AFR[0] Low,  AFR[1] High    address offset  low=0x20, high= 0x24

}GPIO_RegDef_t;  				   // GPIO Register Defination Structure. here including all register related base address.



/*
 * Peripheral definition (Peripheral base addr typcusted to GPIO_RegDef_t
 */

#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)

#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)

#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)

#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

/*
 * Peripheral register structure for EXTI
 */

typedef struct
{

	__v  uint32_t IMR;            //Interrupt mask register                            address offset  0x00
	__v  uint32_t EMR;            //Event mask register                                address offset  0x04
	__v uint32_t RTSR;           //Rising trigger selection register                  address offset  0x08
	__v uint32_t FTSR;           //Falling trigger selection register                 address offset  0x0C
	__v  uint32_t SWIER;          //Software interrupt event register (EXTI_SWIER)     address offset  0x10
	__v  uint32_t PR;             //Pending register (EXTI_PR)                         address offset  0x14

}EXTI_RegDef_t;  				   // EXTI Register Defination Structure

#define  EXTI             ((EXTI_RegDef_t*)EXTI_BASE_ADDR)


/*
 * Peripheral register structure for SYSCFG
 * SYSCFG register maps
 */

typedef struct{

	__v  uint32_t MEMRMP;            //SYSCFG memory remap register                          address offset  0x00
	__v  uint32_t PMC;               //SYSCFG peripheral mode configuration register         address offset  0x04
	__v  uint32_t EXTICR[4];         //SYSCFG external interrupt configuration register      address offset  0x08-0x14
	__v  uint32_t CMPCR;             //Compensation cell control register                    address offset  0x20
	__v  uint32_t CFGR;              //SYSCFG configuration register                         address offset  0x2C

}SYSCFG_RegDef_t;  				     // SYSCFG Register Defination Structure

#define  SYSCFG            ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*
 * Peripheral register structure for SPI
 * SPI register maps
 */


typedef struct{

	__v	uint32_t CR1;            //SPI control register 1                          address offset  0x00
	__v	uint32_t CR2;            //GSPI control register 2                         address offset  0x04
	__v	uint32_t SR;             //SPI status register                             address offset  0x08
	__v	uint32_t DR;             //SPI data register                               address offset  0x0C
	__v	uint32_t CRCPR;          //SPI CRC polynomial register                     address offset  0x10
	__v	uint32_t RXCRCR;         //SPI RX CRC register                             address offset  0x14
	__v	uint32_t TXCRCR;         //SPI TX CRC register                             address offset  0x18
	__v	uint32_t I2SCFGR;        //SPI_I2 S configuration register                 address offset  0x1C
	__v	uint32_t I2SPR ;          //SPI_I2 S prescaler register                     address offset  0x20

}SPI_RegDef_t;


/*
 * Peripheral definition (Peripheral base addr typecusted to SPI_RegDef_t
 */

#define SPI1 ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_I2S2_BASE_ADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_I2S3_BASE_ADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASE_ADDR)



/*
 * Peripheral register structure for RCC
 */


typedef struct{
	__v uint32_t CR;                     /* RCC clock control register   Offset 0x00*/
	__v uint32_t PLLCFGR;                /* RCC PLL configuration register    Offset 0x04*/
	__v uint32_t CFGR;                   /* RCC clock configuration register    Offset 0x08*/
	__v uint32_t CIR;                    /* RCC clock interrupt register    Offset 0x0C*/
	__v uint32_t AHB1RSTR;               /* RCC AHB1 peripheral reset register    Offset 0x10*/
	__v uint32_t AHB2RSTR;               /* RCC AHB2 peripheral reset register    Offset 0x14*/
	__v uint32_t AHB3RSTR;               /* RCC AHB3 peripheral reset register    Offset 0x18*/

	uint32_t RESERVED0;              /* Offset 0x1C*/

	__v uint32_t APB1RSTR;               /* RCC APB1 peripheral reset register    Offset 0x20*/
	__v uint32_t APB2RSTR;               /* RCC APB2 peripheral reset register    Offset 0x24*/

	uint32_t RESERVED1[2]  ;         /* Offset     [0]  0x28*     [1] 0x2C */

	__v uint32_t AHB1ENR;                /* RCC AHB1 peripheral clock enable register    Offset 0x30*/
	__v uint32_t AHB2ENR;                /* RCC AHB2 peripheral clock enable register    Offset 0x34*/
	__v uint32_t AHB3ENR;                /* RCC AHB3 peripheral clock enable register    Offset 0x38*/

	uint32_t RESERVED2;              /* 0x3C */

	__v uint32_t APB1ENR;                /* RCC APB1 peripheral clock enable register    Offset 0x40*/
	__v uint32_t APB2ENR;                /* RCC APB2 peripheral clock enable register    Offset 0x44*/

	uint32_t RESERVED3[2];           /* [0] 0x48,   [1] 0x4C  */

	__v uint32_t AHB1LPENR;              /* RCC AHB1 peripheral clock enable in low power mode register   Offset 0x50*/
	__v uint32_t AHB2LPENR;              /* RCC AHB2 peripheral clock enable in low power mode register   Offset 0x54*/
	__v uint32_t AHB3LPENR;              /* RCC AHB3 peripheral clock enable in low power mode register   Offset 0x58*/

	__v uint32_t RESERVED4;              /* 0x5C */

	__v uint32_t APB1LPENR;              /* RCC APB1 peripheral clock enable in low power mode register   Offset 0x60*/
	__v uint32_t APB2LPENR;              /* RCC APB2 peripheral clock enabled in low power mode register   Offset 0x64*/

	uint32_t RESERVED5[2];           /*[0] 0x68,   [1] 0x6C  */

	__v uint32_t BDCR;                   /* RCC Backup domain control register   Offset 0x70*/
	__v uint32_t CSR;                    /* RCC clock control & status register    Offset 0x74*/

	uint32_t RESERVED6[2];           /* [0] 0x78,   [1] 0x7C  */

	__v uint32_t SSCGR;                  /* RCC spread spectrum clock generation register    Offset 0x80*/
	__v uint32_t PLLI2SCFGR;             /* RCC PLLI2S configuration register    Offset 0x84*/
	__v uint32_t PLLSAICFGR;             /* RCC PLL configuration register    Offset 0x88*/
	__v uint32_t DCKCFGR;                /* RCC dedicated clock configuration register    Offset 0x8C*/
	__v uint32_t CKGATENR;               /* RCC clocks gated enable register    Offset 0x90*/
	__v uint32_t DCKCFGR2;               /* RCC dedicated clocks configuration register 2    Offset 0x94*/

}RCC_Reg_Def_t;



/*
 * Peripheral definition (Peripheral base addr typecusted to RCC_Reg_Def_t
 */

#define RCC ((RCC_Reg_Def_t*)RCC_BASE_ADDR)

/*
 * Clock enable macros for GPIOx
 */


#define GPIOA_PCLK_EN()    (RCC-> AHB1ENR |= (1<<0))

#define GPIOB_PCLK_EN()    (RCC-> AHB1ENR |= (1<<1))

#define GPIOC_PCLK_EN()    (RCC-> AHB1ENR |= (1<<2))

#define GPIOD_PCLK_EN()    (RCC-> AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()    (RCC-> AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()    (RCC-> AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()    (RCC-> AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()    (RCC-> AHB1ENR |= (1<<7))


/*
 * Clock Disable macros for GPIOx
 */
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<0))

#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<1))

#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<2))

#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<3))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<4))
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<5))
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<6))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &=~ (1<<7))

/*
 *  MACROS TO RESET  GPIOx Peripheral
 */
#define GPIOA_REG_RESET() 	   do{(RCC-> AHB1RSTR |= (1<<0));    (RCC-> AHB1RSTR &=~ (1<<0)); }while(0)
#define GPIOB_REG_RESET() 	   do{(RCC-> AHB1RSTR |= (1<<1));    (RCC-> AHB1RSTR &=~ (1<<1)); }while(0)
#define GPIOC_REG_RESET()      do{(RCC-> AHB1RSTR |= (1<<2));    (RCC-> AHB1RSTR &=~ (1<<2)); }while(0)
#define GPIOD_REG_RESET()      do{(RCC-> AHB1RSTR |= (1<<3));    (RCC-> AHB1RSTR &=~ (1<<3)); }while(0)
#define GPIOE_REG_RESET()      do{(RCC-> AHB1RSTR |= (1<<4));    (RCC-> AHB1RSTR &=~ (1<<4)); }while(0)
#define GPIOF_REG_RESET()      do{(RCC-> AHB1RSTR |= (1<<5));    (RCC-> AHB1RSTR &=~ (1<<5)); }while(0)
#define GPIOG_REG_RESET()      do{(RCC-> AHB1RSTR |= (1<<6));    (RCC-> AHB1RSTR &=~ (1<<6)); }while(0)
#define GPIOH_REG_RESET()      do{(RCC-> AHB1RSTR |= (1<<7));    (RCC-> AHB1RSTR &=~ (1<<7)); }while(0)



/*
 *  for interupt
 */

#define GPIO_BASEADDR_TO_CODE(x)        ( (x == GPIOA)?0 :\
		                                  (x == GPIOB)?1 :\
			                           	  (x == GPIOC)?2 :\
						                  (x == GPIOD)?3 :\
								          (x == GPIOE)?4 :\
										  (x == GPIOF)?5 :\
										  (x == GPIOG)?6 :\
										  (x == GPIOH)?7 :0 )

/*
 * Clock enable macros for I2Cx
 */

#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= (1<<23))



/*
 * Clock Disable macros for I2Cx
 */
#define I2C1_PCLK_DI()    (RCC->APB1ENR &=~ (1<<21))
#define I2C2_PCLK_DI()    (RCC->APB1ENR &=~ (1<<22))
#define I2C3_PCLK_DI()    (RCC->APB1ENR &=~ (1<<23))




/*
 * Clock enable macros for SPIx
 */

#define SPI1_PCLK_EN()    (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()    (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()    (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()    (RCC->APB2ENR |= (1<<13))

/*
 * Clock enable macros for USARTx
 */

/*
 * Clock Disable macros for SPIx
 */
#define SPI1_PCLK_DI()    (RCC->APB2ENR &=~ (1<<12))
#define SPI2_PCLK_DI()    (RCC->APB1ENR &=~ (1<<14))
#define SPI3_PCLK_DI()    (RCC->APB1ENR &=~ (1<<15))
#define SPI4_PCLK_DI()    (RCC->APB2ENR &=~ (1<<13))


/*
 *  MACROS TO RESET  SPIx Peripheral
 */
#define SPI1_REG_RESET() 	   do{(RCC-> APB2ENR |= (1<<12));    (RCC-> APB2ENR &=~ (1<<12)); }while(0)
#define SPI2_REG_RESET() 	   do{(RCC-> APB1ENR |= (1<<14));    (RCC-> APB2ENR &=~ (1<<14)); }while(0)
#define SPI3_REG_RESET()       do{(RCC-> APB1ENR |= (1<<15));    (RCC-> APB1ENR &=~ (1<<15)); }while(0)
#define SPI4_REG_RESET()       do{(RCC-> APB2ENR |= (1<<13));    (RCC-> APB2ENR &=~ (1<<13)); }while(0)



/*
 * Clock enable macros for SYSCFG
 */

#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1<<14))  //RCC APB2 peripheral clock enable register (RCC_APB2ENR)

/*
 * Clock Disable macros for SYSCFG
 */

#define SYSCFG_PCLK_DI()    (RCC->APB2ENR &=~ (1<<14))// RCC APB2 peripheral reset register (RCC_APB2RSTR)

/*
 * Clock Disable macros for USARTx
 */


/*
 * External interrupt/event controller (EXTI)
 * Table 38. Vector table for STM32F446xx
 * This table same for all m4 family
 */

#define IRQ_NO_EXTI0       6
#define IRQ_NO_EXTI1       7
#define IRQ_NO_EXTI2       8
#define IRQ_NO_EXTI3       9
#define IRQ_NO_EXTI4       10
#define IRQ_NO_EXTI9_5     23
#define IRQ_NO_EXTI15_10   40
/*
 * MACROS for all the possible IRQ Priority level
 */
#define NVIC_IRQ_PRIO_0      0
#define NVIC_IRQ_PRIO_1	 	 1
#define NVIC_IRQ_PRIO_2 	 2
#define NVIC_IRQ_PRIO_3      3
#define NVIC_IRQ_PRIO_4      4
#define NVIC_IRQ_PRIO_5      5
#define NVIC_IRQ_PRIO_6      6
#define NVIC_IRQ_PRIO_7      7
#define NVIC_IRQ_PRIO_8      8
#define NVIC_IRQ_PRIO_9      9
#define NVIC_IRQ_PRIO_10    10
#define NVIC_IRQ_PRIO_11    11
#define NVIC_IRQ_PRIO_12    12
#define NVIC_IRQ_PRIO_13    13
#define NVIC_IRQ_PRIO_14    14
#define NVIC_IRQ_PRIO_15    15





/*
 * Some generic macros
 */


#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET 		SET

#define HIGH      		SET
#define LOW 			RESET

/***********************************************************************
 *  Bit Position Definition of SPI Peripheral
 ***********************************************************************
 */
//  SPI_CR1 SPI Control Register-1

#define SPI_CR1_CPHA  		 0		//Clock phase
#define SPI_CR1_CPOL  		 1		// Clock polarity
#define SPI_CR1_MSTR 		 2		//Master selection
#define SPI_CR1_BR   		 3		// Baud rate control
#define SPI_CR1_SPE  		 6		//SPI enable
#define SPI_CR1_LSB_FIRST    7		//Frame format
#define SPI_CR1_SSI          8		// Internal slave select
#define SPI_CR1_SSM          9		//Software slave management
#define SPI_CR1_RX_ONLY      10		//Receive only mode enable
#define SPI_CR1_DFF          11		//Data frame format
#define SPI_CR1_CRC_NEXT     12		//CRC transfer next
#define SPI_CR1_CRC_EN       13		//Hardware CRC calculation enable
#define SPI_CR1_BIDI_OE      14		//Output enable in bidirectional mode
#define SPI_CR1_BIDI_MODE    15		//Bidirectional data mode enable


//SPI_CR2  Control Register-2

#define SPI_CR2_RXDMAEN  		 0	//Rx buffer DMA enable
#define SPI_CR2_TXDMAEN  		 1	//Tx buffer DMA enable
#define SPI_CR2_SSOE 		     2	//Slave select output enable
#define SPI_CR2_FRF   			 4	//Frame format
#define SPI_CR2_ERRIE  			 5	//Error interrupt enable
#define SPI_CR2_RXNEIE 			 6	// RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE 			 7	//Tx buffer empty interrupt enable

// SPI_SR Status Register

#define SPI_SR_RXNE				 0	// Receive buffer not empty
#define SPI_SR_TXE  			 1	//Transmit buffer empty
#define SPI_SR_CHSIDE            2	//Channel side
#define SPI_SR_UDR               3	//Underrun flag
#define SPI_SR_CRC_ERR           4	//CRC error flag
#define SPI_SR_MODF              5	//Mode fault
#define SPI_SR_OVR               6	//Overrun flag
#define SPI_SR_BSY               7	// Busy flag
#define SPI_SR_FRE               8	// Frame Error






#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_SPI_driver.h"



#endif /* INC_STM32F446XX_H_ */
