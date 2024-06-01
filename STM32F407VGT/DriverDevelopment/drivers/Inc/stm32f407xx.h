/*
 * stm32f407xx.h
 *
 *  Created on: May, 2024
 *      Author: omprakash barik
 */



#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stdint.h>


#define __vo            volatile

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED  4




#define FLASH_BASEADDR  0x08000000
#define SRAM1_BASEADDR  0x20000000
#define SRAM2_BASEADDR  0x2001C000
#define SRAM3_BASEADDR  0x20020000
#define ROM_BASEADDR    0x1FFF0000

#define APB1_BASEADDR   0x40000000
#define AHB1_BASEADDR   0x40020000
#define APB2_BASEADDR   0x40010000

//BASE ADDRESS FOR PERIPHERAL HANGING COLUMN OF AHB1

#define GPIOA_BASEADDR  0x40020000
#define GPIOB_BASEADDR  0x40020400
#define GPIOC_BASEADDR  0x40020800
#define GPIOD_BASEADDR  0x40020C00

#define RCC_BASEADDR    0x40023800

//BASE ADDRESS FOR PERIPHERAL HANGING COLUMN OF APB1

#define I2C1_BASEADDR   0x40005400
#define I2C2_BASEADDR   0x40005800
#define I2C3_BASEADDR   0x40005C00


#define UART5_BASEADDR  0x40005000
#define UART4_BASEADDR  0x40004C00

#define USART2_BASEADDR 0x40004400
#define USART3_BASEADDR 0x40004800



#define SPI2_BASEADDR   0x40003800
#define SPI3_BASEADDR   0x40003C00

//BASE ADDRESS FOR PERIPHERAL HANGING COLUMN OF APB2

#define USART1_BASEADDR 0x40011000
#define USART6_BASEADDR 0x40011400
#define SPI1_BASEADDR   0x40013000
#define SYSCFG_BASEADDR 0x40013800
#define EXTI_BASEADDR   0x40013C00

/*********peripheral register definition structures ***********/

typedef struct{
	__vo int32_t MODER;
	__vo int32_t OTYPER;
	__vo int32_t OSPEEDR ;
	__vo int32_t PUPDR;
	__vo int32_t IDR;
	__vo int32_t ODR;
	__vo int32_t BSRR;
	__vo int32_t LCKR;
	__vo int32_t AFR[2];


}GPIO_RegDef_t;

typedef struct{
	__vo int32_t  RCC_CR;
	__vo int32_t  RCC_PLLCFGR;
	__vo int32_t  RCC_CFGR;
	__vo int32_t  RCC_CIR;
	__vo int32_t  RCC_AHB1RSTR;
	__vo int32_t  RCC_AHB2RSTR;
	__vo int32_t  RCC_AHB3RSTR;
	__vo int32_t  RESERVED1;
	__vo int32_t  RCC_APB1RSTR;
	__vo int32_t  RCC_APB2RSTR;
	__vo int32_t  RESERVED2[2];
	__vo int32_t  RCC_AHB1ENR;
	__vo int32_t  RCC_AHB2ENR;
	__vo int32_t  RCC_AHB3ENR;
	__vo int32_t  RESERVED3;
	__vo int32_t  RCC_APB1ENR;
	__vo int32_t  RCC_APB2ENR;
	__vo int32_t  RESERVED4[2];
	__vo int32_t  RCC_AHB1LPENR;
	__vo int32_t  RCC_AHB2LPENR;
	__vo int32_t  RCC_AHB3LPENR;
	__vo int32_t  RESERVED5;
	__vo int32_t  RCC_APB1LPENR;
	__vo int32_t  RCC_APB2LPENR;
	__vo int32_t  RESERVED6[2];
	__vo int32_t  RCC_BDCR;
	__vo int32_t  RCC_CSR;
	__vo int32_t  RESERVED7[2];
	__vo int32_t  RCC_SSCGR;
	__vo int32_t  RCC_PLLI2SCFGR;
	__vo int32_t  RCC_PLLSAICFGR;
	__vo int32_t  RCC_DCKCFGR;



}RCC_RegDef_t;


typedef struct{
	__vo int32_t  EXTI_IMR;
	__vo int32_t  EXTI_EMR;
	__vo int32_t  EXTI_RTSR;
	__vo int32_t  EXTI_FTSR;
	__vo int32_t  EXTI_SWIER;
	__vo int32_t  EXTI_PR;
	}EXTI_RegDef_t;


typedef struct{
	__vo int32_t  SYSCFG_MEMRMP;
	__vo int32_t  SYSCFG_PMC;
	__vo int32_t  SYSCFG_EXTICR1;
	__vo int32_t  SYSCFG_EXTICR2;
	__vo int32_t  SYSCFG_EXTICR3;
	__vo int32_t  SYSCFG_EXTICR4;
	__vo int32_t  SYSCFG_CMPCR;
	}SYSCFG_RegDef_t;

typedef struct{
	__vo int32_t  SPI_CR1;
	__vo int32_t  SPI_CR2;
	__vo int32_t  SPI_SR;
	__vo int32_t  SPI_DR;
	__vo int32_t  SPI_CRCPR;
	__vo int32_t  SPI_RXCRCR;
	__vo int32_t  SPI_TXCRCR;
	__vo int32_t  SPI_I2SCFGR;
	__vo int32_t  SPI_I2SPR;
}SPI_RegDef_t;


typedef struct{
	__vo int32_t I2C_CR1;
	__vo int32_t I2C_CR2;
	__vo int32_t I2C_OAR1;
	__vo int32_t I2C_OAR2;
	__vo int32_t I2C_DR;
	__vo int32_t I2C_SR1;
	__vo int32_t I2C_SR2;
	__vo int32_t I2C_CCR;
	__vo int32_t I2C_TRISE;
	__vo int32_t I2C_FLTR;

}I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
} USART_RegDef_t;





#define ENABLE                 1
#define DISABLE                0
#define SET                    ENABLE
#define RESET                  DISABLE
#define GPIO_PIN_SET           SET
#define GPIO_PIN_RESET         RESET
#define FLAG_SET               SET
#define FLAG_RESET             RESET

#define GPIOA                      ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                      ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                      ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                      ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define RCC                        ((RCC_RegDef_t*)RCC_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)



#define GPIOA_REG_RESET()           do{ (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()           do{ (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()           do{ (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()           do{ (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3)); }while(0)

#define GPIOA_PCLK_EN()             RCC->RCC_AHB1ENR |= (1<<0)
#define GPIOB_PCLK_EN()             RCC->RCC_AHB1ENR |= (1<<1)
#define GPIOC_PCLK_EN()             RCC->RCC_AHB1ENR |= (1<<2)
#define GPIOD_PCLK_EN()             RCC->RCC_AHB1ENR |= (1<<3)


#define GPIOA_PCLK_DI()             RCC->RCC_AHB1ENR &= ~(1<<0)
#define GPIOB_PCLK_DI()             RCC->RCC_AHB1ENR &= ~(1<<1)
#define GPIOC_PCLK_DI()             RCC->RCC_AHB1ENR &= ~(1<<2)
#define GPIOD_PCLK_DI()             RCC->RCC_AHB1ENR &= ~(1<<3)


#define I2C1_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 23))

#define SPI1_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 13))

#define USART1_PCCK_EN() (RCC->RCC_APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->RCC_APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->RCC_APB1ENR |= (1 << 5))

#define SYSCFG_PCLK_EN() (RCC->RCC_APB2ENR |= (1 << 14))


#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/*********************************************************
 *Bit position definitions of USART peripheral
 *********************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#include "stm32f407xx_GPIO_driver.h"
#include "stm32f407xx_I2C_driver.h"
#include "stm32f407xx_RCC_driver.h"
#include "stm32f407xx_SPI_driver.h"
#include "stm32f407xx_USART_driver.h"




#endif /* INC_STM32F407XX_H_ */

