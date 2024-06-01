/*
 * stm32f407xx_GPIO_driver.c
 *
 *  Created on: May, 2024
 *      Author: omprakash barik
 */

#include<stm32f407xx.h>
#include<stm32f407xx_GPIO_driver.h>

void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx,int8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
     	}
	}else{
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_DI();
		}

	}
}
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	int32_t temp=0;
	GPIO_PCLKControl(pGPIOHandle->pGPIOx,ENABLE);
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		(pGPIOHandle->pGPIOx->MODER&=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); //clear
		(pGPIOHandle->pGPIOx->MODER|=temp);
	}
	else{};

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2));

	}


	temp=0;
	    temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	    (pGPIOHandle->pGPIOx->OSPEEDR&= ~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		(pGPIOHandle->pGPIOx->OSPEEDR|=temp);
	temp=0;
	    temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	    (pGPIOHandle->pGPIOx->PUPDR&=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		(pGPIOHandle->pGPIOx->PUPDR|=temp);
	temp=0;
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		(pGPIOHandle->pGPIOx->OTYPER&= ~(0x1<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	    (pGPIOHandle->pGPIOx->OTYPER|=temp);





}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	                if(pGPIOx == GPIOA)
					{
		                GPIOA_REG_RESET();
					}else if (pGPIOx == GPIOB)
					{
						GPIOB_REG_RESET();
					}else if (pGPIOx == GPIOC)
					{
						 GPIOC_REG_RESET();
					}else if (pGPIOx == GPIOD)
					{
						 GPIOD_REG_RESET();
					}


}
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	    uint8_t value;
		value = (uint8_t)((pGPIOx->IDR >>PinNumber) & 0x00000001);
		return value;


}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}
void GPIO_WriteToOutputPin(GPIO_RegDef_t  *pGPIOx,uint8_t PinNumber,uint8_t value)
{
	if (value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t  *pGPIOx,uint16_t value)
{
	pGPIOx->ODR = value;

}

void GPIO_Toggle_OutputPin(GPIO_RegDef_t  *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);

}



