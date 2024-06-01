//#include <stdint.h>
//#include <stm32f407xx.h>
//#include <stm32f407xx_GPIO_driver.h>
//
//void delay(){
//	for(int i=0; i<500000; i++){
//
//	}
//}
//
//
//int main(void){
//	GPIO_Handle_t LED1, LED2, LED3, LED4;
//	LED1.pGPIOx= GPIOA;
//	LED1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	LED1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	LED1.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
//	LED1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//	LED1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//	LED2.pGPIOx= GPIOB;
//		LED2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
//		LED2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//		LED2.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
//		LED2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//		LED2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//		LED3.pGPIOx= GPIOC;
//			LED3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//			LED3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//			LED3.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
//			LED3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//			LED3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//			LED4.pGPIOx= GPIOD;
//				LED4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
//				LED4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//				LED4.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
//				LED4.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//				LED4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//
//
//				GPIO_Init(&LED1);GPIO_Init(&LED2);GPIO_Init(&LED3);GPIO_Init(&LED4);
//
//				while(1){
//					GPIO_WriteToOutputPin(GPIOA, 12, 1);
//					delay();
//					GPIO_WriteToOutputPin(GPIOB, 13, 1);
//					delay();
//					GPIO_WriteToOutputPin(GPIOC, 14, 1);
//					delay();
//					GPIO_WriteToOutputPin(GPIOD, 14, 1);
//					delay();
//
//				}
//
//
//
//
//}
