#include"stm32f4xx.h"
#include"timer.h"


#define GPIOBEN						(1U<<1)
#define TIM2EN						(1U<<0)
#define TIM3EN						(1U<<1)
#define TIM_CR1_EN					(1U<<0)
#define TIM2_CC2E  	 				(1U<<4)
#define TIM2_CC3E   				(1U<<8)
#define TIM3_CC1E  					(1U<<0)

void timer_init(void){
	RCC->AHB1ENR |= GPIOBEN;

	GPIOB->MODER &=~ (3U<<6) | (3U<<8) | (3U<<20);
	GPIOB->MODER |= (2U<<6) | (2U<<8) | (2U<<20);

	GPIOB->AFR[0] &=~ (0xF<<12);
	GPIOB->AFR[0] |= (0x1<<12);

	GPIOB->AFR[0] &=~ (0xF<<16);
	GPIOB->AFR[0] |=(0x2<<16);

	GPIOB->AFR[1] &=~ (0xF<<8);
	GPIOB->AFR[1] |= (0x1<<8);

	RCC->APB1ENR |= TIM2EN;
	RCC->APB1ENR |= TIM3EN;

	// TIM2 setup for CH2 (PB3), CH3 (PB10)
	TIM2->PSC = 84-1;
	TIM2->ARR = 1000-1;
	// CH2 PWM mode 1
	TIM2->CCMR1 |= (1U<<14);
	TIM2->CCMR1 |= (1U<<13);
	TIM2->CCMR1 &=~ (1U<<12);
	// CH3 PWM mode 1
	TIM2->CCMR2 |= (1U<<6);
	TIM2->CCMR2 |= (1U<<5);
	TIM2->CCMR2 &=~ (1U<<4);

	TIM2->CCER |= TIM2_CC2E | TIM2_CC3E;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CR1 |= TIM_CR1_EN;

	// TIM3 setup for CH1 (PB4)
	TIM3->PSC = 84-1;
	TIM3->ARR = 1000-1;
	// CH3 PWM mode 1
	TIM3->CCMR1 |= (1U<<6);
	TIM3->CCMR1 |= (1U<<5);
	TIM3->CCMR1 &=~ (1U<<4);

	TIM3->CCER |= TIM3_CC1E;
	TIM3->CCR1 = 0;
	TIM3->CR1 |= TIM_CR1_EN;

}
void set_rgb(uint16_t red, uint16_t green, uint16_t blue){
	TIM2->CCR2 = red;
	TIM3->CCR1 = green;
	TIM2->CCR3 = blue;

}
