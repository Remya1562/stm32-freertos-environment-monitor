
#include"gpio.h"

#define GPIOAEN			(1U<<0)
#define SYSCFGEN		(1U<<14)


void button_exti_init(void){
	RCC->AHB1ENR |= GPIOAEN;

	GPIOA->MODER &=~ (3U<<(1*2));
	GPIOA->MODER |= (0U<<(1*2));

	GPIOA->PUPDR &= ~(3U << (1*2));
	GPIOA->PUPDR |=  (2U << (1*2));


	RCC->APB2ENR |= SYSCFGEN;
	EXTI->PR |= (1U<<1);
	SYSCFG->EXTICR[0] &=~ (0xF<<(1*4));
	EXTI->IMR |= (1U<<1);
	EXTI->RTSR |= (1U<<1);

	NVIC_EnableIRQ(EXTI1_IRQn);


}

void tim2_debounce_init(void) {
    RCC->APB1ENR |= (1U << 0);

    TIM2->PSC = 16000 - 1;

    TIM2->ARR = 20;

    TIM2->EGR |= (1U<<0);
    TIM2->SR &= ~(1U<<0);

    TIM2->DIER |= (1U << 0);

    NVIC_EnableIRQ(TIM2_IRQn);
}

