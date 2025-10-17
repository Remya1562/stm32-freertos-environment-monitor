#include"adc.h"
#include "stm32f4xx.h"

#define ADC1EN          (1U<<8)
#define GPIOAEN         (1U<<0)
#define CR2_ADON        (1U<<0)
#define CR2_SWSTART     (1U<<30)
#define SR_EOC          (1U<<1)

void adc1_init(void){
    RCC->AHB1ENR |= GPIOAEN;
    RCC->APB2ENR |= ADC1EN;

    GPIOA->MODER &= ~(3U << (0*2));
    GPIOA->MODER |=  (3U << (0*2));

    ADC1->CR1 = 0;
    ADC1->CR2 = CR2_ADON;

    ADC1->SMPR2 &= ~(7U << (0*3));
    ADC1->SMPR2 |=  (7U << (0*3));

    ADC1->SQR3 = 0;

    ADC1->CR2 |= CR2_SWSTART;
    while (!(ADC1->SR & SR_EOC));
    (void)ADC1->DR;
    ADC1->SR &= ~SR_EOC;
}

uint32_t adc1_read(void){
    ADC1->CR2 |= CR2_SWSTART;
    while (!(ADC1->SR & SR_EOC));
    uint32_t val = ADC1->DR & 0xFFF;
    ADC1->SR &= ~SR_EOC;
    return val;
}

