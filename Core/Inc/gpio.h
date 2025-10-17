
#ifndef GPIO_H_
#define GPIO_H_

#include"stm32f4xx.h"
#define BTN_PIN			(1U<<1)

void button_exti_init(void);
void tim2_debounce_init(void);


#endif /* GPIO_H_ */
