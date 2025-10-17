
#ifndef TIMER_H_
#define TIMER_H_

#include"stm32f4xx.h"
#include<stdint.h>

void timer_init(void);
void set_rgb(uint16_t red, uint16_t green, uint16_t blue);

#endif /* TIMER_H_ */
