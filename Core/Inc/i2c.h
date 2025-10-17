
#ifndef I2C_H_
#define I2C_H_

#include"stm32f4xx.h"
#include<stdint.h>


void i2c1_init(void);
int i2c1_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
int i2c1_burstread(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

#endif /* I2C_H_ */
