#include "stm32f4xx.h"
#include "i2c.h"

#define GPIOBEN      (1U<<1)
#define I2C1EN       (1U<<21)
#define CR1_PE        (1U<<0)
#define CR1_START     (1U<<8)
#define CR1_STOP      (1U<<9)
#define CR1_ACK       (1U<<10)

#define SR1_SB        (1U<<0)
#define SR1_ADDR      (1U<<1)
#define SR1_TXE       (1U<<7)
#define SR1_RXNE      (1U<<6)
#define SR1_BTF       (1U<<2)
#define SR2_BUSY      (1U<<1)

#define I2C_100KHZ    80
#define TRISE_STD     17

void i2c1_init(void) {
    RCC->AHB1ENR |= GPIOBEN;

    GPIOB->MODER &= ~((3U<<16) | (3U<<18));
    GPIOB->MODER |=  ((2U<<16) | (2U<<18));

    GPIOB->AFR[1] &= ~((0xF<<0) | (0xF<<4));
    GPIOB->AFR[1] |= ((4<<0) | (4<<4));

    GPIOB->OTYPER |= (1U<<8) | (1U<<9);

    GPIOB->PUPDR &= ~((3U<<16) | (3U<<18));
    GPIOB->PUPDR |= ((1U<<16) | (1U<<18));

    RCC->APB1ENR |= I2C1EN;

    I2C1->CR1 &= ~CR1_PE;
    I2C1->CR2 = 16;
    I2C1->CCR = I2C_100KHZ;
    I2C1->TRISE = TRISE_STD;
    I2C1->CR1 |= CR1_PE;
}

int i2c1_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    volatile int temp;
    int timeout;

    timeout = 100000;
    while((I2C1->SR2 & SR2_BUSY) && timeout--) {}
    if(timeout == 0) return -1;

    I2C1->CR1 |= CR1_START;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_SB) && timeout--) {}
    if(timeout == 0) return -1;

    I2C1->DR = dev_addr<<1;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_ADDR) && timeout--) {}
    temp = I2C1->SR2;

    I2C1->DR = reg_addr;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_TXE) && timeout--) {}
    if(timeout == 0) return -1;

    for(uint8_t i=0; i<len; i++){
        I2C1->DR = data[i];
        timeout = 10000;
        while(!(I2C1->SR1 & SR1_TXE) && timeout--) {}
        if(timeout == 0) return -1;
    }

    timeout = 10000;
    while(!(I2C1->SR1 & SR1_BTF) && timeout--) {}
    I2C1->CR1 |= CR1_STOP;

    return 0;
}

int i2c1_burstread(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    volatile int temp;
    int timeout;

    timeout = 100000;
    while((I2C1->SR2 & SR2_BUSY) && timeout--) {}
    if(timeout == 0) return -1;

    I2C1->CR1 |= CR1_START;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_SB) && timeout--) {}
    if(timeout == 0) return -1;

    I2C1->DR = dev_addr<<1;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_ADDR) && timeout--) {}
    temp = I2C1->SR2;

    I2C1->DR = reg_addr;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_TXE) && timeout--) {}
    if(timeout == 0) return -1;

    I2C1->CR1 |= CR1_START;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_SB) && timeout--) {}
    if(timeout == 0) return -1;

    I2C1->DR = (dev_addr<<1)|1;
    timeout = 10000;
    while(!(I2C1->SR1 & SR1_ADDR) && timeout--) {}
    temp = I2C1->SR2;

    for(uint8_t i=0; i<len; i++){
        if(i == len-1){
            I2C1->CR1 &= ~CR1_ACK;
            I2C1->CR1 |= CR1_STOP;
        }
        timeout = 10000;
        while(!(I2C1->SR1 & SR1_RXNE) && timeout--) {}
        if(timeout == 0) return -1;
        data[i] = I2C1->DR;
    }
    I2C1->CR1 |= CR1_ACK;

    return 0;
}
