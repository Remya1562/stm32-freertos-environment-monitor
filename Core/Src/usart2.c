
#include"usart2.h"
#define GPIOAEN			(1U<<0)
#define UART2EN			(1U<<17)

#define CR1_TE			(1U<<3)
#define CR1_RE			(1U<<2)

#define CR1_UE			(1U<<13)
#define SR_TXE			(1U<<7)
#define SR_RXNE			(1U<<5)

#define SYS_FREQ		16000000
#define APB1_CLK		SYS_FREQ

#define UART_BAUDRATE	115200


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk,uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

void uart_tx_init(void);
void uart2_write(int ch);

int __io_putchar(int ch){
	uart2_write(ch);
	return ch;
}


void uart_tx_init(void){
	RCC->AHB1ENR  |=  GPIOAEN;

	GPIOA->MODER  &=~ (1U<<4);
	GPIOA->MODER  |=  (1U<<5);

	GPIOA->AFR[0] &=~ (0xF<<8);
	GPIOA->AFR[0] |= (0x7<<8);

	RCC->APB1ENR   |=  UART2EN;

	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	USART2->CR1  =  CR1_TE;
	USART2->CR1 |=  CR1_UE;

}



static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk,uint32_t BaudRate){
	USARTx->BRR=compute_uart_bd(PeriphClk, BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate){
	return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
}


void uart2_write(int ch){
	while(!(USART2->SR & SR_TXE)){}
	USART2->DR = (ch & 0xff);
}



