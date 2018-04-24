#include "STM32F4xx.h"
#include "RTE_Components.h"             // Component selection
#include "cmsis_armcc.h"
#include "stdbool.h"
#include "UART4.h"

void USART6init() {
	// Start trying to use USART6 (which can have the output pin on PC6, which is 
	// available on the header on pin 5 of J6.
	const int RCC_APB2RSTR = RCC_BASE + 0x24;     // Address of reset register
	const int USART6_BIT_IN_RCC = 5;    // USART6 controlled by bit 5
	
	// int test;
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN; // Enable the UART4 clock
	
	// First, reset the UART then bring it out of reset:
	*(unsigned int*)RCC_APB2RSTR |= 1UL << USART6_BIT_IN_RCC;
	*(unsigned int*)RCC_APB2RSTR &= ~(1UL << USART6_BIT_IN_RCC);
	
	// Set up the UART as a 31.25 kHz UART which can transmit only,
	// I'm not sure why this works (the division factor of 32 is empirical).
	// It might something like 16 MHz overclocked by x16 / 32 = 16e6 / 16 / 32 = 31.25 kbaud
	USART6->BRR = 32 << 4;
	USART6->CR1 = (1UL << 13) | (1UL << 3);
	
	// Turn clock to GPIOC on, and configure PC6 to take input from USART6
	// (note you have to turn the clock on before writing to the register):
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;         // Enable GPIOC clock	
	GPIOC->AFR[0] = (GPIOC->AFR[0] & 0xF0FFFFFF) | (0x8 << 24);

	int bit = 6;
	unsigned long bitMask = ~(3UL << 2*bit);
	GPIOC->MODER = (GPIOC->MODER & bitMask) | (2UL << 2*bit);  // Set mode to alternate function IO push-pull output
}

void USART6wait_until_ready() {
	int bReady = 0;
	do {
		bReady = USART6->SR & (1UL << 7);
	} while (!bReady);
}
	
void USART6send(uint8_t *bytes, int howMany) {
	// Sends howMany bytes, starting at the address pointed to by bytes.
	// Wait for the USART to be ready to accept another byte:
	int next = 0;
	
	while (next < howMany) {
		USART6wait_until_ready();
		uint8_t nextByte = bytes[next++];
		USART6->DR = nextByte;
	}
}