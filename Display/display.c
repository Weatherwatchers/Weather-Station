/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: MOdification to Drive 2x16 LCD
 * Note(s): 
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/
 
 /* MODIFIED BY D. CHESMORE JANUARY 2013   */
 
#include <stdio.h>
#include <stdlib.h>
#include "STM32F4xx.h"
#include "LED.h"
#include "SWT.h"
#include "LCD.h"



volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


/*----------------------------------------------------------------------------
  Function that initializes Button pins
 *----------------------------------------------------------------------------*/
void BTN_Init(void) {

  RCC->AHB1ENR  |= ((1UL <<  0) );              /* Enable GPIOA clock         */

  GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */
  GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */
  GPIOA->OSPEEDR  |=  ((2UL << 2*0)  ); 
  GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */
}

/*----------------------------------------------------------------------------
  Function that read Button pins
 *----------------------------------------------------------------------------*/
 uint32_t BTN_Get(void) {

 return (GPIOA->IDR & (1UL<<0));
}



/* Function to intiialise ADC1    */

void ADC1_init(void) {
	/* Enable clocks */
	RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;
	
	/* ADC12_IN14 is the channel we shall use. It is connected to 
	 * PC4 which is connected to the board edge connectors */
	GPIOC->MODER = 0x3 << (2 * 4);
	GPIOC->PUPDR = 0;
	
	/* Set ADC to discontinuous conversion mode. */
	ADC1->CR1 = 0x00;
	ADC1->CR1 |= ADC_CR1_DISCEN;
	
	/* Ensure CR2 is set to zero. This means data will be right aligned, 
	 * DMA is disabled and there are no extrnal triggers/injected channels */
	ADC1->CR2 = 0x00;
	
	/* Set to one conversion at a time, and set that first conversion 
	 * to come from channel 14 (connected to PC4) */
	ADC1->SQR1 &= ~ADC_SQR1_L;
	ADC1->SQR3 = 14 & ADC_SQR3_SQ1;
	
	/* Enable the ADC */
	ADC1->CR2 |= ADC_CR2_ADON;
}
	
/* function to read ADC and retun value */
unsigned int read_ADC1 (void) {
	/* set SWSTART to 1 to start conversion */
	ADC1->CR2 |= ADC_CR2_SWSTART;
	
	/* Wait until End Of Conversion bit goes high */
	while (!(ADC1->SR & ADC_SR_EOC));
	
	/* Return data value */
	return (ADC1->DR);
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

	uint32_t btns = 0;
	int switch1;
	int switch2;
	int switch3;
	int switch4;
	float value;
	char display_value[20];
	

  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }

  LED_Init();
  BTN_Init();   
  SWT_Init();	
	ADC1_init();
  LCD_Initpins();	
	LCD_DriverOn();
	
	Delay(10);
	LCD_Init();
	LCD_On(1);
	
	while(1) {                                    /* Loop forever               */
    btns = SWT_Get();		/* Read switch states         */
		GPIOD->ODR = btns;
		
		value = read_ADC1(); 												/* Gets a 12 bit right-aligned value from the ADC */
		
		value=value*0.00073;
		sprintf(display_value,"%f", value);
			
		/*check Switches*/
		switch1=SWT_Check(0);
		switch2=SWT_Check(1);
		switch3=SWT_Check(2);
		switch4=SWT_Check(3);
		
		if (switch1!=0){
			LCD_Clear();
			LCD_PutS("Page 1");
			LCD_GotoXY(5,1);
			
			LCD_PutS(display_value);
		}
		
		if (switch2!=0){
			LCD_Clear();
			LCD_PutS("Page 2");
		}
		
		if (switch3!=0){
			LCD_Clear();
			LCD_PutS("Page 3");
		}
		
		if (switch4!=0){
			LCD_Clear();
			LCD_PutS("Page 4");
		}
	}

	LCD_DriverOn();
	LCD_On(1);
	Delay(20);
	LCD_Clear();
	LCD_PutS("Hello World!");
	
	LCD_GotoXY(4,1);
	LCD_PutS("Des & Const");
	
				
}
