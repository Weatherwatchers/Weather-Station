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
#include <math.h>
#include "STM32F4xx.h"
#include "LED.h"
#include "SWT.h"
#include "LCD.h"
#include "UART4.h"



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



void ADC2_init(void) {
	/* Enable clocks */
	RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;
	
	/* ADC12_IN14 is the channel we shall use. It is connected to 
	 * PC4 which is connected to the board edge connectors */
	GPIOC->MODER = 0x3 << (2 * 5);
	GPIOC->PUPDR = 0;
	
	/* Set ADC to discontinuous conversion mode. */
	ADC2->CR1 = 0x00;
	ADC2->CR1 |= ADC_CR1_DISCEN;
	
	/* Ensure CR2 is set to zero. This means data will be right aligned, 
	 * DMA is disabled and there are no extrnal triggers/injected channels */
	ADC2->CR2 = 0x00;
	
	/* Set to one conversion at a time, and set that first conversion 
	 * to come from channel 14 (connected to PC4) */
	ADC2->SQR1 &= ~ADC_SQR1_L;
	ADC2->SQR3 = 15 & ADC_SQR3_SQ1;
	
	/* Enable the ADC */
	ADC2->CR2 |= ADC_CR2_ADON;
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
unsigned int read_ADC2 (void) {
	/* set SWSTART to 1 to start conversion */
	ADC2->CR2 |= ADC_CR2_SWSTART;
	
	/* Wait until End Of Conversion bit goes high */
	while (!(ADC2->SR & ADC_SR_EOC));
	
	/* Return data value */
	return (ADC2->DR);
}

void PC13_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER &= ~((3UL << 2*13) );
	GPIOC->PUPDR &= ~((3UL<<2*13));
	GPIOC->PUPDR |= ((2UL<<2*13));
}
/*
void PE3_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER &= ~((3UL << 2*3) );
	GPIOE->MODER |= ~((1UL << 2*3) );
	GPIOC->OTYPER &= ~((1UL<<6));
}

void output_PE3_on(void){
	GPIOE->BSRR = 1UL << 3;
}

void output_PE3_off(void){
	GPIOE->BSRR = 1UL << 3 << 16;
}*/

uint32_t Val_Get_PC13(void) {
	return (GPIOC->IDR & (1UL<<13));
}


double findSpeed(void)
{
	double count=0;
	int flag=0;

	uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < 1000){
		if(Val_Get_PC13()!=0 && flag==0){
			count=count+1;
			flag=1;
		}
		else if(Val_Get_PC13()==0){
			flag=0;
	}
	}
	if(flag==1 && count==1){
		count = 0;
	}
	return count * 2.4;
}

int findWindDirection(){
	double value;
	char display_value[20];
	
	double voltageLookUp[8] = {2.29,  1.35, 0.27, 0.53,  0.84, 1.84,  2.76, 2.59};   /*Values callibrated to a certain weather vane, may need to change them in order to be more accurate*/
	
	/* Gets a 12 bit right-aligned value from the ADC */
		value = read_ADC1(); 
		
		value=value*0.00073; /*multiplied by 0.00073 as range is 3V and has resolution 12bit therefor 4096 levels. 3V/4096=0.00073*/
		sprintf(display_value,"%f", value); /*value needs to changed to a char to be displayed on the lcd*/
		
		int i;
		int index;
		double closeness=500; /*find closest compass position of the weather vane*/
		for(i=0;i<=7;i++)
		{
			if(closeness>fabs(value-voltageLookUp[i]))
			{
				closeness=fabs(value-voltageLookUp[i]);
				index=i;
			}
			else
			{
				closeness=closeness;
			}
		}
		return index;
}

double findTemp(){
	return read_ADC2();
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

	volatile uint32_t btns = 0;
	volatile int switch1;
	volatile int switch2;
	volatile int switch3;
	int page;
	char* compassPoints[2][8]={"S",  "SW",  "W",  "NW", "N",  "NE",  "E",  "SE"  };
	
	//clock_t start;
	//clock_t end;
	int speedcount;
	double cyclesPerSec;
	char display_speed[20];
	char display_temp[20];
	double speed = 0;
	int flag = 0;
	

	/* Midi CC = 0xnc, 0xcc, 0xvv
	Where n = is the status (0xB only!)
	c = midi channel no.
	cc = controller number
	vv = controller value */
	uint8_t winddirection_midi_values[]={0xB1, 0x01, 0x00};
	uint8_t windspeed_midi_values[]={0xB2, 0x02, 0x00};
	
	
  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }

  LED_Init();
  BTN_Init();   
  SWT_Init();	
	ADC1_init();		/*Initialise everything*/
	//ADC2_init();	
  LCD_Initpins();	
	LCD_DriverOn();
	USART6init(); /*MIDI Initialisation*/
	
	Delay(10);
	LCD_Init();
	LCD_On(1);
	PC13_Init();
	
  LCD_PutS("Hello");
	
	int continueRunning = 1;
	
	while(continueRunning){  /* Loop forever */
		/* Read switch states  */
	  btns = GPIOE->IDR;

		/*FIND WIND DIRECTION*/
		int index = findWindDirection();
		winddirection_midi_values[2] = index;
		
		/*Send data as Midi*/
		USART6send(winddirection_midi_values, 3);
		
		/*FIND WIND SPEED*/
		double speed = findSpeed();
		windspeed_midi_values[2] = speed;
		/*Send data as Midi*/
		USART6send(windspeed_midi_values, 3);
		
		/*FIND TEMP*/
		//double temp = findTemp();
	
		/*check Switches*/
		switch1=SWT_Check(0);
		switch2=SWT_Check(1);
		switch3=SWT_Check(2);
		
		/*Latch system so you only need to press the desired button once, and it will change "page"*/
		if (switch1!=0){
			page=1;
		}
		
		if (switch2!=0){
			page=2;
		}
		
		if (switch3!=0){
			page=3;
		}
	
		/*PAGE 1*/
		if(page==1)
		{
			LCD_Clear();
			LCD_GotoXY(0,0);
			LCD_PutS("Wind Direction");
			LCD_GotoXY(0,1);
			 
			LCD_GotoXY(10,1);
			
			LCD_PutS(compassPoints[0][index]); /*allong with compass position (N, NW, S, SE etc)*/
			
			Delay(1000);
		}
		
		/*PAGE 2*/
		if(page==2)
		{
			LCD_Clear();
			LCD_GotoXY(0,0);
			LCD_PutS("WindSpeed (kmph)");
			
			LCD_GotoXY(10,1);
			LCD_PutS(display_speed);
			
			sprintf(display_speed, "%f", speed);
			LCD_PutS(display_speed);
		
			Delay(100);
		}
		
		/*PAGE 3*/
		if(page==3)
		{
			LCD_Clear();
			LCD_PutS("Page 3");
			Delay(1000);
			
			//sprintf(display_temp, "%f", temp);
			
			LCD_PutS(display_temp);
		}
	}	
}	


