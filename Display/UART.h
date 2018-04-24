/*----------------------------------------------------------------------------
 * Name:    UART4.h
 * Purpose: low level UART4 definition for MIDI transmitter
 * Note(s):
 *----------------------------------------------------------------------------
 * Cobbled together by DAJP, Jan 2018
 *----------------------------------------------------------------------------*/

#ifndef __UART4_H
#define __UART4_H

// UART4 External Function Definitions:
extern void USART6init(void);
extern void USART6send(uint8_t *bytes, int howMany);

#endif
