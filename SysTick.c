// SysTick.c
// Runs on LM4F120/TM4C123
// Provide functions that initialize the SysTick module, wait at least a
// designated number of clock cycles, and wait approximately a multiple
// of 10 milliseconds using busy wait.  After a power-on-reset, the
// LM4F120 gets its clock from the 16 MHz precision internal oscillator,
// which can vary by +/- 1% at room temperature and +/- 3% across all
// temperature ranges.  If you are using this module, you may need more
// precise timing, so it is assumed that you are using the PLL to set
// the system clock to 50 MHz.  This matters for the function
// SysTick_Wait10ms(), which will wait longer than 10 ms if the clock is
// slower.
// Daniel Valvano
// September 11, 2013
// Modified by Min He on 3/7/2024

/* This example accompanies the books
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1, Program 4.7
   
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Program 2.11, Section 2.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include "tm4c123gh6pm.h"
#include <stdint.h>

#define QUARTER_SEC          4000000     // number of machine cycles to generate 1us delay for 16MHz system clock
#define TEN_MS 500000  // reload value to generate 10ms for system clock 50MHz.

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC; // set SysTick timer to use core clock: 50MHz
}

// Time delay using busy wait.
// This assumes 50 MHz system clock.
// Input: 16-bit interger for multiple of 10ms
void SysTick_Wait10ms(uint16_t delay){	
	NVIC_ST_RELOAD_R = TEN_MS*delay-1;
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it                                        
  NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE; // enable SysTick timer
	
	// wait for COUNT bit in control register to be raised.
	while ((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0) {} 
  NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE; // disable SysTick timer
}


// Time delay using busy wait.
// This function assumes 16 MHz system clock.
void SysTick_WaitQuarterSEC(){
  NVIC_ST_CTRL_R = 0;
  NVIC_ST_RELOAD_R = QUARTER_SEC-1; // number of counts to wait
  NVIC_ST_CURRENT_R = 0; // any value written to CURRENT clears
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
  while((NVIC_ST_CTRL_R&NVIC_ST_CTRL_COUNT)==0); // wait for count flag
  NVIC_ST_CTRL_R = 0;
}
