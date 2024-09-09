// SysTickTestMain.c
// Runs on LM4F120/TM4C123
// Test the SysTick functions by activating the PLL, initializing the
// SysTick timer, and flashing an LED at a constant rate.
// Daniel Valvano
// September 12, 2013
// Modified by Min He on 3/7/2024
/* 
	CECS 347 - Lab 1 part 2 Timer 0A intigration
	Objective: 
	Test example project GPTM_Timer0A on your Launchpad, change system clock to 16MHz.

	Modify part 1: Change code related to Systick timer the GPTM to use Timer1 16-bit 
	counting and find the right prescale value for generating 0.25s delay, test it with 
	simulator logic analyzer, then download it to your launchpad to test on board.
	Authors: Roy Mears, Jonathan Ramirez, Nolan Nguyen, Christian Mata
	Professor: Min He 
	09/09/2024
*/
// Documentation Section
// SysTickPLLTest.c
// Runs on TM4C123
// Starter file for Lab 1
// Authors: Roy Mears, Jonathan Ramirez, Nolan Nguyen, Christian Mata
// 09/09/2024
// Port F pin 2 is connected to the onboard red led.


#include "tm4c123gh6pm.h"
#include "Timer0A.h"
#include "PLL.h"

//#define RED_LED       (*((volatile unsigned long *)0x40025008))
//#define RED_LED_MASK    0x02  // bit pisition for onboard red LED
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode
void PORTF_Init(void);
void Portb_Init(void);
/*
void UserTask(void){
  RED_LED ^= RED_LED_MASK;// toggle PF2: RED LED
}

int main(void){
	PORTF_Init();							// PF2(RED LED) is an output for debugging
  PLL_Init();               // set system clock to 16 MHz
  //SysTick_Init();           // initialize SysTick timer
	//Timer1A_Init(&UserTask, 40000);// initialize timer1 (2 Hz)

  while(1){
    WaitForInterrupt();

  }
}

*/

// Ultrasonic_Sensor.c
// This program runs on TM4C123.
// This is an example program to show how to interface HC-SR04 Ultrasonic sensor.
// PB6 connects to echo pin to generate edge-triggered interrupt.
// PB7 connects to Ultrasonic sensor trigger pin.
// SysTick timer is used to generate the required timing for trigger pin and measure echo pulse width.
// Global variable "distance" is used to hold the distance in cemtimeter for the obstacles
// in front of the sensor. 
// By Dr. Min He
// December 10th, 2018

#include <stdint.h>
//#include "SysTick.h"
//#include "tm4c123gh6pm.h"

#define TRIGGER_PIN 		(*((volatile unsigned long *)0x40005200))  // PB7 is the trigger pin	
#define TRIGGER_VALUE 	0x80   // trigger at bit 7
#define ECHO_PIN 				(*((volatile unsigned long *)0x40005100))  // PB6 is the echo pin	
#define ECHO_VALUE 			0x40   // trigger at bit 6
#define MC_LEN 					0.0625 // length of one machine cycle in microsecond for 16MHz clock
#define SOUND_SPEED 		0.0343 // centimeter per micro-second

extern void EnableInterrupts(void);
extern void GPIOPortB_Handler(void);
void PortB_Init(void);
void Delay(void);

static volatile uint8_t done=0;
static volatile uint32_t distance=0;

int main(void){
	PortB_Init();
  EnableInterrupts();
  while(1){
		done = 0;
		distance = 0;
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		//SysTick_Wait1us(2);                                      // use timer to wait 2us
		TRIGGER_PIN |= TRIGGER_VALUE; // send high to trigger
		//SysTick_Wait1us(10);																		// use timer to wait 10us
		TRIGGER_PIN &= ~TRIGGER_VALUE; // send low to trigger
		
    while (!done);
  }
}

void PortB_Init(void){ 
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;           // 1) activate clock for Port A
	while ((SYSCTL_RCGC2_R&SYSCTL_RCGC2_GPIOB)!=SYSCTL_RCGC2_GPIOB){}; // wait for clock to start
                                    // 2) no need to unlock PA2
  GPIO_PORTB_PCTL_R &= ~0xFF000000; // 3) regular GPIO
  GPIO_PORTB_AMSEL_R &= (uint32_t)~0xC0;      // 4) disable analog function on PA2
  GPIO_PORTB_DIR_R &= ~0x40;        // 5) PB6:echo pin, input
  GPIO_PORTB_DIR_R |= 0x80;         // 5) PB7:trigger pin, output
  GPIO_PORTB_AFSEL_R &= ~0xC0;      // 6) regular port function
  GPIO_PORTB_DEN_R |= 0xC0;         // 7) enable digital port
  GPIO_PORTB_IS_R &= ~0x40;         // PB6 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x40;         // PB6 is both edges
  GPIO_PORTB_IEV_R &= ~0x40;        // PB6 both edge event
  GPIO_PORTB_ICR_R = 0x40;          // clear flag 6
  GPIO_PORTB_IM_R |= 0x40;          // arm interrupt on PB6
  NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF1FFF)|0x00006000; // (g) priority 3
  NVIC_EN0_R = 0x00000002;          // (h) enable Port B edge interrupt
}

void GPIOPortB_Handler(void){
	
	if (ECHO_PIN==ECHO_VALUE){  // echo pin rising edge is detected, start timing
		//SysTick_Start();
		// timer one starts in oneshot mode
	}
	else { // echo pin falling edge is detected, end timing and calculate distance.
    // The following code is based on the fact that the HCSR04 ultrasonic sensor 
    // echo pin will always go low after a trigger with bouncing back
    // or after a timeout. The maximum distance can be detected is 400cm.
		// The speed of sound is approximately 340 meters per second, 
		// or  .0343 c/µS.
    // Distance = (echo pulse width * 0.0343)/2; = ((# of mc)*MC_LEN*SOUND_SPEED)/2
		
		//SysTick_Stop();
		// stop timer one
		// store timer one countdown value
		
		distance = (uint32_t)(TIMER1_TAR_R*MC_LEN*SOUND_SPEED)/2;		
		done = 1;
	}
  GPIO_PORTB_ICR_R = 0x40;      // acknowledge flag 6
	//reset timer 1 to initial
}


















/*
void PORTF_Init(void)
{
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate port F
	while ((SYSCTL_RCGCGPIO_R&SYSCTL_RCGCGPIO_R5)!=SYSCTL_RCGCGPIO_R5){};
	
	GPIO_PORTF_DIR_R |= RED_LED_MASK; // make PF2 out (built-in RED LED)
  GPIO_PORTF_AFSEL_R &= ~RED_LED_MASK;// disable alt funct on PF2
  GPIO_PORTF_DEN_R |= RED_LED_MASK; // enable digital I/O on PF2                            
  GPIO_PORTF_PCTL_R &= ~0x000000F0; // configure PF2 as GPIO
  GPIO_PORTF_AMSEL_R |= RED_LED_MASK;   // disable analog functionality on PF  
}
*/
