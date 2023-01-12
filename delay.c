#include "delay.h"
 
static volatile uint32_t ticks;
 
void delayInit() 
{ 
  SystemCoreClockUpdate(); 
	SysTick_Config(SystemCoreClock / 1000000); // Overflow every 1 us 
	//System Tick Timer Configuration.
  //Initialises and starts the System Tick Timer and its interrupt. 
	//After this call, the SysTick timer creates interrupts with the specified time interval. 
	//Counter is in free running mode to generate periodical interrupts.
} 

void delayDecrement(void)
{
	if (ticks != 0x00) 
 		ticks--;
}

void delayUs(uint32_t nTime) 
{ 
 	ticks = nTime; 
 	while(ticks); 
}
 
void delayMs(uint32_t nTime) 
{ 
	while(nTime--)  
 		delayUs(1000); 
} 
