/* 

  triangle output at DAC1 (STM32L072  Project)

  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.


*/

#include "stm32l0xx.h"
#include "sysclk.h"
#include "delay.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS_8;		/* atomic set PA8 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		/* atomic clr PA8 */
}



/*
  Linear Congruential Generator (LCG)
  z = (a*z + c) % m;  
  m = 2^32
    
  https://en.wikipedia.org/wiki/Linear_congruential_generator

  a-1: dividable by 2
  a-1: multiple of 4
  c: not dividable by 2
  
*/


uint32_t z = 0;

uint16_t lcg(void)
{
  //z = (uint32_t)((uint32_t)214013*(uint32_t)z + (uint32_t)2531011);
  z = (uint32_t)((uint32_t)22695477*(uint32_t)z + (uint32_t)1);
  return z>>12;
}


int main()
{
  

  setHSI32MhzSysClock();					/* change to 32MHz */
  
  RCC->IOPENR |= RCC_IOPENR_IOPAEN;		/* Enable clock for GPIO Port A */
  __NOP();
  __NOP();

  GPIOA->MODER &= ~GPIO_MODER_MODE8;	/* clear mode for PA8 */
  GPIOA->MODER |= GPIO_MODER_MODE8_0;	/* Output mode for PA8 */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;	/* no Push/Pull for PA8 */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEED8;	/* low speed for PA8 */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8;	/* no pullup/pulldown for PA8 */
  GPIOA->BSRR = GPIO_BSRR_BR_8;		/* atomic clr PA8 */


  //GPIOA->MODER &= ~GPIO_MODER_MODE4;	/* clear mode for PA8 */
  GPIOA->MODER |= GPIO_MODER_MODE4_Msk;	/* Analog mode for PA4 */
  __NOP();
  __NOP();

  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* Enable the peripheral clock of the DAC */

  //DAC->CR = DAC_CR_TSEL1_Msk;
  DAC->CR = DAC->CR 
 //   |  DAC_CR_WAVE1_0 	/* Noise */
    | DAC_CR_MAMP1_3 
    | DAC_CR_MAMP1_1 
    | DAC_CR_MAMP1_0 
    | DAC_CR_BOFF1 
//    | DAC_CR_TEN1 	/* DAC trigger enable */
    | DAC_CR_EN1; /* enable DAC1 */    
  DAC->DHR12R1 = 0; /* Define the low value of the triangle on */ 
  
  
    
  
  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  //__disable_irq();
  
  for(;;)
  {
    //DAC->DHR12R1++;
    //DAC->DHR12R1 &= 0x03ff;
    DAC->DHR12R1 = lcg() & 0x01ff;
    delay_micro_seconds(2);
  }
}
