/* 

  triangle output at DAC1 (STM32L072  Project)

  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.


*/

#include "stm32l0xx.h"
#include "sysclk.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS_8;		/* atomic set PA8 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		/* atomic clr PA8 */
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

/*
MAMP:
0000: Unmask bit0 of LFSR/ triangle amplitude equal to 1
0001: Unmask bits[1:0] of LFSR/ triangle amplitude equal to 3
0010: Unmask bits[2:0] of LFSR/ triangle amplitude equal to 7
0011: Unmask bits[3:0] of LFSR/ triangle amplitude equal to 15
0100: Unmask bits[4:0] of LFSR/ triangle amplitude equal to 31
0101: Unmask bits[5:0] of LFSR/ triangle amplitude equal to 63
0110: Unmask bits[6:0] of LFSR/ triangle amplitude equal to 127
0111: Unmask bits[7:0] of LFSR/ triangle amplitude equal to 255
1000: Unmask bits[8:0] of LFSR/ triangle amplitude equal to 511
1001: Unmask bits[9:0] of LFSR/ triangle amplitude equal to 1023
1010: Unmask bits[10:0] of LFSR/ triangle amplitude equal to 2047
≥ 1011: Unmask bits[11:0] of LFSR/ triangle amplitude equal to 4095

triangle counter starts with 0 (DHR12R1) and counts to the MAMP value up and down.
This means, double amplitude also means half frequency
*/
  //DAC->CR = DAC_CR_TSEL1_Msk;
  DAC->CR = DAC->CR 
    |  DAC_CR_WAVE1_1 	/* triangle */
    | DAC_CR_MAMP1_3 
    | DAC_CR_MAMP1_0 
    | DAC_CR_BOFF1 
    | DAC_CR_TEN1 	/* DAC trigger enable */
    | DAC_CR_EN1; /* enable DAC1 */    
  DAC->DHR12R1 = 0; /* Define the low value of the triangle on */ 
  
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  __NOP();
  __NOP();
  TIM6->PSC = 0;	// no prescaler 
  TIM6->ARR=8*16;               // defines update speed of the triangle counter (frequency) 
  TIM6->CR2 = TIM_CR2_MMS_1;	// TRGO on update
  TIM6->CR1 = TIM_CR1_CEN;	// enable
    
  
  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  //__disable_irq();
  
  for(;;)
  {
  }
}
