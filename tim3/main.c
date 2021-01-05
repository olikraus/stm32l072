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


  GPIOA->MODER &= ~GPIO_MODER_MODE5;	/* clear mode for PA5 */
  GPIOA->MODER |= GPIO_MODER_MODE5_1;	/* AF mode for PA5 */

  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5;		// clear alternate function
  GPIOA->AFR[0] |= 5 << GPIO_AFRL_AFSEL5_Pos ;		// AF5: TIM2_CH1

  
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  __NOP();
  __NOP();
  
  TIM2->PSC = 0;		// no prescaler
  TIM2->ARR = 0x1fff;	// auto reload
  TIM2->CCR1 = 0xfff;
  
  TIM2->CR1 	= 0		
//			;		// maybe set the URS bit
			;		
  TIM2->CR2 	= 2<TIM_CR2_MMS_Pos   // TRGO triggered from update event
		//	| TIM_CR2_CCDS		// generate DMA request
			;
  TIM2->CCMR1 	= 6 << TIM_CCMR1_OC1M_Pos // PWM Mode 1
				| TIM_CCMR1_OC1PE 		// enable shadow registor for CC1
				;
  TIM2->CCER		= TIM_CCER_CC1E				// output compare result to pin
				;
  TIM2->CR1 		|= TIM_CR1_CEN;		// enable
  TIM2->EGR 		|= TIM_EGR_UG;		// force update

  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  //__disable_irq();
  
  for(;;)
  {
  }
}
