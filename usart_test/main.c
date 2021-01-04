/* 

  USART test for the STM32L072  Project

  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  

  Linux:
    stty -F /dev/ttyUSB0 sane 115200 && cat /dev/ttyUSB0
    or stty -F /dev/ttyUSB0 sane 115200 igncr  && cat /dev/ttyUSB0
    screen /dev/ttyUSB0  (terminate with "C-a k" or "C-a \")
    minicom -D /dev/ttyUSB0  -b 115200 (terminate with "C-a x", change CR mode: "C-a u")
    

  Ensure that -DUSER_VECT_TAB_ADDRESS is set during compilation, otherwise
  interrupts will not work after the "go" commant of the flashware USART upload.
  
  Problem is, that SCB->VTOR doesn't point to FLASH_BASE, so manually
  assigning SCB->VTOR = FLASH_BASE;	will also fix this.


  BSD 3-Clause License

  Copyright (c) 2021, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "stm32l0xx.h"
#include "sysclk.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS_8;		// atomic set PA8 
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		// atomic clr PA8 
}



void usart1_write_byte(uint8_t b)
{
  while ( (USART1->ISR & USART_ISR_TC) == 0 )
      ;
  USART1->TDR = b;
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

  
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  __NOP();
  __NOP();
    
  RCC->CCIPR &= RCC_CCIPR_USART1SEL;		// clear clock selection
  RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;	// select system clock
  
  
  GPIOA->MODER &= ~GPIO_MODER_MODE9;  // clear mode  
  GPIOA->MODER |= GPIO_MODER_MODE9_1;  // enable alternate functions
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;		// clear alternate function
  GPIOA->AFR[1] |= 4 << GPIO_AFRH_AFSEL9_Pos ;		// AF4: USART pins

  GPIOA->MODER &= ~GPIO_MODER_MODE10;  // clear mode  
  GPIOA->MODER |= GPIO_MODER_MODE10_1;  // enable alternate functions
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;		// clear alternate function
  GPIOA->AFR[1] |= 4 << GPIO_AFRH_AFSEL10_Pos ;		// AF4: USART pins

  USART1->BRR = 278; 	/* 32000000U / 115200 with 16x oversampling */ ;
  //USART1->BRR = 32000000U / 9600;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE;	/* default 8-N-1 configuration, transmit & receive enable */
  USART1->CR1 |= USART_CR1_UE;	/* enable usart */

  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  
  for(;;)
  {
    usart1_write_byte('a');
    usart1_write_byte('b');
    usart1_write_byte('\n');
    
    
    delay_micro_seconds(1000000);
  }
}
