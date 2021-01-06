/* 

  USART echo test for the STM32L072  Project

  Configuration is 115200 8-N-1. Only "\n" is sent. Receiving terminal should add \r
  (e.g. use add CR function in "minicom")
  

  Linux:
    stty -F /dev/ttyUSB0 sane 115200 && cat /dev/ttyUSB0
    or stty -F /dev/ttyUSB0 sane 115200 igncr  && cat /dev/ttyUSB0
    screen /dev/ttyUSB0  115200 (terminate with "C-a k" or "C-a \")
    minicom -D /dev/ttyUSB0  -b 115200 (terminate with "C-a x", change CR mode: "C-a u", disable HW control flow!)
    

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
#include "delay.h"
#include "usart.h"
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


static uint8_t usart_buf[32];

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

  
  usart1_init(115200, usart_buf, sizeof(usart_buf));
  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
    
  SPI2->CR1 = 0;	/* disable SPI */
  SPI2->I2SCFGR 	= SPI_I2SCFGR_I2SMOD				// enable I2S
				| (3<<SPI_I2SCFGR_I2SCFG_Pos)		// master receive
				| (0<<SPI_I2SCFGR_I2SSTD_Pos)  	// I2S Philips standard (WS once cycle before MSB)
				| (1<<SPI_I2SCFGR_DATALEN_Pos) 	// 24 bit data len
				;

  SPI->I2SPR 		= (11<<SPI_I2SPR_I2SDIV_Pos)		// see table 144, sec 30.6.3, p864
				| SPI_I2SPR_ODD
				;
				
  SPI2->I2SCFGR 	|= SPI_I2SCFGR_I2SE;		// enable
		  

  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR; /* (1) */
  SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE; /* (2) */
  SPI2->CR1 |= SPI_CR1_SPE; /* enable SPI */

  for(;;)
  {
    usart1_write_string("abc\n");
    
    
    delay_micro_seconds(1000000);
    
  }
}
