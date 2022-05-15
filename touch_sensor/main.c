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

/*================================================*/

/*
  Measure the number or processor clock cycles:
  uint32_t start;

  start = SysTick->VAL;
  ...
  getProcessorClockDelta(start); // return the number of processor cycles since start was recorded

  Limitation: The number of cycles between start and getProcessorClockDelta(start) must not exceed Systick->LOAD
  
*/
uint32_t getProcessorClockDelta(uint32_t start_value)
{
  uint32_t current_value = SysTick->VAL;
  /* SysTick->VAL is decremented, so the simple case is current_value < start_value */
  if ( current_value < start_value )
    return start_value-current_value;
  /* reload happend since start_value */
  return SysTick->LOAD - current_value + start_value;
}

/*
  pin: 0..15
  value: 0..1
  pupd: 0..2  (0=no pullup/pulldown, 1:pullup, 2:pulldown)
*/
void gpio_config_output(GPIO_TypeDef *gpio, unsigned pin, unsigned value, unsigned pupd)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
  uint32_t m1 = ~b1;         /* one bit mask */
  uint32_t b01 = (((uint32_t)1)<<(pin*2)); /* 01 bit pattern */
  //uint32_t b10 = (((uint32_t)2)<<(pin*2)); /* 10 bit pattern */
  uint32_t b11 = (((uint32_t)3)<<(pin*2)); /* 11 bit pattern */
  
  gpio->MODER &= m2;
  gpio->MODER |= b01;             /* 01: output mode */
  gpio->OTYPER &= m1;           /* push pull */
  gpio->OSPEEDR &= m2;
  gpio->OSPEEDR |= b11;         /* 11: very fast */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)pupd)<<(pin*2);             /* 10: pull down */
  
  if ( value != 0 )
    gpio->BSRR = b1;
  else
    gpio->BRR = b1;
}

void gpio_config_input(GPIO_TypeDef *gpio, unsigned pin, unsigned pupd)
{
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
 
  gpio->MODER &= m2;    /* 00: input */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)pupd)<<(pin*2);             /* 10: pull down */
}

uint32_t getChangeToCount(GPIO_TypeDef *gpio, unsigned pin, unsigned value)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t start, count;
  uint32_t v1 = 0;
  uint32_t max = 2000;
  if ( value != 0 )
    v1 = b1;
  start = SysTick->VAL;
  while( max > 0 )
  {
    if ( (gpio->IDR & b1) == v1 )
      break;
    max--;
  }
  count = getProcessorClockDelta(start);
  if ( max == 0 )
    return 0xffffffff;
  return count;
}

uint32_t iosample[32];
uint32_t iosample2[32];

uint32_t getChangeTo0(GPIO_TypeDef *gpio, unsigned pin)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
  uint32_t m1 = ~b1;         /* one bit mask */
  uint32_t b01 = (((uint32_t)1)<<(pin*2)); /* 01 bit pattern */
  //uint32_t b10 = (((uint32_t)2)<<(pin*2)); /* 10 bit pattern */
  uint32_t b11 = (((uint32_t)3)<<(pin*2)); /* 11 bit pattern */
  uint32_t start;
  uint32_t *s = iosample;
  uint32_t i;
  
  gpio->MODER &= m2;
  gpio->MODER |= b01;             /* 01: output mode */
  gpio->OTYPER &= m1;           /* push pull */
  gpio->OSPEEDR &= m2;
  gpio->OSPEEDR |= b11;         /* 11: very fast */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)2)<<(pin*2);             /* 10: pull down */
  
  gpio->BSRR = b1;      /* high output */
  delay_system_ticks(100);     /* wait until high is stable */
  
  //start = SysTick->VAL; /* get start value */
  
  gpio->MODER &= m2;    /* change to input */

  /* the pulldown should enforce a zero after some time */
  //do {
  //} while( (gpio->IDR & b1) != 0 ) ;
  

  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;

  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  *s++ = gpio->IDR;
  
  for( i = 0; i < 32; i++ )
  {
    if ( (iosample[i] & b1) == 0 )
      return i;
  }
  return i;
  //return getProcessorClockDelta(start);
}


uint32_t getDMAChangeTo0(GPIO_TypeDef *gpio, unsigned pin)
{
  uint32_t b1 = (((uint32_t)1)<<pin);
  uint32_t m2 = ~(((uint32_t)3)<<(pin*2));         /* two bit mask */
  uint32_t m1 = ~b1;         /* one bit mask */
  uint32_t b01 = (((uint32_t)1)<<(pin*2)); /* 01 bit pattern */
  //uint32_t b10 = (((uint32_t)2)<<(pin*2)); /* 10 bit pattern */
  uint32_t b11 = (((uint32_t)3)<<(pin*2)); /* 11 bit pattern */
  uint32_t i;
  
  gpio->MODER &= m2;
  gpio->MODER |= b01;             /* 01: output mode */
  gpio->OTYPER &= m1;           /* push pull */
  gpio->OSPEEDR &= m2;
  gpio->OSPEEDR |= b11;         /* 11: very fast */
  gpio->PUPDR &= m2;
  gpio->PUPDR |= ((uint32_t)2)<<(pin*2);             /* 10: pull down */
  
  gpio->BSRR = b1;      /* high output */
  delay_system_ticks(100);     /* wait until high is stable */
  
  //start = SysTick->VAL; /* get start value */

  RCC->AHBENR |= RCC_AHBENR_DMA1EN;        /* Enable DMA1 */
  __NOP();
  __NOP();
RCC->AHBRSTR |= RCC_AHBRSTR_DMA1RST;
  __NOP();
  __NOP();
  RCC->AHBRSTR &= ~RCC_AHBRSTR_DMA1RST;
  __NOP();
  __NOP();
  
  /* the pulldown should enforce a zero after some time */
  //do {
  //} while( (gpio->IDR & b1) != 0 ) ;
  for( i = 0; i < 32; i++ )
  {
    iosample[i] = 0;
  }

  //DMA1->IFCR |= DMA_ISR_TCIF2;
  DMA1_Channel2->CCR = 0;		/* disable DMA channel */  
  TIM6->CR1 = 0;
  
  DMA1_CSELR->CSELR |= (uint32_t)(9 << DMA_CSELR_C2S_Pos );  /* TIM6_UP/DAC channel 1  */
  
  DMA1_Channel2->CCR |= DMA_CCR_MEM2MEM  /* memory to memory mode */
    //| DMA_CCR_MINC	/* memory increment */ 
    //| DMA_CCR_PINC	/* io increment */ 
    | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1	/* 32 bit size */
    | DMA_CCR_PL_0 | DMA_CCR_PL_1               /* high prio */
    //| DMA_CCR_DIR 							/* read from i/o */
    //| DMA_CCR_TEIE 						/* transfer error interrupt */
    //| DMA_CCR_TCIE 						/* transfer complete interrupt enable */
    //| DMA_CCR_CIRC 						/* circular mode */
    ;

  DMA1_Channel2->CPAR = (uint32_t) (&(gpio->ODR)); /* connect to input bits */
  DMA1_Channel2->CPAR = (uint32_t)(iosample2); 
  DMA1_Channel2->CMAR = (uint32_t)(iosample);
  DMA1_Channel2->CNDTR = 1;
  
  //NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
  //NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn,3);


  DMA1_Channel2->CCR |= DMA_CCR_EN;		/* enable DMA channel */  
  
  TIM6->ARR=12;		
  TIM6->PSC = 0;	// no prescaler 
  TIM6->CR2 = TIM_CR2_MMS_1;	// TRGO on update
  TIM6->DIER = TIM_DIER_UDE;	// update DMA
  TIM6->CR1 = TIM_CR1_CEN;	// enable
  
  gpio->MODER &= m2;    /* change to input */
  

  delay_system_ticks(1000);     /* wait for DMA*/
  
  //return DMA1->IFCR;
  return DMA1_Channel2->CNDTR;
  for( i = 0; i < 32; i++ )
  {
    if ( (iosample[i] & b1) == 0 )
      return i;
  }
  return i;
  //return getProcessorClockDelta(start);
}

uint32_t getCapValue(GPIO_TypeDef *gpio, unsigned pin)
{
    /* configure the pin as output without any pullup/pulldown resistor */
    gpio_config_output(gpio, pin, 1, 1);
    /* ensure that the high output level is reached */ 
    delay_system_ticks(1000);
    /* enable pull down */
    gpio_config_output(gpio, pin, 1, 1);
    /* change to input, keep the pulldown */
    gpio_config_input(gpio, pin, 0);
    /* count sys ticks until 0 is detected */
    return getChangeToCount(gpio, pin, 0);
}

/*================================================*/
volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS_8;		// atomic set PA8 
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		// atomic clr PA8 
  
  if ( SysTickCount & 1 )
    gpio_config_output(GPIOA, 8, 1, 0);
  else
    gpio_config_output(GPIOA, 8, 0, 0);
  
}


static uint8_t usart_buf[32];


int main()
{
  
  setHSI32MhzSysClock();					/* change to 32MHz */
  
  RCC->IOPENR |= RCC_IOPENR_IOPAEN;		/* Enable clock for GPIO Port A */
  RCC->IOPENR |= RCC_IOPENR_IOPBEN;		/* Enable clock for GPIO Port B */
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;        /* Enable DMA1 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  
  __NOP();
  __NOP();


  
  usart1_init(115200, usart_buf, sizeof(usart_buf));
  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  
  for(;;)
  {
      
    delay_micro_seconds(100000);

    usart1_write_string("cap=");
    usart1_write_u32(getCapValue(GPIOB, 1));
    usart1_write_string(" cap=");
    usart1_write_u32(getChangeTo0(GPIOB, 1));
    usart1_write_string(" dmacap=");
    usart1_write_u32(getDMAChangeTo0(GPIOB, 1));
    usart1_write_string("\n");
  }
}
