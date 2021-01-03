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


*/

#include "stm32l0xx.h"

volatile unsigned long SysTickCount = 0;

void __attribute__ ((interrupt, used)) SysTick_Handler(void)
{
  SysTickCount++;  
  
  if ( SysTickCount & 1 )
    GPIOA->BSRR = GPIO_BSRR_BS_8;		// atomic set PA8 
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		// atomic clr PA8 
}

/* setup 32MHz as system clock */
void setHSIClock(void)
{
  
  
  /* test if the current clock source is something else than HSI */
  if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) 
  {
    /* enable HSI */
    RCC->CR |= RCC_CR_HSION;    
    /* wait until HSI becomes ready */
    while ( (RCC->CR & RCC_CR_HSIRDY) == 0 )
      ;      
 
    /* enable the HSI "divide by 4" bit */
    RCC->CR |= (uint32_t)(RCC_CR_HSIDIVEN);
    /* wait until the "divide by 4" flag is enabled */
    while((RCC->CR & RCC_CR_HSIDIVF) == 0)
      ;
    
       
    /* then use the HSI clock */
    RCC->CFGR = (RCC->CFGR & (uint32_t) (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; 
    
    /* wait until HSI clock is used */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
      ;
  }
  
  /* disable PLL */
  RCC->CR &= (uint32_t)(~RCC_CR_PLLON);
  /* wait until PLL is inactive */
  while((RCC->CR & RCC_CR_PLLRDY) != 0)
    ;

  /* set latency to 1 wait state */
  FLASH->ACR |= FLASH_ACR_LATENCY;
  
  /* At this point the HSI runs with 4 MHz */
  /* Multiply by 16 divide by 2 --> 32 MHz */
  RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_PLLMUL| RCC_CFGR_PLLDIV ))) | (RCC_CFGR_PLLMUL16 | RCC_CFGR_PLLDIV2); 
  
  /* enable PLL */
  RCC->CR |= RCC_CR_PLLON; 
  
  /* wait until the PLL is ready */
  while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    ;

  /* use the PLL has clock source */
  RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); 
  /* wait until the PLL source is active */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) 
    ;
  
  /* instead of calling SystemCoreClockUpdate() just update SystemCoreClock directly */
  SystemCoreClock = (uint32_t)32000000U;
}


void usart1_write_byte(uint8_t b)
{
  while ( (USART1->ISR & USART_ISR_TC) == 0 )
      ;
  USART1->TDR = b;
}


int main()
{
  
  
  setHSIClock();							/* change to 32MHz */
  
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
