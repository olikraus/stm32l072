/* 

  triangle output at DAC1 (STM32L072  Project)

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
    GPIOA->BSRR = GPIO_BSRR_BS_8;		/* atomic set PA8 */
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		/* atomic clr PA8 */
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


  //GPIOA->MODER &= ~GPIO_MODER_MODE4;	/* clear mode for PA8 */
  GPIOA->MODER |= GPIO_MODER_MODE4_Msk;	/* Analog mode for PA4 */
  __NOP();
  __NOP();

  RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* Enable the peripheral clock of the DAC */

  //DAC->CR = DAC_CR_TSEL1_Msk;
  DAC->CR = DAC->CR 
    |  DAC_CR_WAVE1_1 	/* triangle */
    | DAC_CR_MAMP1_3 
//    | DAC_CR_MAMP1_2 
    | DAC_CR_MAMP1_0 
    | DAC_CR_BOFF1 
    | DAC_CR_TEN1 	/* DAC trigger enable */
    | DAC_CR_EN1; /* enable DAC1 */    
  DAC->DHR12R1 = 0; /* Define the low value of the triangle on */ 
  
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  __NOP();
  __NOP();
  //TIM6->DIER = 0;	// no interrupts, no DMA
  TIM6->PSC = 0;	// no prescaler 
  TIM6->ARR=8*16; 
  TIM6->CR2 = TIM_CR2_MMS_1;	// TRGO on update
  TIM6->CR1 = TIM_CR1_CEN;	// enable
    
  
  
  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  //__disable_irq();
  
  for(;;)
  {
    IWDG->KR = 0xaaaa;
    WWDG->CR = 0x7f;
  __NOP();
  __NOP();
  }
}
