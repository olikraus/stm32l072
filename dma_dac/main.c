/* 

  Use DMA to output a triangle waveat DAC1 (STM32L072  Project)

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
    GPIOA->BSRR = GPIO_BSRR_BS_8;		// atomic set PA8
  else
    GPIOA->BSRR = GPIO_BSRR_BR_8;		// atomic clr PA8 

}

#define AUDIO_SAMPLE_RATE 40000
#define AUDIO_BLOCK_SIZE 1000

/* SAMPLE_RATE=40000 and BLOCK_SIZE=1000 will create a 40000/(2*1000) Hz triangle wave */

uint16_t audio_block_data0[AUDIO_BLOCK_SIZE];
uint16_t audio_block_data1[AUDIO_BLOCK_SIZE];

void audio_block_init(uint16_t *data, int dir)
{
  int i;
  if ( dir == 0 )
  {
    for( i = 0; i < AUDIO_BLOCK_SIZE; i++ )
    {
      data[i] = i;
    }
  }
  else
  {
    for( i = 0; i < AUDIO_BLOCK_SIZE; i++ )
    {
      data[i] = AUDIO_BLOCK_SIZE-1-i;
    }
  }
}

void __attribute__ ((interrupt, used)) DMA1_Channel2_3_IRQHandler(void)
{
    int dir;
  /*
    Idea: Add watchdog update here. this IRQ is triggerted from DMA, which is 
    triggered by TIM6. If this doesn't work any more then we need to reset the device.
  */
  
  if ( DMA1->ISR & DMA_ISR_TCIF2 )
  {
    uint16_t *data;
    DMA1->IFCR |= DMA_ISR_TCIF2;

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;		/* disbale DMA channel */  
    
    data = DMA1_Channel2->CMAR;
    if ( data == (uint32_t)(audio_block_data0) )
    {
      DMA1_Channel2->CMAR = (uint32_t)(audio_block_data1);
      dir = 1;
    }
    else
    {
      DMA1_Channel2->CMAR = (uint32_t)(audio_block_data0);
      dir = 0;
    }
    
    DMA1_Channel2->CNDTR = AUDIO_BLOCK_SIZE;	/* reload data size */
    /* 
      This enable needs to happen before TIM6 timeout (TIM6->ARR value) 
      On the other side, it probably does not matter much if this is exceeded:
      In such a case once sample is skipped
    */
    DMA1_Channel2->CCR |= DMA_CCR_EN;		/* enable DMA channel */  

    /* starting from here, we can do the audio processing for the next block */
    audio_block_init(data, dir);
    
  }
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
    //| DAC_CR_MAMP1_3 
    //| DAC_CR_MAMP1_1 
    //| DAC_CR_MAMP1_0 
    | DAC_CR_BOFF1 
    | DAC_CR_TEN1 	/* DAC trigger enable */
    | DAC_CR_EN1; /* enable DAC1 */    
  DAC->DHR12R1 = 0; /* Define the low value of the triangle on */ 
    
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  DMA1_CSELR->CSELR |= (uint32_t)(9 << DMA_CSELR_C2S_Pos );  /* TIM6_UP/DAC channel 1  */
  DMA1_Channel2->CPAR = (uint32_t) (&(DAC->DHR12R1)); /* connect to DAC*/
  DMA1_Channel2->CMAR = (uint32_t)(audio_block_data0);
  DMA1_Channel2->CNDTR = AUDIO_BLOCK_SIZE;
  DMA1_Channel2->CCR |= DMA_CCR_MINC	/* memory increment */ 
    | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0	/* 16 bit size */
    | DMA_CCR_DIR 							/* read from memory */
    //| DMA_CCR_TEIE 						/* transfer error interrupt */
    | DMA_CCR_TCIE 						/* transfer complete interrupt enable */
    //| DMA_CCR_CIRC 						/* circular mode */
    ;
      
  NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  NVIC_SetPriority(DMA1_Channel2_3_IRQn,3);
  
  DMA1_Channel2->CCR |= DMA_CCR_EN;		/* enable DMA channel */  
  
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
  TIM6->ARR=SystemCoreClock/AUDIO_SAMPLE_RATE;		
  TIM6->PSC = 0;	// no prescaler 
  TIM6->CR2 = TIM_CR2_MMS_1;	// TRGO on update
  TIM6->DIER = TIM_DIER_UDE;	// update DMA
  TIM6->CR1 = TIM_CR1_CEN;	// enable
    

  SysTick->LOAD = 2000*500 *16- 1;
  SysTick->VAL = 0;
  SysTick->CTRL = 7;   /* enable, generate interrupt (SysTick_Handler), do not divide by 2 */
    
  //__disable_irq();
  
  for(;;)
  {
  }
}
