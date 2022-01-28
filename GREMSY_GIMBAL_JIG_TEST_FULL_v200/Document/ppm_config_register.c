static void ppm_config(void)
{
 /* Enable GPIO Channel3/3N Clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /* Configure TIM2_Channel1 in output, push-pull & alternate function mode */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* DMA1 clock enable */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  /* Configure DMA1 Channel5 CR register */
  /* Reset DMA1 Channel5 control register */
  DMA1_Stream6->CR = 0;
  /* Set CHSEL bits according to DMA Channel 5 */
  /* Set DIR bits according to Memory to peripheral direction */
  /* Set PINC bit according to DMA Peripheral Increment Disable */
  /* Set MINC bit according to DMA Memory Increment Enable */
  /* Set PSIZE bits according to Peripheral DataSize = Word */
  /* Set MSIZE bits according to Memory DataSize Word */
  /* Set CIRC bit according to circular mode */
  /* Set PL bits according to very high priority */
  /* Set MBURST bits according to single memory burst */
  /* Set PBURST bits according to single peripheral burst */
  DMA1_Stream6->CR |=   DMA_CHANNEL_2 |
                        DMA_MEMORY_TO_PERIPH |
                        DMA_PINC_DISABLE|
                        DMA_MINC_ENABLE|
                        DMA_PDATAALIGN_WORD|
                        DMA_MDATAALIGN_WORD |
                        DMA_CIRCULAR |
                        DMA_PRIORITY_HIGH |
                        DMA_FIFOMODE_ENABLE|
                        DMA_FIFO_THRESHOLD_FULL|
                        DMA_MBURST_SINGLE |
                        DMA_PBURST_SINGLE;
  DMA1_Stream6->CR &= (~DMA_IT_TC);
  DMA1_Stream6->CR &= (~DMA_IT_DME);
  DMA1_Stream6->CR &= (~DMA_IT_FE);
  DMA1_Stream6->CR &= (~DMA_IT_HT);
  DMA1_Stream6->CR &= (~DMA_IT_TE);
  /* Disable DMA1 Channel5 */
  DMA1_Stream6->CR &= (~DMA_SxCR_EN);
  /* Write to DMA1 Channel5 number of data’s register */
  DMA1_Stream6->NDTR = 9;
  /* Write to DMA1 Channel5 peripheral address register */
  DMA1_Stream6->PAR = (uint32_t)&TIM4->DMAR;
  /* Write to DMA1 Channel5 Memory address register */
  /* Set the address to the memory buffer “aSRC_Buffer?? */
  DMA1_Stream6->M0AR = (uint32_t)aSRC_Buffer;

  __HAL_RCC_TIM4_CLK_ENABLE();
  /* Select the Up-counting for TIM1 counter */
  TIM4->PSC = 83;
  /* Reset mode selection bit fiels*/
  TIM4->CR1 &= ~( TIM_CR1_DIR | TIM_CR1_CMS);
  /* selct Up-counting mode */
  TIM4->CR1 |= TIM_COUNTERMODE_UP;
  /* SET PWM1 mode */
  /* set the Output Compare Mode Bits */
//  TIM4->CCMR2 |= TIM_CCMR2_OC4M;
//  TIM4->CCMR2 &= ~TIM_CCMR2_CC4S;
  /* Select the output compare mode 1*/
  TIM4->CCMR2 |= (TIM_OCMODE_PWM1 << 8);
  /* Enable the output compare 1 Preload */
  TIM4->CCMR2 |= TIM_CCMR2_OC4PE;
  // ccr4
    TIM4->CCR4 = 400;
  /* Enable auto-reload Preload */
//  TIM4->CR1 |= TIM_CR1_ARPE;
  /* TIM1 DMA Update enable */
  TIM4->DIER |= TIM_DMA_UPDATE;
  /* Configure of the DMA Base register and the DMA Burst Length */
//  /* Reset DBA and DBL bit fields */
//  TIM4->DCR &= ~TIM_DCR_DBA;
//  TIM4->DCR &= ~TIM_DCR_DBL;
  /* Select the DMA base register and DMA burst length */
  TIM4->DCR = TIM_DMABase_ARR | TIM_DMABurstLength_1Transfer;
//  /* Enable UEV by setting UG bit to Load buffer data into preload registers
//  */
//  TIM4->EGR |= TIM_EGR_UG;
//  /* wait until the RESET of UG bit*/
//  while((TIM4->EGR & TIM_EGR_UG) == SET){}
//  /* Enable UEV by setting UG bit to load data from preload to active
//  registers */
//  TIM4->EGR |= TIM_EGR_UG;
//  /* Enable the TIM1 Main Output */
//  TIM4->BDTR |= TIM_BDTR_MOE;
  /* Enable CC1 output*/
  TIM4->CCER |= TIM_CCER_CC4E ;//| TIM_CCER_CC2E;
  /* Enable the TIM Counter */
  TIM4->CR1 |= TIM_CR1_CEN;

  /* Enable DMA1 Channel5 */
  DMA1_Stream6->CR |= (uint32_t)DMA_SxCR_EN;
}