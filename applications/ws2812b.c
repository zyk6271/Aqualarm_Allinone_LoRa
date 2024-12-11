#include "ws2812b.h"
#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"

const RGB_Color_TypeDef BLACK    = {0,0,0};

uint32_t Pixel_Buf[Pixel_NUM + 1][24] = {0};

TIM_HandleTypeDef tim2_handle;
extern DMA_HandleTypeDef hdma_tim2_ch1;

void ws2812b_init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    tim2_handle.Instance = TIM2;
    tim2_handle.Init.Prescaler = 0;
    tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim2_handle.Init.Period = 59;
    tim2_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&tim2_handle) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&tim2_handle, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&tim2_handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&tim2_handle);

    RGB_SetColor(0,BLACK);
    RGB_SetColor(1,BLACK);
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&tim2_handle);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

void ws2812b_green(uint8_t id,uint8_t value)
{
    if(value)
    {
        for(uint8_t i=0;i<8;i++) Pixel_Buf[id][i] = ( (25 & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
    }
    else
    {
        for(uint8_t i=0;i<8;i++) Pixel_Buf[id][i] = ( (0 & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
    }
}

void ws2812b_red(uint8_t id,uint8_t value)
{
    if(value)
    {
        for(uint8_t i=8;i<16;i++) Pixel_Buf[id][i] = ( (25 & (1 << (15 -i)))? (CODE_1):CODE_0 );//数组某一行8-15转化存放R
    }
    else
    {
        for(uint8_t i=8;i<16;i++) Pixel_Buf[id][i] = ( (0 & (1 << (15 -i)))? (CODE_1):CODE_0 );//数组某一行8-15转化存放R
    }
}

void ws2812b_blue(uint8_t id,uint8_t value)
{
    if(value)
    {
        for(uint8_t i=16;i<24;i++) Pixel_Buf[id][i] = ( (25 & (1 << (23 -i)))? (CODE_1):CODE_0 );//数组某一行8-15转化存放R
    }
    else
    {
        for(uint8_t i=16;i<24;i++) Pixel_Buf[id][i] = ( (0 & (1 << (23 -i)))? (CODE_1):CODE_0 );//数组某一行8-15转化存放R
    }
}
/*
    功能：设定单个RGB LED的颜色，把结构体中RGB的24BIT转换为0码和1码
*/
void RGB_SetColor(uint8_t LedId,RGB_Color_TypeDef Color)
{
    uint8_t i;

    for(i=0;i<8;i++) Pixel_Buf[LedId][i] = ( (Color.G & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
    for(i=8;i<16;i++) Pixel_Buf[LedId][i] = ( (Color.R & (1 << (15-i)))? (CODE_1):CODE_0 );//数组某一行8~15转化存放R
    for(i=16;i<24;i++) Pixel_Buf[LedId][i] = ( (Color.B & (1 << (23-i)))? (CODE_1):CODE_0 );//数组某一行16~23转化存放B
}

/*
    功能：发送数组
*/
void RGB_SendArray(void)
{
    HAL_TIM_PWM_Stop_DMA(&tim2_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_DMA(&tim2_handle, TIM_CHANNEL_1, (uint32_t *)Pixel_Buf,(Pixel_NUM + 1)*24);
}
