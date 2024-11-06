#include "ws2812b.h"
#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"

const RGB_Color_TypeDef BLACK    = {0,0,0};

uint32_t Pixel_Buf[Pixel_NUM + 1][24] = {0};

TIM_HandleTypeDef pwm_handle;
extern DMA_HandleTypeDef hdma_tim1_ch1;

void ws2812b_init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    pwm_handle.Instance = TIM1;
    pwm_handle.Init.Prescaler = 0;
    pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm_handle.Init.Period = 59;
    pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm_handle.Init.RepetitionCounter = 0;
    pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&pwm_handle) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&pwm_handle, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&pwm_handle) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&pwm_handle, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&pwm_handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&pwm_handle, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&pwm_handle);

    RGB_SetColor(0,BLACK);
    RGB_SetColor(1,BLACK);
}

void TIM1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&pwm_handle);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

void ws2812b_green(uint8_t id,uint8_t value)
{
    if(value)
    {
        for(uint8_t i=0;i<8;i++) Pixel_Buf[id][i] = ( (50 & (1 << (7 -i)))? (CODE_1):CODE_0 );//数组某一行0~7转化存放G
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
        for(uint8_t i=8;i<16;i++) Pixel_Buf[id][i] = ( (50 & (1 << (15 -i)))? (CODE_1):CODE_0 );//数组某一行8-15转化存放R
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
        for(uint8_t i=16;i<24;i++) Pixel_Buf[id][i] = ( (50 & (1 << (23 -i)))? (CODE_1):CODE_0 );//数组某一行8-15转化存放R
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
    HAL_TIM_PWM_Stop_DMA(&pwm_handle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_DMA(&pwm_handle, TIM_CHANNEL_1, (uint32_t *)Pixel_Buf,(Pixel_NUM + 1)*24);
}
