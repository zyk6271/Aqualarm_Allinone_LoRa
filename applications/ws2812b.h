/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-02     Rick       the first version
 */
#ifndef APPLICATIONS_WS28B12_H_
#define APPLICATIONS_WS28B12_H_

#include "stdint.h"

/*这里是上文计算所得CCR的宏定义*/
#define CODE_1       (39)       //1码定时器计数次数
#define CODE_0       (18)       //0码定时器计数次数

/*建立一个定义单个LED三原色值大小的结构体*/
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
}RGB_Color_TypeDef;

#define Pixel_NUM 3  //LED数量宏定义，这里我使用一个LED，（单词pixel为像素的意思）

void RGB_SetColor(uint8_t LedId,RGB_Color_TypeDef Color);//给一个LED装载24个颜色数据码（0码和1码）
void Reset_Load(void); //该函数用于将数组最后24个数据变为0，代表RESET_code
void RGB_SendArray(void);          //发送最终数组
void RGB_RED(uint16_t Pixel_Len);  //显示红灯
void RGB_GREEN(uint16_t Pixel_Len);//显示绿灯
void RGB_BLUE(uint16_t Pixel_Len); //显示蓝灯
void RGB_WHITE(uint16_t Pixel_Len);//显示白灯

#endif /* APPLICATIONS_WS28B12_H_ */
