/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-22     Rick       the first version
 */
#ifndef RADIO_RADIO_ENCODER_H_
#define RADIO_RADIO_ENCODER_H_

void RadioEnqueue(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data);
void RadioQueue_Init(void);

#endif /* RADIO_RADIO_ENCODER_H_ */
