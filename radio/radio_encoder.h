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

struct send_msg
{
    char    data_ptr[253];   /* 数据块首地址 */
    uint8_t data_size;   /* 数据块大小   */
    uint8_t need_ack;    /* 需要应答   */
    uint8_t parameter;   /* 数据参数   */
};

void RadioEnqueue(uint32_t Taget_Id,uint8_t counter,uint8_t Command,uint8_t Data);
void RadioQueue_Init(void);

#endif /* RADIO_RADIO_ENCODER_H_ */
