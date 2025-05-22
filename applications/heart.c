/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-28     Rick       the first version
 */
#include "rtthread.h"
#include "board.h"

#define DBG_TAG "heart"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

RNG_HandleTypeDef rng_handle;
rt_thread_t heart_thread = RT_NULL;
rt_timer_t heart_connect_timer = RT_NULL;

uint8_t gateway_heart_retry = 0;
uint32_t before_wait_time,after_wait_time = 0;

uint32_t random_second_get(uint32_t min,uint32_t max)
{
    uint32_t value, second = 0;
    HAL_RNG_GenerateRandomNumber(&rng_handle, &value);
    second = value % (max - min + 1) + min;
    return second;
}

void gateway_heart_check(void)
{
    if(aq_gateway_find() == 0)
    {
        return;
    }

    if(aq_device_recv_find(aq_gateway_find()))
    {
        aq_device_recv_set(aq_gateway_find(),0);
        wifi_led(1);//online
    }
    else
    {
        wifi_led(2);//offline
    }
}

void gateway_connect_start(void)
{
    if(aq_gateway_find())
    {
        gateway_heart_retry = 0;
        gateway_heart_upload_poweron();
        rt_timer_start(heart_connect_timer);
    }
}

void heart_connect_timer_callback(void *parameter)
{
    if(aq_device_recv_find(aq_gateway_find()) == 0)
    {
        if(gateway_heart_retry < 3)
        {
            gateway_heart_retry ++;
            gateway_heart_upload_poweron();
            rt_timer_start(heart_connect_timer);
        }
        else
        {
            wifi_led(2);//fail
        }
    }
}

void heart_thread_entry(void *parameter)
{
    if(aq_gateway_find())
    {
        wifi_led(0);
        rt_thread_mdelay(random_second_get(20,40) * 500);//10-20秒
        gateway_connect_start();
    }
    while (1)
    {
        if(aq_gateway_find())
        {
            before_wait_time = random_second_get(60,780);
            after_wait_time = 900 - before_wait_time;
            rt_thread_mdelay(before_wait_time * 1000);
            gateway_heart_upload_master();
            LOG_D("gateway_heart_upload_master send at %d",rt_tick_get());
            rt_thread_mdelay(after_wait_time * 1000);
        }
        else
        {
            rt_thread_mdelay(10);
        }
    }
}

void heart_init(void)
{
    rng_handle.Instance = RNG;
    if (HAL_RNG_Init(&rng_handle) != HAL_OK)
    {
        Error_Handler();
    }

    heart_connect_timer = rt_timer_create("connect", heart_connect_timer_callback, RT_NULL, 3000, RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_ONE_SHOT);
    heart_thread = rt_thread_create("heart", heart_thread_entry, RT_NULL, 1024, 10, 10);
    rt_thread_startup(heart_thread);
}
