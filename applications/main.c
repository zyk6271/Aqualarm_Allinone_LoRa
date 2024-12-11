/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-12     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <pin_config.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

const uint8_t fw_main_ver = 0x00;
const uint8_t fw_sub_ver = 0x0F;

int main(void)
{
    LOG_I("System Version is V1.%d.%d\r\n",fw_main_ver,fw_sub_ver);
    led_init();
    pd_init();
    storage_init();
    heart_init();
    button_init();
    warning_init();
    rtc_init();
    radio_init();
    valve_init();
    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
