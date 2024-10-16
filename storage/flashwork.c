/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-13     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <spi_flash.h>
#include <drv_spi.h>
#include <string.h>
#include <stdlib.h>
#include "pin_config.h"
#include "fal.h"
#include "easyflash.h"
#include "flashwork.h"
#include "radio_protocol.h"
#include "water_work.h"

#define DBG_TAG "FLASH"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define MAX_SLAVER_DEVICE  12

rt_spi_flash_device_t w25q16;

uint32_t total_slot = 0;

ALIGN(RT_ALIGN_SIZE)
static rt_slist_t _device_list = RT_SLIST_OBJECT_INIT(_device_list);
static struct rt_mutex flash_mutex;

extern WariningEvent SlaverOfflineEvent;

int storage_init(void)
{
    rt_err_t status;
    rt_mutex_init(&flash_mutex, "flash_mutex", RT_IPC_FLAG_FIFO);
    extern rt_spi_flash_device_t rt_sfud_flash_probe(const char *spi_flash_dev_name, const char *spi_dev_name);
    rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_15);
    w25q16 = rt_sfud_flash_probe("norflash0", "spi10");
    if (RT_NULL == w25q16)
    {
        LOG_E("sfud fail\r\n");
        return RT_ERROR;
    };
    status = fal_init();
    if (status == 0)
    {
        LOG_E("fal_init fail\r\n");
        return RT_ERROR;
    };
    status = easyflash_init();
    if (status != EF_NO_ERR)
    {
        LOG_E("easyflash_init fail\r\n");
        return RT_ERROR;
    };
    LOG_I("Storage Init Success\r\n");
    read_device_from_flash();
    aq_device_print();
    return RT_EOK;
}

uint32_t flash_get_key(char *key_name)
{
    uint8_t read_len = 0;
    uint32_t read_value = 0;
    char read_value_temp[32] = {0};
    read_len = ef_get_env_blob(key_name, read_value_temp, 32, NULL);
    if(read_len>0)
    {
        read_value = atol(read_value_temp);
    }
    else
    {
        read_value = 0;
    }

    return read_value;
}

void flash_set_key(char *key_name,uint32_t value)
{
    char *value_buf = rt_malloc(64);//申请临时buffer空间
    rt_sprintf(value_buf, "%d", value);
    ef_set_env_blob(key_name, value_buf,rt_strlen(value_buf));
    rt_free(value_buf);
}

void read_device_from_flash(void)
{
    total_slot = flash_get_key("slot");

    uint32_t index = 0;
    aqualarm_device_t *device = RT_NULL;
    while (index < 32)
    {
        if ((total_slot & (1 << index)))
        {
            device = (aqualarm_device_t *)rt_malloc(sizeof(aqualarm_device_t));
            char key[16] = {0};
            rt_sprintf(key, "index:%d", index);
            size_t len = ef_get_env_blob(key, device, sizeof(aqualarm_device_t) , NULL);
            if(len == 0)
            {
                rt_free(device);
            }
            else
            {
                rt_mutex_take(&flash_mutex, RT_WAITING_FOREVER);
                rt_slist_append(&_device_list, &(device->slist));
                rt_mutex_release(&flash_mutex);
            }
        }
        index++;
    }
}

int8_t get_free_device_slot(void)
{
    uint32_t index = 0; // 记录当前位的索引
    while (index < MAX_SLAVER_DEVICE)
    {
        if ((total_slot & (1 << index)) == 0)
        {
            total_slot |= 1 << index;
            return index;
        }
        index++;
    }

    return -1;
}

void delete_select_device_slot(uint32_t index)
{
    total_slot &= ~(1 << index);
}

static void aq_device_save(aqualarm_device_t * device)
{
    rt_mutex_take(&flash_mutex, RT_WAITING_FOREVER);
    char key[16] = {0};
    rt_sprintf(key, "index:%d", device->slot);
    ef_set_env_blob(key,device,sizeof(aqualarm_device_t));
    rt_mutex_release(&flash_mutex);
}

aqualarm_device_t *aq_device_find(uint32_t device_id)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_id)
        {
            return device;
        }
    }

    return RT_NULL;
}

uint32_t aq_gateway_find(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type == DEVICE_TYPE_GATEWAY)
        {
            return device->device_id;
        }
    }

    return 0;
}


aqualarm_device_t *aq_device_create(uint8_t rssi_level,uint8_t type,uint32_t device_id)
{
    aqualarm_device_t *check_device = aq_device_find(device_id);
    if(check_device != RT_NULL)
    {
        return check_device;
    }

    if(total_slot > 4095)
    {
        return RT_NULL;
    }

    int8_t slot = get_free_device_slot();
    if(slot < 0)
    {
        return RT_NULL;
    }

    aqualarm_device_t *device = (aqualarm_device_t *)rt_malloc(sizeof(aqualarm_device_t));
    if (device == RT_NULL)
    {
        return RT_NULL;
    }

    rt_memset(device, 0, sizeof(aqualarm_device_t));
    device->slot = slot;
    device->type = type;
    device->ack = 0;
    device->rssi_level = rssi_level;
    device->online = 1;
    device->battery = 0;
    device->device_id = device_id;
    device->waterleak = 0;
    rt_slist_init(&(device->slist));

    rt_mutex_take(&flash_mutex, RT_WAITING_FOREVER);
    rt_slist_append(&_device_list, &(device->slist));
    flash_set_key("slot",total_slot);
    aq_device_save(device);
    rt_mutex_release(&flash_mutex);

    beep_once();

    return device;
}

uint8_t aq_device_delete(uint32_t device_id)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_id)
        {
            rt_mutex_take(&flash_mutex, RT_WAITING_FOREVER);
            delete_select_device_slot(device->slot);
            flash_set_key("slot",total_slot);
            rt_slist_remove(&_device_list, &(device->slist));
            rt_mutex_release(&flash_mutex);
            rt_free(device);
            return RT_EOK;
        }
    }

    return RT_ERROR;
}

void aq_gateway_delete(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type == DEVICE_TYPE_GATEWAY)
        {
            aq_device_delete(device->device_id);
        }
    }
}

void aq_doorunit_delete(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type == DEVICE_TYPE_DOORUNIT)
        {
            aq_device_delete(device->device_id);
            gateway_sync_device_del(device->device_id);
        }
    }
}

void aq_device_print(void)
{
    rt_slist_t *node;
    rt_slist_for_each(node, &_device_list)
    {
        aqualarm_device_t *device = rt_slist_entry(node, aqualarm_device_t, slist);
        LOG_I("[device info %d]:addr %d,slot %d,type %d,rssi_level %d,snr %d\r\n",device->slot,device->device_id,device->slot,device->type,device->rssi_level,device->snr);
        LOG_I("[device status %d]:battery %d,waterleak %d,ack %d,online %d\r\n",device->slot,device->battery,\
                device->waterleak,device->ack,device->online);
    }
}

void aq_device_recv_set(uint32_t device_addr,uint8_t value)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_addr)
        {
            device->recv = value;
        }
    }
}

uint8_t aq_device_recv_find(uint32_t device_addr)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_addr)
        {
            return device->recv;
        }
    }

    return 0;
}

void aq_device_waterleak_set(uint32_t device_id,uint8_t value)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_id)
        {
            if(device->waterleak != value)
            {
                device->waterleak = value;
                aq_device_save(device);
            }
        }
    }
}

uint8_t aq_device_waterleak_find(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->waterleak)
        {
            return 1;
        }
    }

    return 0;
}

void aq_device_battery_set(uint32_t device_id,uint8_t value)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_id)
        {
            if(device->battery != value)
            {
                device->battery = value;
                aq_device_save(device);
            }
        }
    }
}

void aq_device_online_set(uint32_t device_id,uint8_t state)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_id)
        {
            if(device->online != state)
            {
                device->online = state;
                aq_device_save(device);
            }
        }
    }
}

void aq_device_rssi_level_set(uint32_t device_id,uint8_t rssi_level)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == device_id)
        {
            if(device->rssi_level != rssi_level)
            {
                device->rssi_level = rssi_level;
                aq_device_save(device);
            }
        }
    }
}

uint8_t aq_device_offline_find(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->online == 0 && device->type != DEVICE_TYPE_GATEWAY  && device->type != DEVICE_TYPE_DOORUNIT)
        {
            return 1;
        }
    }

    return 0;
}

uint8_t aq_device_offline_upload(uint8_t *send_buf)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    uint8_t len = 0;
    unsigned char *obj = send_buf;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->online == 0 && device->type != DEVICE_TYPE_GATEWAY)
        {
            *obj = (device->device_id >> 24) & 0xFF;
            obj ++;
            *obj = (device->device_id >> 16) & 0xFF;
            obj ++;
            *obj = (device->device_id >> 8) & 0xFF;
            obj ++;
            *obj = device->device_id & 0xFF;
            obj ++;
            len += 1;
        }
    }

    return len;
}

uint8_t aq_device_clone_upload(uint8_t *send_buf)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    uint8_t len = 0;
    unsigned char *obj = send_buf;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type != DEVICE_TYPE_GATEWAY)
        {
            *obj = (device->device_id >> 24) & 0xFF;
            obj ++;
            *obj = (device->device_id >> 16) & 0xFF;
            obj ++;
            *obj = (device->device_id >> 8) & 0xFF;
            obj ++;
            *obj = device->device_id & 0xFF;
            obj ++;
            *obj = device->type;
            obj ++;
            *obj = device->online;
            obj ++;
            *obj = device->rssi_level;
            obj ++;
            *obj = device->battery;
            obj ++;
            len += 1;
        }
    }

    return len;
}

uint8_t aq_device_slaver_count(void)
{
    rt_slist_t *node;
    uint8_t count = 0;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type != DEVICE_TYPE_GATEWAY)
        {
            count ++;
        }
    }

    return count;
}

void aq_device_heart_recv_clear(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type != DEVICE_TYPE_GATEWAY)
        {
            device->recv = 0;
        }
    }
}

uint8_t aqualarm_device_heart_recv(rx_format *rx_frame)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->device_id == rx_frame->source_addr)
        {
            device->recv = 1;
            aq_device_online_set(rx_frame->source_addr,1);
            aq_device_rssi_level_set(rx_frame->source_addr,rx_frame->rssi_level);
            return RT_EOK;
        }
    }

    return RT_ERROR;
}

void aq_device_heart_check(void)
{
    rt_slist_t *node;
    aqualarm_device_t *device = RT_NULL;
    rt_slist_for_each(node, &_device_list)
    {
        device = rt_slist_entry(node, aqualarm_device_t, slist);
        if(device->type != DEVICE_TYPE_GATEWAY)//except gateway
        {
            if(device->recv)
            {
                device->recv = 0;
            }
            else
            {
                aq_device_online_set(device->device_id,0);
            }
        }
    }
    if(aq_device_offline_find())
    {
        warning_enable(SlaverOfflineEvent);
        gateway_warning_slaver_offline();
    }
}
