/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-27     Rick       the first version
 */
#include <rtthread.h>
#include <agile_button.h>
#include "pin_config.h"
#include "water_work.h"

#define DBG_TAG "key"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

uint8_t key_on_count,key_off_count,key_on_off_flag = 0;
uint8_t key_on_long_click,key_off_long_click = 0;

extern enum Device_Status DeviceStatus;

agile_btn_t *key_on_btn = RT_NULL;
agile_btn_t *key_off_btn = RT_NULL;
agile_btn_t *water_leak_btn = RT_NULL;
agile_btn_t *water_lost_btn = RT_NULL;

void key_on_click_handle(void)
{
    switch(DeviceStatus)
    {
    case ValveClose:
        if(valve_open() == RT_EOK)//valve open success
        {
            gateway_control_master_control(1);
        }
        break;
    case ValveOpen:
        beep_once();
        break;
    case SlaverLowPower:
        break;
    case SlaverUltraLowPower:
        beep_three_times();
        break;
    case SlaverSensorLeak:
        beep_three_times();
        break;
    case SlaverOffline:
        break;
    case MasterSensorLost:
        valve_open();
        break;
    case MasterSensorLeak:
        beep_three_times();
        break;
    case MasterSensorAbnormal:
        beep_three_times();
        break;
    case LearnDevice:
        break;
    case InternalValveFail:
        break;
    case ExternalValveFail:
        break;
    case MasterLowTemp:
        break;
    default:
        break;
    }
}

void key_off_click_handle(void)
{
    switch(DeviceStatus)
    {
    case ValveClose:
        if(get_valve_lock())
        {
            led_valve_fail();
        }
        else
        {
            beep_key_down();
        }
        break;
    case ValveOpen:
        if(valve_close() == RT_EOK)//valve close success
        {
            gateway_control_master_control(0);
        }
        break;
    case SlaverLowPower:
        break;
    case SlaverUltraLowPower:
        beep_once();
        break;
    case SlaverSensorLeak:
        beep_stop();
        break;
    case SlaverOffline:
        break;
    case MasterSensorLost:
        valve_close();
        beep_stop();
        break;
    case MasterSensorLeak:
        beep_stop();
        break;
    case MasterSensorAbnormal:
        beep_key_down();
        warning_all_clear();
        gateway_warning_master_leak(0);
        break;
    case LearnDevice:
        break;
    case InternalValveFail:
        beep_stop();
        break;
    case ExternalValveFail:
        beep_stop();
        break;
    case MasterLowTemp:
        beep_stop();
        beep_key_down();
        break;
    default:
        break;
     }
}

void key_off_long_hold_handle(void)
{
    if(key_off_count < 4)
    {
        key_off_count ++;
        LOG_D("key_off_long_hold_handle %d\r\n",key_off_count);
    }
    else
    {
        if(key_on_count)
        {
            key_on_off_long_click_handle();
        }
        else
        {
            if(key_off_long_click == 0)
            {
                key_off_long_click = 1;
                radio_start_learn_device();
            }
        }
    }
}

void key_off_long_free_handle(void)
{
    key_on_off_flag = 0;
    key_off_count = 0;
    key_off_long_click = 0;
}

void key_on_long_hold_handle(void)
{
    if(key_on_count < 4)
    {
        key_on_count ++;
        LOG_D("key_on_long_hold_handle %d\r\n",key_on_count);
    }
    else
    {
        if(key_off_count)
        {
            key_on_off_long_click_handle();
        }
        else
        {
            if(key_on_long_click == 0)
            {
                key_on_long_click = 1;
            }
        }
    }
}

void key_on_long_free_handle(void)
{
    key_on_off_flag = 0;
    key_on_count = 0;
    key_on_long_click = 0;
}

void key_on_off_long_click_handle(void)
{
    if(key_on_count > 3 && key_off_count > 3)
    {
        if(key_on_off_flag == 0)
        {
            key_on_off_flag = 1;
            LOG_D("key_on_off_long_click_handle\r\n");
            gateway_sync_device_reset();
            ef_env_set_default();
            led_factory_start();
            rt_thread_mdelay(3000);
            rt_hw_cpu_reset();
        }
    }
    else
    {
        LOG_D("key_on_off_long_click_handle %d %d\r\n",key_off_count,key_on_count);
    }
}

void water_leak_up_callback(agile_btn_t *btn)
{
    MasterStatusChangeToDeAvtive();
    LOG_D("water_leak_up_callback\r\n");
}

void water_leak_down_callback(agile_btn_t *btn)
{
    extern WariningEvent MasterSensorLeakEvent;
    warning_enable(MasterSensorLeakEvent);
    LOG_D("water_leak_down_callback\r\n");
}

void water_lost_plugin_callback(agile_btn_t *btn)
{
    warning_lost_clear();
    LOG_D("water_lost_up_callback\r\n");
}

void water_lost_plugout_callback(agile_btn_t *btn)
{
    extern WariningEvent MasterSensorLostEvent;
    warning_enable(MasterSensorLostEvent);
    LOG_D("water_lost_down_callback\r\n");
}

void button_init(void)
{
    rt_pin_mode(ANT_INT_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(ANT_EXT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(ANT_INT_PIN, PIN_LOW);
    rt_pin_write(ANT_EXT_PIN, PIN_HIGH);

    key_on_btn = agile_btn_create(KEY_ON_PIN, PIN_LOW, PIN_MODE_INPUT);
    key_off_btn = agile_btn_create(KEY_OFF_PIN, PIN_LOW, PIN_MODE_INPUT);
    water_leak_btn = agile_btn_create(SENSOR_LEAK_PIN, PIN_LOW, PIN_MODE_INPUT);
    water_lost_btn = agile_btn_create(SENSOR_LOST_PIN, PIN_LOW, PIN_MODE_INPUT);

    agile_btn_set_hold_cycle_time(key_on_btn,1000);
    agile_btn_set_hold_cycle_time(key_off_btn,1000);

    agile_btn_set_event_cb(key_on_btn, BTN_PRESS_UP_EVENT, key_on_click_handle);
    agile_btn_set_event_cb(key_off_btn, BTN_PRESS_UP_EVENT, key_off_click_handle);
    agile_btn_set_event_cb(key_on_btn, BTN_HOLD_EVENT, key_on_long_hold_handle);
    agile_btn_set_event_cb(key_on_btn, BTN_HOLD_FREE_EVENT, key_on_long_free_handle);
    agile_btn_set_event_cb(key_off_btn, BTN_HOLD_EVENT, key_off_long_hold_handle);
    agile_btn_set_event_cb(key_off_btn, BTN_HOLD_FREE_EVENT, key_off_long_free_handle);

    agile_btn_set_event_cb(water_leak_btn, BTN_PRESS_UP_EVENT, water_leak_up_callback);
    agile_btn_set_event_cb(water_leak_btn, BTN_PRESS_DOWN_EVENT, water_leak_down_callback);
    agile_btn_set_event_cb(water_lost_btn, BTN_PRESS_UP_EVENT, water_lost_plugin_callback);
    agile_btn_set_event_cb(water_lost_btn, BTN_PRESS_DOWN_EVENT, water_lost_plugout_callback);

    agile_btn_start(key_on_btn);
    agile_btn_start(key_off_btn);

    agile_btn_start(water_leak_btn);
    agile_btn_start(water_lost_btn);
}
