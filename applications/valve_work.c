/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-27     Rick       the first version
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "pin_config.h"
#include "led.h"
#include "flashwork.h"
#include "water_work.h"

#define DBG_TAG "valve"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

uint8_t valve_status = 0;
uint8_t lock_status = 0;
uint8_t valve_valid = 1;
uint8_t valve_check_tick = 0;

uint8_t internal_valve_open_result = 0;     //0:normal,1:warning
uint8_t internal_valve_close_result = 0;    //0:normal,1:warning
uint8_t internal_valve_check_result = 0;    //0:normal,1:warning
uint8_t external_valve_open_result = 0;     //0:normal,1:warning
uint8_t external_valve_close_result = 0;    //0:normal,1:warning
uint8_t external_valve_check_result = 0;    //0:normal,1:warning

#define VALVE_STATUS_CLOSE   0
#define VALVE_STATUS_OPEN    1

rt_timer_t valve_open_timer = RT_NULL;
rt_timer_t valve_open_once_timer = RT_NULL;
rt_timer_t valve_close_timer = RT_NULL;
rt_timer_t valve_detect_timer = RT_NULL;
rt_timer_t valve_check_timer = RT_NULL;
rt_timer_t delay_close_timer = RT_NULL;

extern enum Device_Status DeviceStatus;
extern WariningEvent InternalValveFailEvent;
extern WariningEvent ExternalValveFailEvent;
extern WariningEvent SlaverSensorLeakEvent;

void valve_turn_control(int dir)
{
    if(dir < 0)
    {
        if(rt_pin_read(MOTO_CLOSE_STATUS_PIN))//close switch high
        {
            radio_csma_pause_set();
        }
        valve_status = VALVE_STATUS_CLOSE;
        rt_pin_write(MOTO_CONTROL_PIN,PIN_LOW);
    }
    else
    {
        if(rt_pin_read(MOTO_OPEN_STATUS_PIN))//open switch high
        {
            radio_csma_pause_set();
        }
        valve_status = VALVE_STATUS_OPEN;
        rt_pin_write(MOTO_CONTROL_PIN,PIN_HIGH);
    }
}

rt_err_t valve_open(void)
{
    if(lock_status == 1 || valve_valid == 0 || pd_chip_lock_voltage_get() < 3)
    {
        led_valve_fail();
        return RT_ERROR;
    }

    DeviceStatus = ValveOpen;
    led_valve_on();
    beep_once();
    pd_valve_control(1);
    valve_turn_control(1);

    rt_timer_stop(delay_close_timer);
    rt_timer_stop(valve_check_timer);
    rt_timer_stop(valve_close_timer);
    rt_timer_start(valve_open_timer);
    rt_timer_start(valve_detect_timer);

    return RT_EOK;
}

rt_err_t valve_close(void)
{
    if(valve_valid == 0)
    {
        led_valve_fail();
        valve_turn_control(-1);
        pd_valve_control(0);
        return RT_ERROR;
    }

    DeviceStatus = ValveClose;
    led_valve_off();
    beep_key_down();
    pd_valve_control(0);
    valve_turn_control(-1);

    rt_timer_stop(delay_close_timer);
    rt_timer_stop(valve_check_timer);
    rt_timer_stop(valve_open_timer);
    rt_timer_stop(valve_detect_timer);
    rt_timer_start(valve_close_timer);

    return RT_EOK;
}

void valve_lock(void)
{
    if(lock_status == 0)
    {
        lock_status = 1;
        flash_set_key("valve_lock",1);
    }
}

void valve_unlock(void)
{
    if(lock_status == 1)
    {
        lock_status = 0;
        flash_set_key("valve_lock",0);
    }
}

uint8_t get_valve_lock(void)
{
    return lock_status;
}

uint8_t get_valve_status(void)
{
    return valve_status;
}

void valve_check_start(void)
{
    valve_check_tick = 0;
    rt_timer_stop(valve_open_timer);
    rt_timer_stop(valve_close_timer);
    rt_timer_stop(valve_detect_timer);
    rt_timer_start(valve_check_timer);
}

void valve_open_timer_callback(void *parameter)
{
    uint8_t internal_valve_open_status = rt_pin_read(MOTO_OPEN_STATUS_PIN);
    uint8_t external_valve_open_status = pd_valve_error_open();

    if(internal_valve_open_status == 0)
    {
        valve_valid = 1;
        internal_valve_open_result = 0;
        valvefail_warning_disable();
        LOG_D("valve_open_timer_callback internal_valve_check success");
    }
    else
    {
        valve_valid = 0;
        internal_valve_open_result = 1;
        warning_enable(InternalValveFailEvent);
        LOG_E("valve_open_timer_callback internal_valve_check failed");
    }

    if(external_valve_open_status == 0)
    {
        external_valve_open_result = 0;
        LOG_D("valve_open_timer_callback external_valve_check success\r\n");
    }
    else
    {
        external_valve_open_result = 1;
        gateway_warning_master_valve_check(4);
        LOG_E("valve_open_timer_callback external_valve_check failed");
    }
}

void valve_close_timer_callback(void *parameter)
{
    uint8_t internal_valve_close_status = rt_pin_read(MOTO_CLOSE_STATUS_PIN);
    uint8_t external_valve_close_status = pd_valve_error_close();

    if(internal_valve_close_status == 0)
    {
        valve_valid = 1;
        internal_valve_close_result = 0;
        valvefail_warning_disable();
        LOG_D("valve_close_timer_callback internal_valve_check success");
    }
    else
    {
        valve_valid = 0;
        internal_valve_close_result = 1;
        warning_enable(InternalValveFailEvent);
        LOG_E("valve_close_timer_callback internal_valve_check failed");
    }

    if(external_valve_close_status == 0)
    {
        external_valve_close_result = 0;
        LOG_D("valve_close_timer_callback external_valve_check success\r\n");
    }
    else
    {
        external_valve_close_result = 1;
        gateway_warning_master_valve_check(4);
        LOG_E("valve_close_timer_callback external_valve_check failed");
    }
}

void valve_detect_timer_callback(void *parameter)
{
    valve_check_start();
}

void valve_open_once_timer_callback(void *parameter)
{
    if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen || DeviceStatus == MasterSensorLost)
    {
        valve_open();
    }
}

void delay_close_timer_callback(void *parameter)
{
    if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen || DeviceStatus == MasterSensorLost)
    {
        valve_lock();
        valve_close();
        gateway_control_master_control(0);
    }
}

void valve_check_timer_callback(void *parameter)
{
    switch(valve_check_tick++)
    {
    case 0://start turn
        if(valve_status == VALVE_STATUS_OPEN)
        {
            rt_pin_write(MOTO_CONTROL_PIN,PIN_HIGH);
            pd_valve_check();
        }
        else
        {
            rt_timer_stop(valve_check_timer);
        }
        break;
    case 15://check start and turn back
        if(rt_pin_read(MOTO_OPEN_STATUS_PIN) == 0)
        {
            rt_pin_write(MOTO_CONTROL_PIN,PIN_LOW);
        }
        else
        {
            valve_valid = 0;
            internal_valve_check_result = 1;
            rt_timer_stop(valve_check_timer);
            warning_enable(InternalValveFailEvent);
        }
        break;
    case 20://check back and turn forward
        if(rt_pin_read(MOTO_OPEN_STATUS_PIN) == 1)
        {
            rt_pin_write(MOTO_CONTROL_PIN,PIN_HIGH);
        }
        else
        {
            valve_valid = 0;
            internal_valve_check_result = 1;
            rt_timer_stop(valve_check_timer);
            warning_enable(InternalValveFailEvent);
            rt_pin_write(MOTO_CONTROL_PIN,PIN_HIGH);
        }
        break;
    case 28://check forward
        if(rt_pin_read(MOTO_OPEN_STATUS_PIN) == 0)
        {
            valve_valid = 1;
            internal_valve_check_result = 0;
            valvefail_warning_disable();
            gateway_warning_master_valve_check(1);
        }
        else
        {
            valve_valid = 0;
            internal_valve_check_result = 1;
            rt_timer_stop(valve_check_timer);
            warning_enable(InternalValveFailEvent);
        }
        break;
    case 30://check external,stop all
        rt_timer_stop(valve_check_timer);
        if(pd_valve_error_check() == 0)
        {
            external_valve_check_result = 0;
            gateway_warning_master_valve_check(2);
        }
        else
        {
            external_valve_check_result = 1;
            gateway_warning_master_valve_check(4);
        }
        break;
    default:
        break;
    }
}

void valve_delay_control(uint8_t value)
{
    if(value)
    {
        rt_timer_start(delay_close_timer);
    }
    else
    {
        rt_timer_stop(delay_close_timer);
    }
}

void valve_init(void)
{
    lock_status = flash_get_key("valve_lock");

    rt_pin_mode(MOTO_CONTROL_PIN,PIN_MODE_OUTPUT);
    rt_pin_mode(MOTO_CLOSE_STATUS_PIN,PIN_MODE_INPUT);
    rt_pin_mode(MOTO_OPEN_STATUS_PIN,PIN_MODE_INPUT);

    valve_open_timer = rt_timer_create("valve_open", valve_open_timer_callback, RT_NULL, 30000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_close_timer = rt_timer_create("valve_close", valve_close_timer_callback, RT_NULL, 30000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_detect_timer  = rt_timer_create("valve_detect", valve_detect_timer_callback, RT_NULL, 60*1000*5, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_open_once_timer = rt_timer_create("valve_open_once", valve_open_once_timer_callback, RT_NULL, 2*1000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_check_timer = rt_timer_create("valve_check_tick", valve_check_timer_callback, RT_NULL, 1000, RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER);
    delay_close_timer = rt_timer_create("delay_close_timer", delay_close_timer_callback, RT_NULL, 4*60*60*1000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);

    if(aq_device_waterleak_find())
    {
        warning_enable(SlaverSensorLeakEvent);
    }
    else
    {
        rt_timer_start(valve_open_once_timer);
    }
}
