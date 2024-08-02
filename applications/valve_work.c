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

static uint8_t valve_status = 0;
static uint8_t lock_status = 0;
static uint8_t valve_valid = 1;
static uint8_t internal_valve_warning = 0;

#define VALVE_STATUS_CLOSE   0
#define VALVE_STATUS_OPEN    1

rt_timer_t valve_open_timer = RT_NULL;
rt_timer_t valve_open_once_timer = RT_NULL;
rt_timer_t valve_close_timer = RT_NULL;
rt_timer_t valve_detect_timer = RT_NULL;
rt_timer_t valve_internal_first_check_timer = RT_NULL;
rt_timer_t valve_internal_final_check_timer = RT_NULL;
rt_timer_t valve_external_check_timer = RT_NULL;

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
    if(lock_status == 1 || valve_valid == 0)
    {
        led_valve_fail();
        return RT_ERROR;
    }

    if(pd_chip_lock_voltage_get() < 3)
    {
        led_valve_fail();
        return RT_ERROR;
    }

    DeviceStatus = ValveOpen;
    led_valve_on();
    beep_once();
    pd_valve_control(1);
    valve_turn_control(1);

    rt_timer_stop(valve_close_timer);
    rt_timer_stop(valve_open_once_timer);
    rt_timer_stop(valve_external_check_timer);
    rt_timer_stop(valve_internal_first_check_timer);
    rt_timer_stop(valve_internal_final_check_timer);
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

    rt_timer_stop(valve_open_timer);
    rt_timer_stop(valve_open_once_timer);
    rt_timer_stop(valve_detect_timer);
    rt_timer_stop(valve_external_check_timer);
    rt_timer_stop(valve_internal_first_check_timer);
    rt_timer_stop(valve_internal_final_check_timer);
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

void valve_check(void)
{
    if(valve_status == VALVE_STATUS_OPEN && rt_pin_read(MOTO_OPEN_STATUS_PIN) == 0)
    {
        valve_turn_control(-1);
        pd_valve_check();
        rt_timer_stop(valve_open_timer);
        rt_timer_stop(valve_open_once_timer);
        rt_timer_stop(valve_close_timer);
        rt_timer_stop(valve_detect_timer);
        rt_timer_stop(valve_internal_final_check_timer);
        rt_timer_start(valve_internal_first_check_timer);
        rt_timer_start(valve_external_check_timer);
    }
}

void valve_internal_first_check_timer_callback(void *parameter)
{
    valve_turn_control(1);
    if(rt_pin_read(MOTO_OPEN_STATUS_PIN))
    {
        internal_valve_warning = 0;
        rt_timer_start(valve_internal_final_check_timer);
        LOG_D("valve_internal_first_check_timer_callback success\r\n");
    }
    else
    {
        valve_valid = 0;
        internal_valve_warning = 1;
        warning_enable(InternalValveFailEvent);
        LOG_E("valve_internal_first_check_timer_callback failed\r\n");
    }
}

void valve_internal_final_check_timer_callback(void *parameter)
{
    if(rt_pin_read(MOTO_OPEN_STATUS_PIN) == 0)
    {
        internal_valve_warning = 0;
        gateway_warning_master_valve_fail(1);
        LOG_D("valve_internal_final_check_timer_callback success\r\n");
    }
    else
    {
        valve_valid = 0;
        internal_valve_warning = 1;
        warning_enable(InternalValveFailEvent);
        LOG_E("valve_internal_final_check_timer_callback failed\r\n");
    }
}

void valve_external_check_timer_callback(void *parameter)
{
    if(pd_valve_error_check() == 0)
    {
        if(internal_valve_warning == 0)
        {
            valve_valid = 1;
            valvefail_warning_disable();
            gateway_warning_master_valve_fail(2);
        }
        LOG_D("valve_external_check_timer_callback success\r\n");
    }
    else
    {
        valve_valid = 0;
        warning_enable(ExternalValveFailEvent);
        LOG_E("valve_external_check_timer_callback failed\r\n");
    }
}

void valve_open_timer_callback(void *parameter)
{
    uint8_t internal_valve_result = rt_pin_read(MOTO_OPEN_STATUS_PIN);
    uint8_t external_valve_result = pd_valve_error_open();
    if(internal_valve_result == 0 && external_valve_result == 0)
    {
        valve_valid = 1;
        valvefail_warning_disable();
        LOG_D("valve_open_timer_callback check success\r\n");
    }
    else
    {
        valve_valid = 0;
        if(internal_valve_result)
        {
            warning_enable(InternalValveFailEvent);
        }
        if(external_valve_result)
        {
            warning_enable(ExternalValveFailEvent);
        }
        LOG_E("valve_open_timer_callback check failed,internal_valve_result %d,external_valve_result %d\r\n",internal_valve_result,external_valve_result);
    }
}

void valve_close_timer_callback(void *parameter)
{
    uint8_t internal_valve_result = rt_pin_read(MOTO_CLOSE_STATUS_PIN);
    uint8_t external_valve_result = pd_valve_error_close();
    if(internal_valve_result == 0 && external_valve_result == 0)
    {
        valve_valid = 1;
        valvefail_warning_disable();
        LOG_D("valve_close_timer_callback check success\r\n");
    }
    else
    {
        valve_valid = 0;
        if(internal_valve_result)
        {
            warning_enable(InternalValveFailEvent);
        }
        if(external_valve_result)
        {
            warning_enable(ExternalValveFailEvent);
        }
        LOG_E("valve_close_timer_callback check failed,internal_valve_result %d,external_valve_result %d\r\n",internal_valve_result,external_valve_result);
    }
}

void valve_detect_timer_callback(void *parameter)
{
    valve_check();
}

void valve_open_once_timer_callback(void *parameter)
{
    valve_open();
}

void valve_init(void)
{
    lock_status = flash_get_key("valve_lock");

    rt_pin_mode(MOTO_CONTROL_PIN,PIN_MODE_OUTPUT);
    rt_pin_mode(MOTO_CLOSE_STATUS_PIN,PIN_MODE_INPUT);
    rt_pin_mode(MOTO_OPEN_STATUS_PIN,PIN_MODE_INPUT);

    valve_internal_first_check_timer = rt_timer_create("valve_check_1", valve_internal_first_check_timer_callback, RT_NULL, 5000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_internal_final_check_timer = rt_timer_create("valve_check_2", valve_internal_final_check_timer_callback, RT_NULL, 8000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_external_check_timer = rt_timer_create("valve_external_check", valve_external_check_timer_callback, RT_NULL, 25000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_open_timer = rt_timer_create("valve_open", valve_open_timer_callback, RT_NULL, 30000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_close_timer = rt_timer_create("valve_close", valve_close_timer_callback, RT_NULL, 30000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_detect_timer  = rt_timer_create("valve_detect", valve_detect_timer_callback, RT_NULL, 60*1000*5, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
    valve_open_once_timer = rt_timer_create("valve_open_once", valve_open_once_timer_callback, RT_NULL, 2*1000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);

    if(aq_device_waterleak_find())
    {
        warning_enable(SlaverSensorLeakEvent);
    }
    else
    {
        rt_timer_start(valve_open_once_timer);
    }
}
