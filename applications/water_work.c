/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-25     Rick       the first version
 */
#include <rtthread.h>
#include <water_work.h>

#define DBG_TAG "water_work"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

uint8_t allow_add_device = 0;

enum Device_Status DeviceStatus = ValveOpen;

WariningEvent NowEvent;
WariningEvent InternalValveFailEvent;
WariningEvent ExternalValveFailEvent;
WariningEvent SlaverLowPowerEvent;
WariningEvent SlaverUltraLowPowerEvent;
WariningEvent SlaverSensorLeakEvent;
WariningEvent SlaverOfflineEvent;
WariningEvent MasterSensorLostEvent;
WariningEvent MasterSensorLeakEvent;
WariningEvent MasterLowTempEvent;

rt_timer_t radio_learn_timer = RT_NULL;

void radio_learn_timer_callback(void* parameter)
{
    allow_add_device = 0;
    warning_all_clear();
    led_notice_once();
    led_valve_on_resume();
    gateway_connect_start();
}

void warning_enable(WariningEvent event)
{
    if(event.priority >= NowEvent.priority)
    {
        NowEvent.last_id = event.warning_id;
        NowEvent.priority = event.priority;
        if(event.callback!=RT_NULL)
        {
            NowEvent.callback = event.callback;
            NowEvent.callback(RT_NULL);
        }
        LOG_D("Warning_Enable Success,warning id is %d , now priority is %d\r\n",event.warning_id,event.priority);
    }
    else
    {
        LOG_D("Warning_Enable Fail last is %d Now is %d\r\n",NowEvent.priority,event.priority);
    }
}

void warning_event_bind(uint8_t warning_id,uint8_t priority,WariningEvent *event,void (*callback)(void*))
{
    rt_memset(event,0,sizeof(WariningEvent));
    event->warning_id = warning_id;
    event->priority = priority;
    event->callback = callback;
}

void SlaverLowPowerEventCallback(void *parameter)
{
    led_valve_on_pause();
    led_slave_low_start();
}
void SlaverUltraLowPowerEventCallback(void *parameter)
{
    valve_lock();
    valve_close();
    led_slave_low_start();
    DeviceStatus = SlaverUltraLowPower;
}
void SlaverSensorLeakEventCallback(void *parameter)
{
    if(DeviceStatus != SlaverSensorLeak)
    {
        valve_lock();
        valve_close();
        led_water_alarm_start();
        DeviceStatus = SlaverSensorLeak;
    }
}
void MasterSensorLostEventCallback(void *parameter)
{
    led_valve_on_pause();
    led_master_lost_start();
    DeviceStatus = MasterSensorLost;
    gateway_warning_master_lost(1);
}

void MasterStatusChangeToDeAvtive(void)
{
    DeviceStatus = MasterSensorAbnormal;
}

void MasterSensorLeakEventCallback(void *parameter)
{
    valve_close();
    led_warn_off();
    led_water_alarm_start();
    DeviceStatus = MasterSensorLeak;
    gateway_warning_master_leak(1);
}
void MasterLowTempEventCallback(void *parameter)
{
    valve_close();
    led_ntc_alarm();
    DeviceStatus = MasterLowTemp;
}

void SlaverOfflineEventCallback(void *parameter)
{
    if(DeviceStatus != SlaverOffline)
    {
        valve_close();
        DeviceStatus = SlaverOffline;
        led_offline_start();
    }
}

void valvefail_warning_disable(void)
{
    if(DeviceStatus == InternalValveFail || DeviceStatus == ExternalValveFail)
    {
        warning_all_clear();
    }
}
void InternalValveFailEventCallback(void *parameter)
{
    led_valve_on_pause();
    led_moto_fail_start();
    DeviceStatus = InternalValveFail;
    gateway_warning_master_valve_check(3);
}

void ExternalValveFailEventCallback(void *parameter)
{
    led_valve_on_pause();
    led_moto_fail_start();
    DeviceStatus = ExternalValveFail;
    gateway_warning_master_valve_check(4);
}

void warning_init(void)
{
    warning_event_bind(0,0,&NowEvent,RT_NULL);//本地存储器
    warning_event_bind(1,3,&InternalValveFailEvent,InternalValveFailEventCallback);
    warning_event_bind(2,3,&ExternalValveFailEvent,ExternalValveFailEventCallback);
    warning_event_bind(3,4,&SlaverLowPowerEvent,SlaverLowPowerEventCallback);
    warning_event_bind(4,6,&SlaverUltraLowPowerEvent,SlaverUltraLowPowerEventCallback);
    warning_event_bind(5,8,&SlaverSensorLeakEvent,SlaverSensorLeakEventCallback);
    warning_event_bind(6,5,&SlaverOfflineEvent,SlaverOfflineEventCallback);
    warning_event_bind(7,1,&MasterSensorLostEvent,MasterSensorLostEventCallback);
    warning_event_bind(8,7,&MasterSensorLeakEvent,MasterSensorLeakEventCallback);
    warning_event_bind(9,2,&MasterLowTempEvent,MasterLowTempEventCallback);

    radio_learn_timer = rt_timer_create("radio_learn", radio_learn_timer_callback, RT_NULL, 30*1000, RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
}

uint32_t warning_status_get(void)
{
    return DeviceStatus;
}

void warning_all_clear(void)
{
    beep_stop();
    led_warn_off();
    led_valve_on_resume();
    DeviceStatus = get_valve_status();
    rt_memset(&NowEvent, 0, sizeof(WariningEvent));
}

void warning_lost_clear(void)
{
    gateway_warning_master_lost(0);
    if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen || DeviceStatus == MasterSensorLost)
    {
        warning_all_clear();
        led_loss_off();
        led_valve_on_resume();
    }
}

void warning_offline_check(void)
{
    if(DeviceStatus == SlaverOffline)
    {
        if(aq_device_offline_find() == 0)
        {
            warning_all_clear();
        }
    }
}

void factory_water_leak_simulate(void)
{
    warning_enable(MasterSensorLeakEvent);
}

void radio_start_learn_device(void)
{
    if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen)
    {
        led_valve_on_pause();
        DeviceStatus = LearnDevice;
        allow_add_device = 1;
        led_learn_start();
        rt_timer_start(radio_learn_timer);
    }
    else if(DeviceStatus == LearnDevice)
    {
        rt_timer_stop(radio_learn_timer);
        allow_add_device = 0;
        warning_all_clear();
        led_notice_once();
        led_valve_on_resume();
        gateway_connect_start();
    }
    else
    {
        LOG_D("Now in Warining Mode\r\n");
    }
}

void radio_refresh_learn_device(void)
{
    led_relearn();
    rt_timer_start(radio_learn_timer);
}
