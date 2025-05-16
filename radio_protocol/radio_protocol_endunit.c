/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-20     Rick       the first version
 */
#include "rtthread.h"
#include "flashwork.h"
#include "water_work.h"
#include "radio_protocol.h"
#include "radio_protocol_endunit.h"

#define DBG_TAG "RADIO_PROTOCOL_ENDUNIT"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern uint8_t allow_add_device;
extern enum Device_Status DeviceStatus;
extern WariningEvent SlaverSensorLostEvent;
extern WariningEvent SlaverSensorLeakEvent;
extern WariningEvent SlaverLowPowerEvent;
extern WariningEvent SlaverUltraLowPowerEvent;

static void radio_frame_endunit_parse_heart(rx_format *rx_frame,aqualarm_device_t *device)
{
    tx_format tx_frame = {0};
    uint8_t battery = rx_frame->rx_data[2];
    uint8_t warn_status = rx_frame->rx_data[3];

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_DOWNLINK;
    tx_frame.dest_addr = rx_frame->source_addr;
    tx_frame.source_addr = get_local_address();
    tx_frame.command = HEART_UPLOAD_CMD;
    radio_endunit_command_send(&tx_frame);

    aq_device_battery_set(device->device_id,battery);

    if(device->battery == 1)
    {
        warning_enable(SlaverLowPowerEvent);
    }
    else if(device->battery == 2)
    {
        warning_enable(SlaverUltraLowPowerEvent);
    }

    gateway_warning_endunit_heart(device->device_id,battery,warn_status,device->rssi_level);
}

static void radio_frame_endunit_parse_learn(rx_format *rx_frame,aqualarm_device_t *device)
{
    uint8_t sub_command = 0;
    tx_format tx_frame = {0};

    if(rx_frame->dest_addr == 0xFFFFFFFF && allow_add_device == 1)
    {
        sub_command = 0;
        tx_frame.msg_ack = RT_TRUE;
        tx_frame.msg_type = MSG_UNCONFIRMED_DOWNLINK;
        tx_frame.dest_addr = rx_frame->source_addr;
        tx_frame.source_addr = get_local_address();
        tx_frame.command = LEARN_DEVICE_CMD;
        tx_frame.tx_data = &sub_command;
        tx_frame.tx_len = 1;
        radio_endunit_command_send(&tx_frame);
    }
    else if(rx_frame->dest_addr == get_local_address())
    {
        sub_command = rx_frame->rx_data[2];
        tx_frame.msg_ack = RT_TRUE;
        tx_frame.msg_type = MSG_UNCONFIRMED_DOWNLINK;
        tx_frame.dest_addr = rx_frame->source_addr;
        tx_frame.source_addr = get_local_address();
        tx_frame.command = LEARN_DEVICE_CMD;
        tx_frame.tx_data = &sub_command;
        tx_frame.tx_len = 1;
        radio_endunit_command_send(&tx_frame);

        switch(sub_command)
        {
        case 1://add device
            if(aq_device_create(rx_frame->rssi_level,DEVICE_TYPE_ENDUNIT,rx_frame->source_addr) == RT_NULL)
            {
                LOG_I("aq_device_create failed %d\r\n",rx_frame->source_addr);
                learn_fail_ring();
                return;
            }
            beep_once();    //响一声
            radio_refresh_learn_device();
            gateway_sync_device_add(DEVICE_TYPE_ENDUNIT,rx_frame->source_addr,rx_frame->rssi_level);
            break;
        case 2://start learn
            radio_start_learn_device();
            break;
        default:
            break;
        }
    }
}

static void radio_frame_endunit_parse_valve(rx_format *rx_frame,aqualarm_device_t *device)
{
    uint32_t value = rx_frame->rx_data[2];
    device->rssi_level = rx_frame->rssi_level;

    tx_format tx_frame = {0};
    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_DOWNLINK;
    tx_frame.dest_addr = rx_frame->source_addr;
    tx_frame.source_addr = get_local_address();
    tx_frame.command = CONTROL_VALVE_CMD;
    tx_frame.tx_data = &value;
    tx_frame.tx_len = 1;
    radio_endunit_command_send(&tx_frame);

    if(value)
    {
        if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen)
        {
            valve_unlock();
            valve_open();
            gateway_control_slaver_control(rx_frame->source_addr,1,rx_frame->rssi_level);
        }
    }
    else
    {
        aq_device_waterleak_set(device->device_id,0);
        if(DeviceStatus == SlaverSensorLeak)
        {
            if(aq_device_waterleak_find() == 0)
            {
                warning_all_clear();
            }
        }
        else if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen)
        {
            valve_lock();
            valve_close();
            gateway_control_slaver_control(rx_frame->source_addr,0,rx_frame->rssi_level);
        }
        else if(DeviceStatus == SlaverOffline)
        {
            gateway_control_slaver_control(rx_frame->source_addr,0,rx_frame->rssi_level);
        }
    }
}

static void radio_frame_endunit_parse_warn(rx_format *rx_frame,aqualarm_device_t *device)
{
    uint8_t send_buf[2] = {0};

    send_buf[0] = rx_frame->rx_data[2];
    send_buf[1] = rx_frame->rx_data[3];

    tx_format tx_frame = {0};
    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_DOWNLINK;
    tx_frame.dest_addr = rx_frame->source_addr;
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 2;
    radio_endunit_command_send(&tx_frame);

    switch(rx_frame->rx_data[2])//sub command
    {
    case 0://leak
        aq_device_waterleak_set(device->device_id,rx_frame->rx_data[3]);
        if(rx_frame->rx_data[3])
        {
            warning_enable(SlaverSensorLeakEvent);
        }
        else
        {
            if(DeviceStatus == SlaverSensorLeak)
            {
                if(aq_device_waterleak_find() == 0)
                {
                    warning_all_clear();
                    valve_close();
                }
            }
        }
        gateway_warning_slaver_leak(device->device_id,rx_frame->rx_data[3],device->rssi_level);
        break;
    case 1://lost
        gateway_warning_slaver_lost(device->device_id,rx_frame->rx_data[3],device->rssi_level);
        break;
    default:
        break;
    }
}

void radio_frame_endunit_parse(rx_format *rx_frame)
{
    aqualarm_device_t *device = aq_device_find(rx_frame->source_addr);

    if(rx_frame->rx_data[0] == LEARN_DEVICE_CMD)//learn device ignore address check
    {
        radio_frame_endunit_parse_learn(rx_frame,RT_NULL);
    }

    if(device == RT_NULL)
    {
        return;
    }

    if((rx_frame->dest_addr != get_local_address()))
    {
        return;
    }

    uint8_t command = rx_frame->rx_data[0];
    switch(command)
    {
    case HEART_UPLOAD_CMD:
        radio_frame_endunit_parse_heart(rx_frame,device);
        break;
    case CONTROL_VALVE_CMD:
        radio_frame_endunit_parse_valve(rx_frame,device);
        break;
    case WARNING_UPLOAD_CMD:
        radio_frame_endunit_parse_warn(rx_frame,device);
        break;
    default:
        break;
    }

    aq_device_heart_recv(rx_frame);
    warning_offline_check();
}

//transmit
void radio_endunit_command_send(tx_format *tx_frame)
{
    unsigned short send_len = 0;

    send_len = set_lora_tx_byte(send_len,0xEF);
    send_len = set_lora_tx_byte(send_len,(NET_REGION_SELECT << 4) | NETWORK_VERSION);
    send_len = set_lora_tx_byte(send_len,(tx_frame->msg_ack << 7) | (DEVICE_TYPE_SELECT << 3) | tx_frame->msg_type);
    send_len = set_lora_tx_word(send_len,tx_frame->dest_addr);
    send_len = set_lora_tx_word(send_len,tx_frame->source_addr);
    send_len = set_lora_tx_byte(send_len,tx_frame->command);
    send_len = set_lora_tx_byte(send_len,tx_frame->tx_len);
    send_len = set_lora_tx_buffer(send_len,tx_frame->tx_data,tx_frame->tx_len);
    send_len = set_lora_tx_crc(send_len);
    lora_tx_enqueue(get_lora_tx_buf(),send_len,tx_frame->msg_type == MSG_CONFIRMED_UPLINK ? 1 : 0, 1);
}
