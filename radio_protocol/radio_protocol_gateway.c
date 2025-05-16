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
#include "radio_protocol_gateway.h"

#define DBG_TAG "RADIO_PROTOCOL_GATEWAY"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

extern uint8_t allow_add_device;
extern enum Device_Status DeviceStatus;

static void radio_frame_gateway_parse_sync(rx_format *rx_frame)
{
    uint32_t device_addr = 0;
    uint8_t sub_command = rx_frame->rx_data[2];
    switch(sub_command)
    {
    case 0://request sync
        gateway_sync_device_upload();
        break;
    case 1://delete device
        device_addr = rx_frame->rx_data[3]<<24 | rx_frame->rx_data[4]<<16 | rx_frame->rx_data[5]<<8 | rx_frame->rx_data[6];
        aq_device_delete(device_addr);
        break;
    default:
        break;
    }

}
static void radio_frame_gateway_parse_heart(rx_format *rx_frame)
{
    uint8_t sub_command = rx_frame->rx_data[2];
    switch(sub_command)
    {
    case 0://response heart
        LOG_D("radio_frame_gateway_parse_heart %d at %d",rx_frame->source_addr,rt_tick_get());
        break;
    default:
        break;
    }
}

static void radio_frame_gateway_parse_learn(rx_format *rx_frame)
{
    tx_format tx_frame = {0};
    uint8_t sub_command = 0;

    if(rx_frame->dest_addr == 0xFFFFFFFF && allow_add_device == 1)
    {
        gateway_learn_broacast_confirm(rx_frame->source_addr);
    }
    else if(rx_frame->dest_addr == get_local_address())
    {
        sub_command = rx_frame->rx_data[2];
        switch(sub_command)
        {
        case 1://add device
            aq_gateway_delete();
            if(aq_device_create(0,DEVICE_TYPE_GATEWAY,rx_frame->source_addr) == RT_NULL)
            {
                LOG_E("aq_device_create failed %d\r\n",rx_frame->source_addr);
                learn_fail_ring();
            }
            else
            {
                beep_once();
                wifi_led(1);
                radio_refresh_learn_device();
                gateway_learn_add_upload(rx_frame->source_addr);
            }
            break;
        default:
            break;
        }
    }
}

static void radio_frame_gateway_parse_ack(rx_format *rx_frame)
{
    rf_ack_callback();
}

static void radio_frame_gateway_parse_ota(rx_format *rx_frame)
{
    uint8_t pkt_size = 0;
    uint32_t fw_size,fw_offset = 0;
    uint8_t sub_command = rx_frame->rx_data[2];
    switch(sub_command)
    {
    case 1:
        fw_size = rx_frame->rx_data[3] << 24 | rx_frame->rx_data[4] << 16 | \
                    rx_frame->rx_data[5] << 8 | rx_frame->rx_data[6];
        lora_ota_begin(fw_size);
        break;
    case 2:
        fw_offset = rx_frame->rx_data[3] << 24 | rx_frame->rx_data[4] << 16 | \
                        rx_frame->rx_data[5] << 8 | rx_frame->rx_data[6];
        pkt_size = rx_frame->rx_data[7];
        lora_ota_receive(&rx_frame->rx_data[8], fw_offset, pkt_size);
        break;
    case 3:
        lora_ota_end();
        break;
    default:
        break;
    }
}

static void radio_frame_gateway_parse_control(rx_format *rx_frame)
{
    uint8_t sub_command = rx_frame->rx_data[2];
    switch(sub_command)
    {
    case 0://close valve
        if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen)
        {
            valve_close();
            gateway_control_master_control(0);
        }
        break;
    case 1://open valve
        if(DeviceStatus == ValveClose || DeviceStatus == ValveOpen)
        {
            valve_unlock();
            valve_open();
            gateway_control_master_control(1);
        }
        break;
    default:
        break;
    }
}

void radio_frame_gateway_parse(rx_format *rx_frame)
{
    aqualarm_device_t *device = aq_device_find(rx_frame->source_addr);

    if(rx_frame->rx_data[0] == LEARN_DEVICE_CMD)//learn device ignore source address check
    {
        radio_frame_gateway_parse_learn(rx_frame);
    }

    if(device == RT_NULL)
    {
        return;
    }

    if((rx_frame->dest_addr != get_local_address()))
    {
        return;
    }

    wifi_led(1);
    aq_device_recv_set(rx_frame->source_addr,1);

    switch(rx_frame->rx_data[0])//command
    {
    case HEART_UPLOAD_CMD:
        radio_frame_gateway_parse_heart(rx_frame);
        break;
    case CONTROL_VALVE_CMD:
        radio_frame_gateway_parse_control(rx_frame);
        break;
    case DEVICE_SYNC_CMD:
        radio_frame_gateway_parse_sync(rx_frame);
        break;
    case CONFIRM_ACK_CMD:
        radio_frame_gateway_parse_ack(rx_frame);
        break;
    case FIRMWARE_UPDATE_CMD:
        radio_frame_gateway_parse_ota(rx_frame);
        break;
    default:
        break;
    }
}

//transmit
void radio_gateway_command_send(tx_format *tx_frame)
{
    if(tx_frame->dest_addr == 0)
    {
        return;
    }

    wifi_communication_blink();

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
    lora_tx_enqueue(get_lora_tx_buf(),send_len,tx_frame->msg_type == MSG_CONFIRMED_UPLINK ? 1 : 0, 0);
}

