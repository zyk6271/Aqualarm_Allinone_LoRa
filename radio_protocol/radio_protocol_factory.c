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
#include "radio_protocol.h"
#include "radio_protocol_factory.h"

#define DBG_TAG "RADIO_PROTOCOL_FACTORY"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

int16_t factory_controller_recv_rssi = 0;
int8_t factory_controller_recv_snr = 0;
int16_t factory_gateway_recv_rssi = 0;
int8_t factory_gateway_recv_snr = 0;

int16_t factory_controller_recv_rssi_read(void)
{
    return factory_controller_recv_rssi;
}

int8_t factory_controller_recv_snr_read(void)
{
    return factory_controller_recv_snr;
}

int16_t factory_gateway_recv_rssi_read(void)
{
    return factory_gateway_recv_rssi;
}

int8_t factory_gateway_recv_snr_read(void)
{
    return factory_gateway_recv_snr;
}

static void radio_frame_factory_parse_heart(rx_format *rx_frame)
{
    factory_controller_recv_rssi = rx_frame->rssi;
    factory_controller_recv_snr = rx_frame->snr;
    factory_gateway_recv_rssi = rx_frame->rx_data[2] << 8 | rx_frame->rx_data[3];
    factory_gateway_recv_snr = rx_frame->rx_data[4];

    LOG_I("factory test frame received,recv rssi %d,recv snr %d\r\n",factory_controller_recv_rssi,factory_controller_recv_snr);
    LOG_I("radio_frame_factory_parse_heart gateway rssi %d,gateway snr %d\r\n",factory_gateway_recv_rssi,factory_gateway_recv_snr);
}

void radio_frame_factory_parse(rx_format *rx_frame)
{
    if((rx_frame->dest_addr != get_local_address()) || (rx_frame->source_addr != 99999999))
    {
        return;
    }

    switch(rx_frame->rx_data[0])
    {
    case FACTORY_RF_TEST_CMD:
        radio_frame_factory_parse_heart(rx_frame);
        break;
    default:
        break;
    }
}

void radio_factory_command_send(tx_format *tx_frame)
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
    lora_tx_enqueue(get_lora_tx_buf(),send_len,tx_frame->msg_type == MSG_CONFIRMED_UPLINK ? 1 : 0, 2);
}

void radio_factory_test_packet_send(void)
{
    tx_format tx_frame = {0};
    tx_frame.msg_ack = RT_FALSE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = 99999999;
    tx_frame.source_addr = get_local_address();
    tx_frame.command = FACTORY_RF_TEST_CMD;
    radio_factory_command_send(&tx_frame);
}
