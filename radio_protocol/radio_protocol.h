/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-10     Rick       the first version
 */
#ifndef RADIO_PROTOCOL_RADIO_PROTOCOL_H_
#define RADIO_PROTOCOL_RADIO_PROTOCOL_H_

typedef struct
{
    int16_t rssi;
    int8_t snr;
    uint8_t rssi_level;
    uint8_t msg_type;
    uint8_t calc_crc;
    uint8_t src_crc;
    uint8_t device_type;
    uint32_t dest_addr;
    uint32_t source_addr;
    uint8_t *rx_data;
    uint32_t rx_len;
}rx_format;

typedef struct
{
    uint8_t msg_ack;
    uint8_t msg_type;
    uint32_t dest_addr;
    uint32_t source_addr;
    uint8_t command;
    uint8_t parameter;
    uint8_t *tx_data;
    uint32_t tx_len;
}tx_format;

#define NETWORK_VERSION               0x01

#define FRAME_START                   0xEF

#define MSG_UNCONFIRMED_UPLINK        0x00
#define MSG_CONFIRMED_UPLINK          0x01
#define MSG_UNCONFIRMED_DOWNLINK      0x02
#define MSG_CONFIRMED_DOWNLINK        0x03

#define DEVICE_TYPE_ENDUNIT           0x00
#define DEVICE_TYPE_MAINUNIT          0x01
#define DEVICE_TYPE_GATEWAY           0x02
#define DEVICE_TYPE_ALLINONE          0x03
#define DEVICE_TYPE_DOORUNIT          0x04
#define DEVICE_TYPE_PWRSTOP           0x05
#define DEVICE_TYPE_MOTION_SENSOR     0x06
#define DEVICE_TYPE_FACTORY_TOOL      0x0E
#define DEVICE_TYPE_EXTENDED_DEVICES  0x0F

#define NETID_TEST_ENV                0x01
#define NETID_CHN_ENV                 0x02
#define NETID_EUR_ENV                 0x03
#define NETID_USA_ENV                 0x04

#endif /* RADIO_PROTOCOL_RADIO_PROTOCOL_H_ */
