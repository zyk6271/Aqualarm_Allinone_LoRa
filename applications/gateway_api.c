/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-22     Rick       the first version
 */
#include "rtthread.h"
#include "flashwork.h"
#include "water_work.h"
#include "radio_protocol.h"
#include "radio_protocol_gateway.h"

void gateway_warning_master_leak(uint8_t value)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 1;//master leak
    send_buf[1] = value;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 2;
    radio_gateway_command_send(&tx_frame);
}

void gateway_warning_master_lost(uint8_t value)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 2;//master lost
    send_buf[1] = value;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 2;
    radio_gateway_command_send(&tx_frame);
}

void gateway_warning_master_valve_check(uint8_t value)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 3;//master valve fail
    send_buf[1] = value;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 2;
    radio_gateway_command_send(&tx_frame);
}

void gateway_warning_master_low_temp(uint8_t value)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 4;//master low temp
    send_buf[1] = value;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 2;
    radio_gateway_command_send(&tx_frame);
}

void gateway_warning_slaver_offline(void)
{
    tx_format tx_frame = {0};

    uint8_t *send_buf = rt_malloc(255);
    if(send_buf == RT_NULL)
    {
        return;
    }

    uint32_t length = aq_device_offline_upload(&send_buf[2]);

    send_buf[0] = 5;//slaver heart
    send_buf[1] = length;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = (length * 4) + 2;
    radio_gateway_command_send(&tx_frame);

    rt_free(send_buf);
}

void gateway_warning_endunit_heart(uint32_t slaver_addr,uint8_t battery_level,uint8_t warn_status,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 6;//slaver low bat
    send_buf[1] = (slaver_addr>>24) & 0xFF;
    send_buf[2] = (slaver_addr>>16) & 0xFF;
    send_buf[3] = (slaver_addr>>8) & 0xFF;
    send_buf[4] = slaver_addr & 0xFF;
    send_buf[5] = battery_level;
    send_buf[6] = warn_status;
    send_buf[7] = rssi_level;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 8;
    radio_gateway_command_send(&tx_frame);
}

void gateway_warning_slaver_leak(uint32_t slaver_addr,uint8_t leak_level,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 7;//slaver leak
    send_buf[1] = (slaver_addr>>24) & 0xFF;
    send_buf[2] = (slaver_addr>>16) & 0xFF;
    send_buf[3] = (slaver_addr>>8) & 0xFF;
    send_buf[4] = slaver_addr & 0xFF;
    send_buf[5] = leak_level;
    send_buf[6] = rssi_level;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_warning_slaver_lost(uint32_t slaver_addr,uint8_t lost_level,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 8;//slaver lost
    send_buf[1] = (slaver_addr>>24) & 0xFF;
    send_buf[2] = (slaver_addr>>16) & 0xFF;
    send_buf[3] = (slaver_addr>>8) & 0xFF;
    send_buf[4] = slaver_addr & 0xFF;
    send_buf[5] = lost_level;
    send_buf[6] = rssi_level;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = WARNING_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_control_master_control(uint8_t value)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 0;//master control
    send_buf[1] = value;

    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = CONTROL_VALVE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 2;
    radio_gateway_command_send(&tx_frame);
}

void gateway_control_slaver_control(uint32_t device_addr,uint8_t value,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 1;//slaver control
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = value;
    send_buf[6] = rssi_level;

    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = CONTROL_VALVE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_control_door_control(uint32_t device_addr,uint8_t value,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 2;//door control
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = value;
    send_buf[6] = rssi_level;

    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = CONTROL_VALVE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_control_door_delay(uint32_t device_addr,uint8_t value,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 3;//door delay
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = value;
    send_buf[6] = rssi_level;

    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = CONTROL_VALVE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_control_motion_sensor_control(uint32_t device_addr,uint8_t rssi_level,uint8_t value,uint8_t range_time_level,uint8_t delay_time_level,uint8_t human_detected)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[16] = {0};

    send_buf[0] = 4;//slaver control
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = value;
    send_buf[6] = rssi_level;
    send_buf[7] = range_time_level;
    send_buf[8] = delay_time_level;
    send_buf[9] = human_detected;

    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = CONTROL_VALVE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 10;
    radio_gateway_command_send(&tx_frame);
}

void gateway_learn_broacast_confirm(uint32_t dst_addr)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 0;//power on heart
    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = dst_addr;
    tx_frame.source_addr = get_local_address();
    tx_frame.command = LEARN_DEVICE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 1;
    radio_gateway_command_send(&tx_frame);
}

void gateway_learn_add_upload(uint32_t dst_addr)
{
    extern const uint8_t fw_main_ver;
    extern const uint8_t fw_sub_ver;

    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 1;//power on heart
    send_buf[1] = fw_main_ver;//fw_main_ver
    send_buf[2] = fw_sub_ver;//fw_sub_ver

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = dst_addr;
    tx_frame.source_addr = get_local_address();
    tx_frame.command = LEARN_DEVICE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 3;
    radio_gateway_command_send(&tx_frame);
}

void gateway_heart_upload_poweron(void)
{
    extern const uint8_t fw_main_ver;
    extern const uint8_t fw_sub_ver;

    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 0;//power on heart
    send_buf[1] = warning_status_get();//warning status
    send_buf[2] = get_valve_status();//valve status
    send_buf[3] = fw_main_ver;//fw_main_ver
    send_buf[4] = fw_sub_ver;//fw_sub_ver
    send_buf[5] = aq_device_slaver_count();//slave_count

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = HEART_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 6;
    radio_gateway_command_send(&tx_frame);
}

void gateway_heart_upload_master(void)
{
    extern const uint8_t fw_main_ver;
    extern const uint8_t fw_sub_ver;

    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 1;//master heart
    send_buf[1] = warning_status_get();//warning status
    send_buf[2] = get_valve_status();//valve status
    send_buf[3] = fw_main_ver;//fw_main_ver
    send_buf[4] = fw_sub_ver;//fw_sub_ver

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = HEART_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 5;
    radio_gateway_command_send(&tx_frame);
}

void gateway_heart_doorunit_upload(uint32_t device_addr,uint8_t rssi_level,uint8_t bat_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 2;//slaver heart
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = rssi_level;
    send_buf[6] = bat_level;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = HEART_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_heart_motion_sensor_upload(uint32_t device_addr,uint8_t rssi_level,uint8_t range_time_level,uint8_t delay_time_level,uint8_t human_detected)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[16] = {0};

    send_buf[0] = 3;//motion sensor heart
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = rssi_level;
    send_buf[6] = range_time_level;
    send_buf[7] = delay_time_level;
    send_buf[8] = human_detected;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = HEART_UPLOAD_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 9;
    radio_gateway_command_send(&tx_frame);
}

void gateway_sync_device_add(uint8_t device_type,uint32_t device_addr,uint8_t rssi_level)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 0;//slaver add
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;
    send_buf[5] = rssi_level;
    send_buf[6] = device_type;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = DEVICE_SYNC_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 7;
    radio_gateway_command_send(&tx_frame);
}

void gateway_sync_device_del(uint32_t device_addr)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 1;//slaver del
    send_buf[1] = (device_addr>>24) & 0xFF;
    send_buf[2] = (device_addr>>16) & 0xFF;
    send_buf[3] = (device_addr>>8) & 0xFF;
    send_buf[4] = device_addr & 0xFF;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = DEVICE_SYNC_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 5;
    radio_gateway_command_send(&tx_frame);
}

void gateway_sync_device_reset(void)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 2;//device reset

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = DEVICE_SYNC_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 1;
    radio_gateway_command_send(&tx_frame);
}

void gateway_sync_device_upload(void)
{
    tx_format tx_frame = {0};

    uint8_t *send_buf = rt_malloc(255);
    if(send_buf == RT_NULL)
    {
        return;
    }

    uint32_t length = 0;
    length = aq_device_clone_upload(&send_buf[4]);
    send_buf[0] = 3;//device sync
    send_buf[1] = warning_status_get();//warning status
    send_buf[2] = get_valve_status();//valve status
    send_buf[3] = length;//warning status

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_CONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = DEVICE_SYNC_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = (length * 8) + 4;
    radio_gateway_command_send(&tx_frame);

    rt_free(send_buf);
}

void gateway_ota_resp_start(void)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 1;//sub_command:start

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = FIRMWARE_UPDATE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 1;
    radio_gateway_command_send(&tx_frame);
}

void gateway_ota_ack_transfer(void)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 2;

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = FIRMWARE_UPDATE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 1;
    radio_gateway_command_send(&tx_frame);
}

void gateway_ota_resp_end(void)
{
    tx_format tx_frame = {0};
    uint8_t send_buf[8] = {0};

    send_buf[0] = 3;//sub_command:end

    tx_frame.msg_ack = RT_TRUE;
    tx_frame.msg_type = MSG_UNCONFIRMED_UPLINK;
    tx_frame.dest_addr = aq_gateway_find();
    tx_frame.source_addr = get_local_address();
    tx_frame.command = FIRMWARE_UPDATE_CMD;
    tx_frame.tx_data = send_buf;
    tx_frame.tx_len = 1;
    radio_gateway_command_send(&tx_frame);
}
