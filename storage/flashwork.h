#ifndef __FLASHWORK_H__
#define __FLASHWORK_H__

#include <stdint.h>

typedef struct{
    int snr;
    int rssi;
    uint8_t slot;
    uint8_t rssi_level;
    uint8_t type;
    uint8_t battery;
    uint8_t waterleak;
    uint8_t online;
    uint8_t ack;
    uint8_t recv;
    uint8_t rsvd;
    uint32_t device_id;
    rt_slist_t slist;
}aqualarm_device_t;

void aq_device_online_set(uint32_t device_id,uint8_t state);

#endif

