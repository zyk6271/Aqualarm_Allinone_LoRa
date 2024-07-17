/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-27     Rick       the first version
 */
#ifndef APPLICATIONS_STATUS_H_
#define APPLICATIONS_STATUS_H_

#include <stdint.h>

enum Device_Status
{
    ValveClose = 0,
    ValveOpen,
    LearnDevice,
    InternalValveFail,
    ExternalValveFail,
    SlaverLowPower,
    SlaverUltraLowPower,
    SlaverSensorLeak,
    SlaverOffline,
    MasterSensorLost,
    MasterSensorLeak,
    MasterSensorAbnormal,
    MasterLowTemp,
};

typedef struct
{
    uint8_t last_id;
    uint8_t warning_id;
    uint8_t priority;
    void (*callback)(void* parameter);
}WariningEvent;


#endif /* APPLICATIONS_STATUS_H_ */
