/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-20     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "radio.h"
#include "radio_app.h"
#include "radio_driver.h"
#include "radio_encoder.h"

#define DBG_TAG "RADIO_CSMA"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define CSMA_NOISE_THRESHOLD_IN_DBM (-75)

static uint8_t cad_detected = 0;
static struct rt_completion cad_done_sem;

void radio_cad_detected(uint8_t channelActivityDetected)
{
    cad_detected = channelActivityDetected;
    rt_completion_done(&cad_done_sem);
}

rt_err_t csma_check_start(uint32_t send_freq)
{
    rt_err_t ret = RT_EOK;
    int16_t rssi =  0;
    uint32_t carrierSenseTime = 0;

    rt_completion_init(&cad_done_sem);

    Radio.Standby();
    Radio.SetChannel( send_freq );
    SUBGRF_SetCadParams(LORA_CAD_02_SYMBOL, 22, 10, LORA_CAD_ONLY, 0);
    Radio.StartCad();

    if(rt_completion_wait(&cad_done_sem, 100) != RT_EOK)
    {
        LOG_E("CAD fail,no irq signal,exit csma");
        ret = RT_ERROR;
        goto __exit;
    }

    if(cad_detected)
    {
        LOG_E("CAD detected valid frame,exit csma");
        ret = RT_ERROR;
        goto __exit;
    }

    Radio.Standby();
    Radio.SetModem( MODEM_FSK );
    Radio.SetRxConfig( MODEM_FSK, 200000, 600, 0, 200000, 3, 0, false,
                      0, false, 0, 0, false, true );
    Radio.Rx(0);
    rt_thread_mdelay(3);

    carrierSenseTime = rt_tick_get();
    while( rt_tick_get() - carrierSenseTime < 5 )
    {
        rssi = Radio.Rssi( MODEM_FSK );
        if( rssi > CSMA_NOISE_THRESHOLD_IN_DBM && rssi != 0)
        {
            LOG_E("RSSI detected too high %d",rssi);
            ret = RT_ERROR;
            goto __exit;
        }
    }
    Radio.Standby();
    Radio.SetTxConfig( MODEM_LORA, LORA_TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE,
                                   true, 0, 0, LORA_IQ_INVERSION_ON_DISABLE, 5000 );

    return ret;

__exit:
    Radio.SetChannel( RF_RX_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON_DISABLE, true );

    Radio.SetMaxPayloadLength(MODEM_LORA, 255);
    Radio.Rx(0);

    return ret;
}
