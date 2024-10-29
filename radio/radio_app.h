/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-15     Rick       the first version
 */
#ifndef RADIO_RADIO_APP_H_
#define RADIO_RADIO_APP_H_

/*
 * LoRa Config
 */
#define TX_RX_FREQUENCE_OFFSET                      0            // 0
#define LORA_TX_OUTPUT_POWER                        10          //14dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON_DISABLE          false
#define LORA_IQ_INVERSION_ON_DISABLE                true
/*
 * Radio Config
 */
#define RF_RX_FREQUENCY                             434000000
#define RF_GATEWAY_TX_FREQUENCY                     433500000
#define RF_SLAVER_TX_FREQUENCY                      434500000
#define RF_FACTORY_TX_FREQUENCY                     435500000
#define RF_FACTORY_RX_FREQUENCY                     436000000
#define RX_TIMEOUT_VALUE                            0
#define TX_TIMEOUT_VALUE                            3000

void RF_Init(void);
void RF_Send(char *payload,int size);

#endif /* RADIO_RADIO_APP_H_ */
