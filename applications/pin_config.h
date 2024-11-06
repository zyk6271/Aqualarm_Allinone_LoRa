/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-06     Rick       the first version
 */

#ifndef APPLICATIONS_PIN_CONFIG_H_
#define APPLICATIONS_PIN_CONFIG_H_
/*
 * RF
 */
#define RF_SW1_PIN                      0
#define RF_SW2_PIN                      1

/*
 * SENSOR
 */
#define SENSOR_LEAK_PIN                 18
#define SENSOR_LOST_PIN                 10

/*
 * MOTO
 */
#define MOTO_CONTROL_PIN                28
#define MOTO_CLOSE_STATUS_PIN           5
#define MOTO_OPEN_STATUS_PIN            4

/*
 * KEY
 */
#define KEY_ON_PIN                      11
#define KEY_OFF_PIN                     12

/*
 * BUZZER
 */
#define BEEP_PIN                        24

/*
 * PD CONTROLLER
 */
#define PD_CHIP_IRQ1_PIN                45
#define PD_CHIP_IRQ2_PIN                3

#endif /* APPLICATIONS_PIN_CONFIG_H_ */
