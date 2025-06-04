/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-05     Rick       the first version
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "board.h"
#include "pin_config.h"
#include "agile_button.h"
#include "water_work.h"

#define DBG_TAG "PD"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

struct pd_chip_t
{
    agile_btn_t *pd_chip_irq_btn;
    struct rt_spi_device *pd_chip_device;
    uint8_t pd_input;
    uint8_t pd_model;
    uint8_t pd_type;
};

struct pd_chip_t pd_chip[2];
struct pd_chip_t *factory_chip;
static uint8_t factory_rssi_test_flag,pd_voltage_lock,pd_voltage_level,pd_current_level = 0;

rt_timer_t pd_factory_read_timer = RT_NULL;

void pd_spi_write_single(struct rt_spi_device *device, uint8_t address, uint8_t data)
{
    uint8_t write_address = address | 0x80;//write bit7 = 1
    rt_spi_send_then_send(device,&write_address,1,&data,1);
}

void pd_spi_write_multiple(struct rt_spi_device *device, uint8_t address, uint8_t* data,uint32_t length)
{
    uint8_t write_address = address | 0x80;//write bit7 = 1
    rt_spi_send_then_send(device,&write_address,1,data,length);
}

uint8_t pd_spi_read_single(struct rt_spi_device *device, uint8_t address)
{
    uint8_t read_address ;
    uint8_t data ;
    read_address = address & 0x7F ;//read bit7 = 0
    rt_spi_send_then_recv(device,&read_address,1,&data,1);
    return data ;
}

void pd_spi_read_multiple(struct rt_spi_device *device, uint8_t address, uint8_t* data, uint32_t length)
{
    uint8_t read_address ;
    read_address = address & 0x7F ;//read bit7 = 0
    rt_spi_send_then_recv(device,&read_address,1,data,length);
}

uint8_t pd_chip_factory_get(struct rt_spi_device *device)
{
    uint8_t data = pd_spi_read_single(device,0x01) & 0x80;
    return data;
}

uint8_t pd_chip_type_get(struct rt_spi_device *device)
{
    uint8_t data = (pd_spi_read_single(device,0x01) >> 4) & 0x07;
    return data;
}

uint8_t pd_chip_model_get(struct rt_spi_device *device)
{
    uint8_t data = pd_spi_read_single(device,0x01) & 0xF;
    return data;
}

uint8_t pd_chip_input_voltage_get(struct rt_spi_device *device)
{
    uint8_t data[2] = {0};
    uint16_t voltage = 0;
    pd_spi_read_multiple(device,0x05,data,2);
    voltage = ((data[0] << 8 | data[1]) & 0x1E00) >> 9;
    return voltage;
}

uint8_t pd_chip_input_current_get(struct rt_spi_device *device)
{
    uint8_t data[2] = {0};
    uint16_t current = 0;
    pd_spi_read_multiple(device,0x05,data,2);
    current = (data[0] << 8  | data[1]) & 0x1FF;
    return current;
}

void pd_chip_output_power_set(struct rt_spi_device *device,uint8_t voltage,uint8_t current)
{
    uint8_t data[2] = {0};
    data[0] = ((voltage << 9) | current) >> 8;
    data[1] = ((voltage << 9) | current) & 0xFF;
    pd_spi_write_multiple(device,0x06,data,2);
}

void pd_chip_factory_gpio_set(struct rt_spi_device *device)
{
    uint8_t data = 0;
    data = rt_pin_read(PD_CHIP_IRQ2_PIN);
    data |= rt_pin_read(PD_CHIP_IRQ1_PIN) << 1;
    data |= rt_pin_read(MOTO_CLOSE_STATUS_PIN) << 2;
    data |= rt_pin_read(MOTO_OPEN_STATUS_PIN) << 3;
    data |= rt_pin_read(SENSOR_LOST_PIN) << 4;
    data |= rt_pin_read(SENSOR_LEAK_PIN) << 5;
    data |= rt_pin_read(KEY_OFF_PIN) << 6;
    data |= rt_pin_read(KEY_ON_PIN) << 7;
    pd_spi_write_single(device,0x08,data);
}

void pd_chip_factory_info_set(struct rt_spi_device *device)
{
    uint8_t data = 0;

    extern enum Device_Status DeviceStatus;
    extern uint8_t valve_status;
    extern uint8_t internal_valve_open_result;
    extern uint8_t internal_valve_close_result;
    extern uint8_t internal_valve_check_result;
    extern uint8_t external_valve_open_result;
    extern uint8_t external_valve_close_result;
    extern uint8_t external_valve_check_result;

    data = DeviceStatus;
    data |= valve_status << 1;
    data |= internal_valve_close_result << 2;
    data |= internal_valve_open_result << 3;
    data |= internal_valve_check_result << 4;
    data |= external_valve_close_result << 5;
    data |= external_valve_open_result << 6;
    data |= external_valve_check_result << 7;
    pd_spi_write_single(device,0x09,data);
}

void pd_chip_self_addr_set(struct rt_spi_device *device)
{
    uint8_t data[4] = {0};
    data[0] = (get_local_address() >> 24) & 0xFF;
    data[1] = (get_local_address() >> 16) & 0xFF;
    data[2] = (get_local_address() >> 8) & 0xFF;
    data[3] = get_local_address() & 0xFF;
    pd_spi_write_multiple(device,0x0C,data,4);
}

void pd_chip_rssi_test_set(struct rt_spi_device *device)
{
    int8_t data[6] = {0};
    data[0] = factory_controller_recv_snr_read();
    data[1] = factory_controller_recv_rssi_read() & 0xFF;
    data[2] = (factory_controller_recv_rssi_read() >> 8) & 0xFF;
    data[3] = factory_gateway_recv_snr_read();
    data[4] = factory_gateway_recv_rssi_read() & 0xFF;
    data[5] = (factory_gateway_recv_rssi_read() >> 8) & 0xFF;
    pd_spi_write_multiple(device,0x0B,(uint8_t *)data,6);
}

void pd_factory_read_timer_callback(void *parameter)
{
    uint8_t action_data = 0;
    static uint8_t time_count = 0;
    static uint8_t simu_water_leak = 0;
    static uint8_t simu_valve_check = 0;
    static uint8_t simu_open_valve = 0;
    static uint8_t simu_close_valve = 0;
    static uint8_t simu_rf_test = 0;

    if(factory_chip == RT_NULL)
    {
        return;
    }

    pd_chip_factory_gpio_set(factory_chip->pd_chip_device);
    pd_chip_factory_info_set(factory_chip->pd_chip_device);
    pd_chip_rssi_test_set(factory_chip->pd_chip_device);

    action_data = pd_spi_read_single(factory_chip->pd_chip_device,0x0A);
    if(simu_water_leak != (action_data & 0x01))
    {
        simu_water_leak = action_data & 0x01;
        if(simu_water_leak)
        {
            factory_water_leak_simulate();
        }
    }

    if(simu_valve_check != (action_data & 0x02))
    {
        simu_valve_check = action_data & 0x02;
        if(simu_valve_check)
        {
            valve_check_start();
        }
    }

    if(simu_close_valve != (action_data & 0x04))
    {
        simu_close_valve = action_data & 0x04;
        if(simu_close_valve)
        {
            valve_close();
        }
    }

    if(simu_open_valve != (action_data & 0x08))
    {
        simu_open_valve = action_data & 0x08;
        if(simu_open_valve)
        {
            valve_open();
        }
    }

    simu_rf_test = (action_data & 0x10);
    if(simu_rf_test)
    {
        factory_rssi_test_flag = 1;
        if(time_count++ > 1)
        {
            time_count = 0;
            radio_factory_test_packet_send();
        }
    }
    else
    {
        factory_rssi_test_flag = 0;
    }
}

void pd_valve_control(uint8_t value)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        if(pd_chip[i].pd_type == 0x02)
        {
            pd_spi_write_single(pd_chip[i].pd_chip_device,0x02,value);
        }
    }
}

void pd_valve_check(void)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        if(pd_chip[i].pd_type == 0x02)
        {
            pd_spi_write_single(pd_chip[i].pd_chip_device,0x03,1);
        }
    }
}

int pd_valve_error_close(void)
{
    int error = -1;
    for(uint8_t i = 0; i < 2; i++)
    {
        if(pd_chip[i].pd_type == 0x02)
        {
            error = pd_spi_read_single(pd_chip[i].pd_chip_device,0x04) & 0x01;
        }
    }

    return error;
}

int pd_valve_error_open(void)
{
    int error = -1;
    for(uint8_t i = 0; i < 2; i++)
    {
        if(pd_chip[i].pd_type == 0x02)
        {
            error = pd_spi_read_single(pd_chip[i].pd_chip_device,0x04) & 0x02;
        }
    }

    return error;
}

int pd_valve_error_check(void)
{
    int error = -1;
    for(uint8_t i = 0; i < 2; i++)
    {
        if(pd_chip[i].pd_type == 0x02)
        {
            error = pd_spi_read_single(pd_chip[i].pd_chip_device,0x04) & 0x04;
        }
    }

    return error;
}

uint8_t pd_chip_lock_voltage_get(void)
{
    return pd_voltage_level;
}

uint8_t factory_rssi_test_flag_get(void)
{
    return factory_rssi_test_flag;
}

void pd_chip_plug_in_handshake(struct pd_chip_t *pd)
{
    pd->pd_type = pd_chip_type_get(pd->pd_chip_device);
    LOG_D("pd_chip_type_get 0x%02X\r\n",pd->pd_type);
    pd->pd_model = pd_chip_model_get(pd->pd_chip_device);
    LOG_D("pd_chip_model_get 0x%02X\r\n",pd->pd_model);
    if(pd->pd_type == 0x01)//power input
    {
        pd->pd_input = 1;
        if(pd_voltage_lock == 0)//lock pd voltage
        {
            uint8_t input_voltage = pd_chip_input_voltage_get(pd->pd_chip_device);
            LOG_D("pd_chip_input_voltage_get %d\r\n",input_voltage);
            if(input_voltage >= 3)//0:off 1:5V 2:9V 3:12V 4:20V
            {
                pd_voltage_lock = 1;
                pd_voltage_level = input_voltage;
                pd_current_level = pd_chip_input_current_get(pd->pd_chip_device);
                pd_output_control(pd_voltage_level,pd_current_level);
            }
        }
    }
    else if(pd->pd_type == 0x02)//device
    {
        pd->pd_input = 0;
        pd_valve_control(get_valve_status());
        if(pd_chip_factory_get(pd->pd_chip_device) && (rt_tick_get() < 10 * 1000))
        {
            factory_chip = pd;
            led_factory_gw_blink();
            pd_chip_self_addr_set(factory_chip->pd_chip_device);
            rt_timer_start(pd_factory_read_timer);
        }
    }
    else if(pd->pd_type == 0x03)//power output
    {
        if(pd_voltage_level >= 3)//0:off 1:5V 2:9V 3:12V 4:20V
        {
            pd_output_control(pd_voltage_level,pd_current_level);
        }
    }
}

void pd_chip_plug_in_callback(agile_btn_t *btn)
{
    LOG_D("pd_chip_plug_in_callback,pin %d\r\n",btn->pin);
    switch(btn->pin)
    {
    case PD_CHIP_IRQ1_PIN:
        pd_chip_plug_in_handshake(&pd_chip[0]);
        break;
    case PD_CHIP_IRQ2_PIN:
        pd_chip_plug_in_handshake(&pd_chip[1]);
        break;
    default:
        break;
    }
}

void pd_chip_plug_out_handshake(struct pd_chip_t *pd)
{
    if(factory_chip != RT_NULL && pd == factory_chip)
    {
        LOG_D("Exit factory test mode,rebooting now");
        rt_thread_mdelay(1000);
        rt_hw_cpu_reset();
    }
}

void pd_chip_plug_out_callback(agile_btn_t *btn)
{
    LOG_D("pd_chip_plug_out_callback,pin %d\r\n",btn->pin);
    switch(btn->pin)
    {
    case PD_CHIP_IRQ1_PIN:
        pd_chip_plug_out_handshake(&pd_chip[0]);
        break;
    case PD_CHIP_IRQ2_PIN:
        pd_chip_plug_out_handshake(&pd_chip[1]);
        break;
    default:
        break;
    }
}

void pd_output_control(uint8_t output_voltage,uint16_t output_current)
{
    for(uint8_t i = 0; i < 2; i++)
    {
        if(pd_chip[i].pd_input == 0)
        {
            pd_chip_output_power_set(pd_chip[i].pd_chip_device,pd_voltage_level,pd_current_level);
            LOG_D("pd_output_control to chip[%d],output_voltage %d,output_current %d\r\n",i,output_voltage,output_current);
        }
    }
}

void pd_init(void)
{
    uint8_t i = 0;
    pd_chip[0].pd_chip_irq_btn = agile_btn_create(PD_CHIP_IRQ1_PIN, PIN_HIGH, PIN_MODE_INPUT);
    pd_chip[1].pd_chip_irq_btn = agile_btn_create(PD_CHIP_IRQ2_PIN, PIN_HIGH, PIN_MODE_INPUT);
    for(i = 0; i < 2; i++)
    {
        agile_btn_set_event_cb(pd_chip[i].pd_chip_irq_btn, BTN_PRESS_DOWN_EVENT, pd_chip_plug_in_callback);
        agile_btn_set_event_cb(pd_chip[i].pd_chip_irq_btn, BTN_PRESS_UP_EVENT, pd_chip_plug_out_callback);
        agile_btn_start(pd_chip[i].pd_chip_irq_btn);
    }

    pd_factory_read_timer = rt_timer_create("factory_read", pd_factory_read_timer_callback, \
                                                RT_NULL, 1000, RT_TIMER_FLAG_SOFT_TIMER | RT_TIMER_FLAG_PERIODIC);
    rt_hw_spi_device_attach("spi1", "spi11", GPIOA, GPIO_PIN_1);
    rt_hw_spi_device_attach("spi1", "spi12", GPIOA, GPIO_PIN_2);

    pd_chip[0].pd_chip_device = (struct rt_spi_device *)rt_device_find("spi11");
    pd_chip[1].pd_chip_device = (struct rt_spi_device *)rt_device_find("spi12");

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0. */
    cfg.max_hz = 200 * 1000;             /* max 200k */
    for(i = 0; i < 2; i++)
    {
        rt_spi_configure(pd_chip[i].pd_chip_device, &cfg);
    }
}
