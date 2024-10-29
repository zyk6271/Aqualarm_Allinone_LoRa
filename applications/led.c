/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-03     Rick       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "signal_led.h"
#include "pin_config.h"

//定义信号灯对象句柄
static led_t *led_obj_off_red = RT_NULL;
static led_t *led_obj_loss_red = RT_NULL;
static led_t *led_obj_off_red_once = RT_NULL;
static led_t *led_obj_off_red_three = RT_NULL;
static led_t *led_obj_on_green = RT_NULL;

static led_t *beep_obj = RT_NULL;
static led_t *beep_obj_loss = RT_NULL;
static led_t *beep_obj_once = RT_NULL;
static led_t *beep_obj_learn = RT_NULL;
static led_t *beep_obj_three = RT_NULL;

static led_t *led_obj_wifi_green_led = RT_NULL;
static led_t *led_obj_wifi_red_led = RT_NULL;
static led_t *led_obj_wifi_blue_led = RT_NULL;

rt_thread_t led_thread = RT_NULL;

uint8_t led_valve_on_pause_flag = 0;

//定义内存操作函数接口
led_mem_opreation_t led_mem_opreation;

static void gw_green_on(void *param)
{
    ws2812b_green(0,1);
}

static void gw_green_off(void *param)
{
    ws2812b_green(0,0);
}

static void gw_red_on(void *param)
{
    ws2812b_red(0,1);
}

static void gw_red_off(void *param)
{
    ws2812b_red(0,0);
}

static void gw_blue_on(void *param)
{
    ws2812b_blue(0,1);
}

static void gw_blue_off(void *param)
{
    ws2812b_blue(0,0);
}

static void off_red_on(void *param)
{
    ws2812b_red(1,1);
}

static void off_red_off(void *param)
{
    ws2812b_red(1,0);
}

static void on_green_on(void *param)
{
    if(led_valve_on_pause_flag == 0)
    {
        ws2812b_green(1,1);
    }
}

static void on_green_off(void *param)
{
    ws2812b_green(1,0);
}

static void beep_on(void *param)
{
    rt_pin_mode(BEEP_PIN,PIN_MODE_OUTPUT);
    rt_pin_write(BEEP_PIN,PIN_HIGH);
}

static void beep_close(void *param)
{
    rt_pin_mode(BEEP_PIN,PIN_MODE_OUTPUT);
    rt_pin_write(BEEP_PIN,PIN_LOW);
}

static void led_run(void *parameter)
{
    ws2812b_init();
    while(1)
    {
        rt_thread_mdelay(LED_TICK_TIME);
        led_ticks();
        RGB_SendArray();
    }
}

void beep_three_times(void)
{
    led_start(beep_obj_three);
    led_start(led_obj_off_red_three);
}

void wifi_communication_blink(void)
{
    led_start(led_obj_wifi_blue_led);
}

void wifi_led(uint8_t num)
{
    switch(num)
    {
    case 0://无设备
        led_stop(led_obj_wifi_red_led);
        led_set_mode(led_obj_wifi_green_led, LOOP_PERMANENT,"500,500,");
        led_start(led_obj_wifi_green_led);
        break;
    case 1://心跳成功
        led_stop(led_obj_wifi_red_led);
        led_set_mode(led_obj_wifi_green_led, LOOP_PERMANENT,"200,0,");
        led_start(led_obj_wifi_green_led);
        break;
    case 2://心跳失败
        led_stop(led_obj_wifi_green_led);
        led_set_mode(led_obj_wifi_red_led, LOOP_PERMANENT,"200,0,");
        led_start(led_obj_wifi_red_led);
        break;
    default:
        break;
    }
}
void led_valve_fail(void)
{
    led_set_mode(beep_obj, 3,"200,200,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, 3,"200,200,");
    led_start(led_obj_off_red);
}
void led_notice_once(void)
{
    led_set_mode(beep_obj, 1,"200,0,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, 1,"200,0,");
    led_start(led_obj_off_red);
}
void led_factory_start(void)
{
    led_stop(led_obj_on_green);
    led_set_mode(beep_obj, 5,"200,200,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, 5,"200,200,");
    led_start(led_obj_off_red);
}
void led_learn_start(void)
{
    led_set_mode(beep_obj, 5,"200,200,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, 75,"200,200,");
    led_start(led_obj_off_red);
}
void led_slave_low_start(void)
{
    led_set_mode(beep_obj, LOOP_PERMANENT,"200,5000,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, LOOP_PERMANENT,"200,5000,");
    led_start(led_obj_off_red);
}
void led_moto_fail_start(void)
{
    led_set_mode(beep_obj, LOOP_PERMANENT,"200,200,200,200,200,200,200,200,200,200,200,10000,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, LOOP_PERMANENT,"200,200,200,200,200,200,200,200,200,200,200,10000,");
    led_start(led_obj_off_red);
}
void led_offline_start(void)
{
    led_set_mode(beep_obj, LOOP_PERMANENT,"200,200,200,200,200,200,200,5000,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, LOOP_PERMANENT,"200,200,200,200,200,200,200,5000,");
    led_start(led_obj_off_red);
}
void led_master_lost_start(void)
{
    led_set_mode(beep_obj_loss, LOOP_PERMANENT,"200,200,200,5000,");
    led_start(beep_obj_loss);
    led_set_mode(led_obj_loss_red, LOOP_PERMANENT,"200,200,200,5000,");
    led_start(led_obj_loss_red);
}
void led_water_alarm_start(void)
{
    led_set_mode(beep_obj, LOOP_PERMANENT,"200,200,200,200,200,5000,");
    led_start(beep_obj);
    led_set_mode(led_obj_off_red, LOOP_PERMANENT,"200,200,200,200,200,5000,");
    led_start(led_obj_off_red);
}

void beep_stop(void)
{
    led_stop(beep_obj_loss);
    led_stop(beep_obj);
}

void beep_key_down(void)
{
    led_start(beep_obj_once);
    led_start(led_obj_off_red_once);
}

void beep_once(void)
{
    led_start(beep_obj_once);
}

void learn_fail_ring(void)
{
    led_start(beep_obj_learn);
}

void led_relearn(void)
{
    led_set_mode(led_obj_off_red, 75,"200,200,");
    led_start(led_obj_off_red);
}

void led_valve_on_pause(void)
{
    led_stop(led_obj_on_green);
    led_valve_on_pause_flag = 1;
}

void led_valve_on_resume(void)
{
    led_valve_on_pause_flag = 0;
    if(get_valve_status())
    {
        led_start(led_obj_on_green);
    }
}

void led_ntc_alarm(void)
{
    led_set_mode(beep_obj, LOOP_PERMANENT,"50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50\
                            ,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,2000,");
    led_set_mode(led_obj_off_red, LOOP_PERMANENT,"50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50\
                            ,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,2000,");
    led_set_mode(led_obj_on_green, LOOP_PERMANENT,"50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50\
                            ,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,2000,");
    led_start(led_obj_off_red);
    led_start(led_obj_on_green);
    led_start(beep_obj);
}

void led_valve_on(void)
{
    led_stop(led_obj_off_red);
    led_set_mode(led_obj_on_green,LOOP_PERMANENT,"200,0,");
    led_start(led_obj_on_green);
}

void led_valve_off(void)
{
    led_stop(led_obj_on_green);
}

void led_warn_off(void)
{
    led_stop(led_obj_off_red);
}

void led_loss_off(void)
{
    led_stop(led_obj_loss_red);
}

void led_factory_gw_blink(void)
{
    led_set_mode(led_obj_wifi_green_led, LOOP_PERMANENT, "500,500,");
    led_set_mode(led_obj_wifi_red_led, LOOP_PERMANENT, "0,500,500,");
    led_start(led_obj_wifi_green_led);
    led_start(led_obj_wifi_red_led);
}

int led_init(void)
{
    led_mem_opreation.malloc_fn = (void* (*)(size_t))rt_malloc;
    led_mem_opreation.free_fn = rt_free;
    led_set_mem_operation(&led_mem_opreation);

    led_obj_off_red = led_create(off_red_on, off_red_off, NULL);
    led_set_mode(led_obj_off_red, LOOP_PERMANENT, "200,1,");

    led_obj_loss_red = led_create(off_red_on, off_red_off, NULL);
    led_set_mode(led_obj_loss_red, LOOP_PERMANENT, "200,200,");

    led_obj_off_red_once = led_create(off_red_on, off_red_off, NULL);
    led_set_mode(led_obj_off_red_once, 1, "200,1,");

    led_obj_off_red_three = led_create(off_red_on, off_red_off, NULL);
    led_set_mode(led_obj_off_red_three, 3, "200,200,");

    led_obj_on_green = led_create(on_green_on, on_green_off, NULL);
    led_set_mode(led_obj_on_green, 3, "200,200,");

    beep_obj = led_create(beep_on, beep_close, NULL);
    led_set_mode(beep_obj, LOOP_PERMANENT, "200,200,");

    beep_obj_loss = led_create(beep_on, beep_close, NULL);
    led_set_mode(beep_obj_loss, LOOP_PERMANENT, "200,200,");

    beep_obj_once = led_create(beep_on, beep_close, NULL);
    led_set_mode(beep_obj_once, 1, "200,1,");

    beep_obj_three = led_create(beep_on, beep_close, NULL);
    led_set_mode(beep_obj_three, 3, "200,200,");

    beep_obj_learn = led_create(beep_on, beep_close, NULL);
    led_set_mode(beep_obj_learn, 3, "50,50,200,200,");

    led_obj_wifi_green_led = led_create(gw_green_on, gw_green_off, NULL);
    led_set_mode(led_obj_wifi_green_led, LOOP_PERMANENT, "200,200,");

    led_obj_wifi_red_led = led_create(gw_red_on, gw_red_off, NULL);
    led_set_mode(led_obj_wifi_red_led, LOOP_PERMANENT, "200,200,");

    led_obj_wifi_blue_led = led_create(gw_blue_on, gw_blue_off, NULL);
    led_set_mode(led_obj_wifi_blue_led, 1, "50,50,");

    led_thread = rt_thread_create("signal_led",
                            led_run,
                            RT_NULL,
                            512,
                            RT_THREAD_PRIORITY_MAX/2,
                            100);
    if (led_thread != RT_NULL)
    {
        rt_thread_startup(led_thread);
    }

    return RT_EOK;
}
