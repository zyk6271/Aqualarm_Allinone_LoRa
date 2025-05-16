#include <rtthread.h>
#include <rtdevice.h>
#include "pin_config.h"
#include "rtc.h"
#include "board.h"

#define DBG_TAG "RTC"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

rt_sem_t rtc_sem;
RTC_HandleTypeDef rtc_handle;
rt_thread_t rtc_thread = RT_NULL;

static uint8_t valve_hours,rtc_hours = 0;

void rtc_thread_entry(void *parameter)
{
    aq_device_heart_recv_clear();//clear all slave device recv flag to zero
    while(1)
    {
        rt_sem_take(rtc_sem, RT_WAITING_FOREVER);
        LOG_I("rtc_hours is %d\r\n",rtc_hours);
        if((++ valve_hours) % 120 == 0)
        {
            valve_hours = 0;
            valve_check_start();
        }
        if(rtc_hours < 47)
        {
            rtc_hours++;
        }
        else
        {
            rtc_hours = 0;
            aq_device_heart_check();
        }
        gateway_heart_check();
    }
}

void RTC_Alarm_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(&rtc_handle);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    rt_sem_release(rtc_sem);

    RTC_TimeTypeDef sTime = {0};
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&rtc_handle, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
}

void rtc_init(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    RTC_AlarmTypeDef sAlarm = {0};

    /* USER CODE BEGIN RTC_Init 1 */
    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
    */
    rtc_handle.Instance = RTC;
    rtc_handle.Init.HourFormat = RTC_HOURFORMAT_24;
    rtc_handle.Init.AsynchPrediv = 127;
    rtc_handle.Init.SynchPrediv = 255;
    rtc_handle.Init.OutPut = RTC_OUTPUT_DISABLE;
    rtc_handle.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    rtc_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtc_handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    rtc_handle.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
    rtc_handle.Init.BinMode = RTC_BINARY_NONE;
    if (HAL_RTC_Init(&rtc_handle) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN Check_RTC_BKUP */

    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
    */
    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&rtc_handle, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_FEBRUARY;
    sDate.Date = 0x18;
    sDate.Year = 0x14;

    if (HAL_RTC_SetDate(&rtc_handle, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    /** Enable the Alarm A
    */
    sAlarm.AlarmTime.Hours = 0x1;
    sAlarm.AlarmTime.Minutes = 0x0;
    sAlarm.AlarmTime.Seconds = 0x0;
    sAlarm.AlarmTime.SubSeconds = 0x0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
    sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&rtc_handle, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

    rtc_sem = rt_sem_create("rtc_sem", 0, RT_IPC_FLAG_FIFO);
    rtc_thread = rt_thread_create("rtc_thread", rtc_thread_entry, RT_NULL, 2048, 11, 10);
    rt_thread_startup(rtc_thread);
}
