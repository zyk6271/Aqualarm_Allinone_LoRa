/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-28     Rick       the first version
 */
#include "rtthread.h"
#include "board.h"

#define DBG_TAG "RADIO_UTILS"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

CRC_HandleTypeDef crc_handle;

volatile unsigned char lora_tx_buf[255];         //LoRa发送缓存

void radio_crc_init(void)
{
    crc_handle.Instance = CRC;
    crc_handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
    crc_handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
    crc_handle.Init.GeneratingPolynomial = 0x07;
    crc_handle.Init.CRCLength = CRC_POLYLENGTH_8B;
    crc_handle.Init.InitValue = 0;
    crc_handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    crc_handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    crc_handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&crc_handle) != HAL_OK)
    {
        Error_Handler();
    }
}

unsigned char get_check_crc(unsigned char *pack, unsigned short pack_len)
{
    unsigned char calc_crc = HAL_CRC_Calculate(&crc_handle, (uint32_t *)pack, pack_len);

    return calc_crc;
}

unsigned char *get_lora_tx_buf(void)
{
    return lora_tx_buf;
}

/**
 * @brief  写单字节
 * @param[in] {dest} 缓存区地址偏移
 * @param[in] {byte} 写入字节值
 * @return 写入完成后的总长度
 */
unsigned short set_lora_tx_byte(unsigned short dest, unsigned char byte)
{
    unsigned char *obj = (unsigned char *)lora_tx_buf + dest;

    *obj = byte;
    dest += 1;

    return dest;
}

/**
 * @brief  写wifi_uart_buffer
 * @param[in] {dest} 缓存区地址偏移
 * @param[in] {src} 源地址（需要发送的数据）
 * @param[in] {len} 需要发送的数据长度
 * @return 写入完成后的总长度
 */
unsigned short set_lora_tx_buffer(unsigned short dest, const unsigned char *src, unsigned short len)
{
    unsigned char *obj = (unsigned char *)lora_tx_buf + dest;

    rt_memcpy(obj,src,len);

    dest += len;
    return dest;
}

/**
 * @brief  写4字节
 * @param[in] {dest} 缓存区地址偏移
 * @param[in] {byte} 写入字节值
 * @return 写入完成后的总长度
 */
unsigned short set_lora_tx_word(unsigned short dest, unsigned int word)
{
    unsigned char *obj = (unsigned char *)lora_tx_buf + dest;

    *obj = (word >> 24) & 0xFF;
    obj ++;
    *obj = (word >> 16) & 0xFF;
    obj ++;
    *obj = (word >> 8) & 0xFF;
    obj ++;
    *obj = word & 0xFF;

    dest += 4;

    return dest;
}

/**
 * @param[in] {dest} 缓存区地址偏移
 * @param[in] {src} 源地址（需要发送的数据）
 * @param[in] {len} 需要发送的数据长度
 * @return 写入完成后的总长度
 */
unsigned short set_lora_tx_crc(unsigned short dest)
{
    unsigned char *obj = (unsigned char *)lora_tx_buf + dest;

    *obj = HAL_CRC_Calculate(&crc_handle, (uint32_t *)lora_tx_buf, dest);
    dest += 1;

    return dest;
}
