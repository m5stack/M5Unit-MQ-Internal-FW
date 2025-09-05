/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __MYFLASH_H
#define __MYFLASH_H

#ifdef __cplusplus

extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_hal_flash_ex.h"

#define STM32G0xx_PAGE_SIZE              (0x800)
#define STM32G0xx_FLASH_PAGE0_STARTADDR  (0x8000000)
#define STM32G0xx_FLASH_PAGE15_STARTADDR (STM32G0xx_FLASH_PAGE0_STARTADDR + 15 * STM32G0xx_PAGE_SIZE)
#define I2C_ADDR                         (STM32G0xx_FLASH_PAGE15_STARTADDR + 0)

uint8_t set_i2c_addr(uint8_t data);
uint8_t get_i2c_addr(void);

#ifdef __cplusplus
}
#endif

#endif /* __MYFLASH_H */
