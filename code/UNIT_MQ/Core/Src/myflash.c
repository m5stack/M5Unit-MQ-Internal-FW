#include "myflash.h"

/**
 * @brief  Get the flash page number from a given address.
 * @param  addr Flash memory address.
 * @retval Page number.
 */
static uint32_t get_page(uint32_t addr)
{
    return (addr - STM32G0xx_FLASH_PAGE0_STARTADDR) / FLASH_PAGE_SIZE;
}

/**
 * @brief  Modify a single byte in a 64-bit value.
 * @param  data Pointer to the 64-bit data.
 * @param  byte_index Index of the byte to modify (0~7).
 * @param  new_value New byte value.
 */
static void set_byte_in_uint64(uint64_t *data, uint8_t byte_index, uint8_t new_value)
{
    *data &= ~((uint64_t)(0xFF) << (byte_index * 8));
    *data |= (uint64_t)new_value << (byte_index * 8);
}

/**
 * @brief  Modify a single byte in a 32-bit value.
 * @param  data Pointer to the 32-bit data.
 * @param  byte_index Index of the byte to modify (0~3).
 * @param  new_value New byte value.
 */
static void set_byte_in_uint32(uint32_t *data, uint8_t byte_index, uint8_t new_value)
{
    *data &= ~((uint32_t)(0xFF) << (byte_index * 8));
    *data |= (uint32_t)new_value << (byte_index * 8);
}

/**
 * @brief  Read a 32-bit word from flash.
 * @param  address Flash memory address.
 * @retval The read value.
 */
static uint32_t my_flash_read_word(uint32_t address)
{
    return *((__IO uint32_t *)(address));
}

/**
 * @brief  Erase a flash page.
 * @param  page_address Address within the page to erase.
 * @retval 1 if successful, 0 otherwise.
 */
static uint8_t my_flash_earse_pages(uint32_t page_address)
{
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef my_flash;
    my_flash.TypeErase = FLASH_TYPEERASE_PAGES;
    my_flash.Page      = get_page(page_address);
    my_flash.NbPages   = 1;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&my_flash, &page_error);
    HAL_FLASH_Lock();

    if (status == HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief  Write a 16-bit half-word to flash.
 * @param  address Flash memory address.
 * @param  data 16-bit data to write.
 * @retval 1 if successful, 0 otherwise.
 */
static uint8_t my_flash_write_half_word(uint32_t address, uint16_t data)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();
    return (status == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Write a 32-bit word to flash.
 * @param  address Flash memory address.
 * @param  data 32-bit data to write.
 * @retval 1 if successful, 0 otherwise.
 */
static uint8_t my_flash_write_word(uint32_t address, uint32_t data)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();
    return (status == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Write a 64-bit double-word to flash.
 * @param  address Flash memory address.
 * @param  data 64-bit data to write.
 * @retval 1 if successful, 0 otherwise.
 */
static uint8_t my_flash_write_double_word(uint32_t address, uint64_t data)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();
    return (status == HAL_OK) ? 1 : 0;
}

/**
 * @brief  Set the I2C address in flash.
 * @param  data New I2C address.
 * @retval 1 if successful, 0 otherwise.
 */
uint8_t set_i2c_addr(uint8_t data)
{
	__disable_irq();
    uint32_t temp = my_flash_read_word(I2C_ADDR);
    set_byte_in_uint32(&temp, 0, data);
    my_flash_earse_pages(I2C_ADDR);
    while (my_flash_write_word(I2C_ADDR, temp) != 1) {
    }
    __enable_irq();
    return (get_i2c_addr() == data) ? 1 : 0;
}

/**
 * @brief  Get the I2C address stored in flash.
 * @retval I2C address.
 */
uint8_t get_i2c_addr(void)
{
    return *((__IO uint8_t *)(I2C_ADDR));
}
