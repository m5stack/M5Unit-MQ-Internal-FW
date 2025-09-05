/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "myflash.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FIRMWARE_VERSION    (1)
#define I2C1_ADDR_BASE      (0x11)
#define ADC_CHANNEL_NUMS    (3)
#define ADC_AVERAGE_NUMS    (20)
#define ADC_SAMPLES_NUMS    (ADC_CHANNEL_NUMS * ADC_AVERAGE_NUMS)
#define HEAT_DURATION       (20 * 1000)
#define APPLICATION_ADDRESS ((uint32_t)0x08001800)
#define VREFINT_CALIB       (*((uint16_t *)0x1FFF75AA))
#define ADC_MAX             (4095)

#define MQ_CFG_REG_ADDR                      (0x00)
#define LED_CFG_REG_ADDR                     (0x01)
#define MQ_HIGH_PIN_CFG_REG_ADDR             (0x10)
#define MQ_LOW_PIN_CFG_REG_ADDR              (0x11)
#define MQ_8B_ADC_REG_ADDR                   (0x20)
#define MQ_12B_ADC_REG_ADDR                  (0x30)
#define MQ_ADC_VALID_TAGS_REG_ADDR           (0x40)
#define NTC_TEMP_8B_ADC_REG_ADDR             (0x50)
#define NTC_TEMP_12B_ADC_REG_ADDR            (0x60)
#define NTC_TEMP_VAL_REG_ADDR                (0x70)
#define INTERNALT_REFERENCE_VOLTAGE_REG_ADDR (0x80)
#define MQ_CHANNEL_VOLTAGE_REG_ADDR          (0x82)
#define NTC_CHANNEL_VOLTAGE_REG_ADDR         (0x84)
#define IAP_UPDATE_ADDR                      (0xFD)
#define SW_VER_REG_ADDR                      (0xFE)
#define I2C_ADDR_REG_ADDR                    (0xFF)

typedef enum {
    MODE_OFF = 0x00,  // Off mode
    MODE_HEAT,        // Constant temperature heating mode
    MODE_PIN_SWITCH   // Pin level switching mode
} mq_mode_t;

typedef enum {
    LED_DISABLE = 0x00,  // Disabled
    LED_ENABLE           // Enabled
} led_status_t;

typedef enum {
    HIGH_LEVEL = 0x00,  // High level
    LOW_LEVEL           // Low level
} level_status_t;

__IO static uint8_t mq_cfg_reg                  = 0;
__IO static uint8_t led_cfg_reg                 = 0;
__IO static uint8_t mq_heat_pin_cfg_reg[2]      = {30, 5};
__IO static uint8_t mq_8b_adc_reg               = 0;
__IO static uint16_t mq_12b_adc_reg             = 0;
__IO static uint8_t mq_adc_valid_tags           = 0;
__IO static uint8_t ntc_8b_adc_reg              = 0;
__IO static uint16_t ntc_12b_adc_reg            = 0;
__IO static uint8_t ntc_resistance_val_reg[2]   = {0};
__IO static uint16_t internal_reference_voltage = 0;
__IO static uint16_t mq_voltage         = 0;
__IO static uint16_t ntc_voltage        = 0;
__IO static uint8_t sw_ver_reg                  = FIRMWARE_VERSION;
__IO uint8_t i2c_addr_reg                       = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

__IO uint32_t adc_value_buf[ADC_SAMPLES_NUMS];
__IO uint32_t i2c_stop_timeout_delay = 0;
__IO uint8_t mq_adc_flag             = 0;
__IO uint8_t mq_work_mode_switch     = 0;
__IO uint32_t mq_start_time          = 0;
__IO uint32_t mq_level_status        = LOW_LEVEL;
__IO uint32_t high_level_time        = 0;
__IO uint32_t low_level_time         = 0;
static uint16_t adc_12bit            = 0;
uint16_t led_pwm                     = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void unit_mq_init(void)
{
    uint8_t data = get_i2c_addr();
    if (data == 0xFF) {
        i2c_addr_reg = I2C1_ADDR_BASE;
    } else {
        i2c_addr_reg = data;
    }
    high_level_time = mq_heat_pin_cfg_reg[0] * 1000;
    low_level_time  = mq_heat_pin_cfg_reg[1] * 1000;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    long result;

    result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (result < out_min)
        result = out_min;
    else if (result > out_max)
        result = out_max;

    return result;
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
    uint8_t rx_buf[16];
    uint8_t tx_buf[32];
    uint8_t rx_mark[16] = {0};
    if (len > 1) {
        if (rx_data[0] <= 0x01 && len <= 3) {
            for (int i = 0; i < len - 1; i++) {
                rx_buf[rx_data[0] + i]  = rx_data[1 + i];
                rx_mark[rx_data[0] + i] = 1;
            }
            if (rx_mark[0]) {
                if (mq_cfg_reg != rx_buf[0] && rx_buf[0] < 0x03) {
                    mq_work_mode_switch = 1;
                    mq_cfg_reg          = rx_buf[0];
                }
            }
            if (rx_mark[1]) {
                if (rx_buf[1] < 0x02) {
                    led_cfg_reg = rx_buf[1];
                }
            }
        } else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x11 && len <= 3) {
            for (int i = 0; i < len - 1; i++) {
                rx_buf[rx_data[0] + i - 0x10]  = rx_data[1 + i];
                rx_mark[rx_data[0] + i - 0x10] = 1;
            }
            if (rx_mark[0]) {
                if (rx_buf[0] >= 30) {
                    mq_heat_pin_cfg_reg[0] = rx_buf[0];
                    high_level_time        = mq_heat_pin_cfg_reg[0] * 1000;
                }
            }
            if (rx_mark[1]) {
                if (rx_buf[1] >= 5) {
                    mq_heat_pin_cfg_reg[1] = rx_buf[1];
                    low_level_time         = mq_heat_pin_cfg_reg[1] * 1000;
                }
            }
        } else if (rx_data[0] == I2C_ADDR_REG_ADDR && len == 2) {
            if (rx_data[1] >= 0x08 && rx_data[1] <= 0x77) {
                if (i2c_addr_reg != rx_data[1]) {
                    i2c_addr_reg = rx_data[1];
                    set_i2c_addr(i2c_addr_reg);
                    user_i2c_init();
                }
            }
        } else if (rx_data[0] == IAP_UPDATE_ADDR && len == 2) {
            if (rx_data[1] > 0) {
                NVIC_SystemReset();  // Reset the MCU, used for IAP firmware upgrade
            }
        }
    }

    if (len == 1) {
        if (rx_data[0] <= LED_CFG_REG_ADDR) {
            uint8_t data_len = 2 + 0x00 - rx_data[0];
            tx_buf[0]        = mq_cfg_reg;
            tx_buf[1]        = led_cfg_reg;
            i2c1_set_send_data((tx_buf + 2 - data_len), data_len);
        } else if (rx_data[0] <= MQ_LOW_PIN_CFG_REG_ADDR) {
            uint8_t data_len = 2 + 0x10 - rx_data[0];
            memcpy(tx_buf, (uint8_t *)mq_heat_pin_cfg_reg, 2);
            i2c1_set_send_data((tx_buf + 2 - data_len), data_len);
        } else if (rx_data[0] == MQ_8B_ADC_REG_ADDR) {
            tx_buf[0] = mq_8b_adc_reg;
            i2c1_set_send_data(tx_buf, 1);
        } else if (rx_data[0] <= (MQ_12B_ADC_REG_ADDR + 1)) {
            uint8_t data_len = 2 + 0x30 - rx_data[0];
            tx_buf[0]        = mq_12b_adc_reg & 0xFF;
            tx_buf[1]        = (mq_12b_adc_reg >> 8) & 0xFF;
            i2c1_set_send_data((tx_buf + 2 - data_len), data_len);
        } else if (rx_data[0] == MQ_ADC_VALID_TAGS_REG_ADDR) {
            tx_buf[0] = mq_adc_valid_tags;
            i2c1_set_send_data(tx_buf, 1);
        } else if (rx_data[0] == NTC_TEMP_8B_ADC_REG_ADDR) {
            tx_buf[0] = ntc_8b_adc_reg;
            i2c1_set_send_data(tx_buf, 1);
        } else if (rx_data[0] <= NTC_TEMP_12B_ADC_REG_ADDR) {
            uint8_t data_len = 2 + 0x60 - rx_data[0];
            tx_buf[0]        = ntc_12b_adc_reg & 0xFF;
            tx_buf[1]        = (ntc_12b_adc_reg >> 8) & 0xFF;
            i2c1_set_send_data((tx_buf + 2 - data_len), data_len);
        } else if (rx_data[0] <= NTC_TEMP_VAL_REG_ADDR) {
            uint8_t data_len = 2 + 0x70 - rx_data[0];
            tx_buf[0]        = ntc_resistance_val_reg[0];
            tx_buf[1]        = ntc_resistance_val_reg[1];
            i2c1_set_send_data((tx_buf + 2 - data_len), data_len);
        } else if (rx_data[0] <= (NTC_CHANNEL_VOLTAGE_REG_ADDR + 1)) {
            uint8_t data_len = 6 + 0x80 - rx_data[0];
            tx_buf[0]        = internal_reference_voltage & 0xFF;
            tx_buf[1]        = (internal_reference_voltage >> 8) & 0xFF;
            tx_buf[2]        = mq_voltage & 0xFF;
            tx_buf[3]        = (mq_voltage >> 8) & 0xFF;
            tx_buf[4]        = ntc_voltage & 0xFF;
            tx_buf[5]        = (ntc_voltage >> 8) & 0xFF;
            i2c1_set_send_data((tx_buf + 6 - data_len), data_len);
        } else if (rx_data[0] == SW_VER_REG_ADDR) {
            i2c1_set_send_data((uint8_t *)&sw_ver_reg, sizeof(sw_ver_reg));
        } else if (rx_data[0] == I2C_ADDR_REG_ADDR) {
            i2c1_set_send_data((uint8_t *)&i2c_addr_reg, sizeof(i2c_addr_reg));
        }
    }
}

void iap_set()
{
    uint8_t i;
    uint32_t *pVecTab = (uint32_t *)(0x20000000);
    for (i = 0; i < 48; i++) {
        *(pVecTab++) = *(__IO uint32_t *)(APPLICATION_ADDRESS + (i << 2));
    }
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    iap_set();
    unit_mq_init();
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_IWDG_Init();
    /* USER CODE BEGIN 2 */
    user_i2c_init();
    i2c1_it_enable();
    LL_TIM_EnableAllOutputs(TIM1);
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value_buf, ADC_SAMPLES_NUMS);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
    LL_TIM_OC_SetCompareCH1(TIM1, 0);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        mq_adc_valid_tags   = mq_adc_flag;
        i2c_timeout_counter = 0;
        if (i2c_stop_timeout_flag) {
            if (i2c_stop_timeout_delay < HAL_GetTick()) {
                i2c_stop_timeout_counter++;
                i2c_stop_timeout_delay = HAL_GetTick() + 10;
            }
        }
        if (i2c_stop_timeout_counter > 50) {
            LL_I2C_DeInit(I2C1);
            LL_I2C_DisableAutoEndMode(I2C1);
            LL_I2C_Disable(I2C1);
            LL_I2C_DisableIT_ADDR(I2C1);
            user_i2c_init();
            i2c1_it_enable();
            HAL_Delay(10);
        }
        // led status switch
        if (led_cfg_reg == LED_DISABLE) {
            LL_TIM_OC_SetCompareCH1(TIM1, 0);
        } else {
            if (mq_adc_flag) {
                led_pwm = map(mq_12b_adc_reg, 0, 2200, 0, 1000);
                LL_TIM_OC_SetCompareCH1(TIM1, led_pwm);
            } else {
                LL_TIM_OC_SetCompareCH1(TIM1, 0);
            }
        }
        // mq work status switch
        switch (mq_cfg_reg) {
            case MODE_OFF:
                LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
                if (mq_work_mode_switch == 1) {
                    mq_work_mode_switch = 0;
                    mq_adc_flag         = 0;
                }
                break;
            case MODE_HEAT:
                LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
                if (mq_work_mode_switch == 1) {
                    mq_work_mode_switch = 0;
                    mq_start_time       = HAL_GetTick();
                    mq_adc_flag         = 0;
                } else {
                    if (mq_adc_flag == 0) {
                        if (HAL_GetTick() - mq_start_time > HEAT_DURATION) {
                            mq_adc_flag = 1;
                        }
                    }
                }
                break;
            case MODE_PIN_SWITCH:
                if (mq_work_mode_switch == 1) {
                    mq_work_mode_switch = 0;
                    mq_start_time       = HAL_GetTick();
                    mq_level_status     = HIGH_LEVEL;
                    mq_adc_flag         = 0;
                }
                switch (mq_level_status) {
                    case HIGH_LEVEL:
                        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
                        if (mq_adc_flag == 0) {
                            if (HAL_GetTick() - mq_start_time > HEAT_DURATION) {
                                mq_adc_flag = 1;
                            }
                        }
                        if (HAL_GetTick() - mq_start_time > high_level_time) {
                            mq_level_status = LOW_LEVEL;
                            mq_start_time   = HAL_GetTick();
                            mq_adc_flag     = 0;
                        }
                        break;
                    case LOW_LEVEL:
                        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
                        if (HAL_GetTick() - mq_start_time > low_level_time) {
                            mq_level_status = HIGH_LEVEL;
                            mq_start_time   = HAL_GetTick();
                        }
                        break;
                }
                break;
            default:
                break;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        LL_IWDG_ReloadCounter(IWDG);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
    }

    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1) {
    }

    /* LSI configuration and activation */
    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() != 1) {
    }

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while (LL_RCC_PLL_IsReady() != 1) {
    }

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(64000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint64_t adc_total[ADC_CHANNEL_NUMS] = {0};
    uint32_t v_refint_actual             = 0;
    float ntc_resistance                 = 0;
    int temp                             = 0;
    HAL_ADC_Stop_DMA(hadc);
    for (uint8_t i = 0; i < ADC_SAMPLES_NUMS; i++) {
        adc_total[i % ADC_CHANNEL_NUMS] += adc_value_buf[i];
    }

    // Calculate actual VREFINT (in mV), using calibration value
    v_refint_actual = (uint32_t)((3000.0f * VREFINT_CALIB) / ((float)adc_total[2] / ADC_AVERAGE_NUMS));

    // Update MQ channel ADC values
    adc_12bit      = adc_total[0] / ADC_AVERAGE_NUMS;
    mq_8b_adc_reg  = map(adc_12bit, 0, 4095, 0, 255);
    mq_12b_adc_reg = adc_12bit;

    // Update NTC channel ADC values
    adc_12bit       = adc_total[1] / ADC_AVERAGE_NUMS;
    ntc_8b_adc_reg  = map(adc_12bit, 0, 4095, 0, 255);
    ntc_12b_adc_reg = adc_12bit;

    // Update voltage values (in mV)
    internal_reference_voltage = v_refint_actual;
    mq_voltage         = (uint32_t)((v_refint_actual * (float)mq_12b_adc_reg) / 4095.0f) * 2;
    ntc_voltage        = (uint32_t)((v_refint_actual * (float)ntc_12b_adc_reg) / 4095.0f);

    // Calculate NTC resistance using voltage divider formula
    ntc_resistance = (2000.0f * internal_reference_voltage) / ntc_voltage - 2000.0f;

    // Convert resistance to integer and store as 2-byte register value
    temp                      = (int)ntc_resistance;
    ntc_resistance_val_reg[1] = (uint8_t)(temp >> 8);    // High byte
    ntc_resistance_val_reg[0] = (uint8_t)(temp & 0xFF);  // Low byte

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value_buf, ADC_SAMPLES_NUMS);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
