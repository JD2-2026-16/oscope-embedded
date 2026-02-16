/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"
#include "tim.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buttons.h"
#include "usbd_cdc_if.h"
#include <encoder.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const uint16_t kVdivOptions_mV[] = {200U, 500U, 1000U, 2000U, 5000U};
static const uint32_t kSdivOptions_us[] = {50U,   100U,  200U,  500U,
                                           1000U, 2000U, 5000U, 10000U};

typedef struct {
  uint8_t ch1_enabled;
  uint8_t ch2_enabled;
  uint8_t ch1_vdiv_idx;
  uint8_t ch2_vdiv_idx;
  uint8_t sdiv_idx;
} scope_cfg_t;

static scope_cfg_t g_scope_cfg = {
    .ch1_enabled = 1U,
    .ch2_enabled = 1U,
    .ch1_vdiv_idx = 2U, // 1.00 V/div
    .ch2_vdiv_idx = 2U, // 1.00 V/div
    .sdiv_idx = 2U,     // 200 us/div
};

static bool g_scope_cfg_dirty = true;
static bool g_btn1_prev = false;
static bool g_btn4_prev = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t ReadADC_Blocking(ADC_HandleTypeDef *hadc) {
  HAL_ADC_Start(hadc);
  if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
    HAL_ADC_Stop(hadc);
    return 0xFFFF; // error marker
  }
  uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  return v;
}

static int clamp_idx_with_delta(int current, int delta, int min, int max) {
  int next = current + delta;
  if (next < min) {
    next = min;
  } else if (next > max) {
    next = max;
  }
  return next;
}

static int find_nearest_u32_idx(const uint32_t *arr, int count,
                                uint32_t value) {
  int best_idx = 0;
  uint32_t best_err = (arr[0] > value) ? (arr[0] - value) : (value - arr[0]);
  for (int i = 1; i < count; ++i) {
    uint32_t err = (arr[i] > value) ? (arr[i] - value) : (value - arr[i]);
    if (err < best_err) {
      best_err = err;
      best_idx = i;
    }
  }
  return best_idx;
}

static int find_nearest_u16_idx(const uint16_t *arr, int count,
                                uint32_t value) {
  int best_idx = 0;
  uint32_t best_err = (arr[0] > value) ? ((uint32_t)arr[0] - value)
                                       : (value - (uint32_t)arr[0]);
  for (int i = 1; i < count; ++i) {
    uint32_t v = (uint32_t)arr[i];
    uint32_t err = (v > value) ? (v - value) : (value - v);
    if (err < best_err) {
      best_err = err;
      best_idx = i;
    }
  }
  return best_idx;
}

static void ScopeCfg_SetVdivIdx(uint8_t channel, uint8_t idx) {
  uint8_t max_idx =
      (uint8_t)(sizeof(kVdivOptions_mV) / sizeof(kVdivOptions_mV[0]) - 1U);
  if (idx > max_idx) {
    idx = max_idx;
  }

  if (channel == 1U) {
    if (g_scope_cfg.ch1_vdiv_idx != idx) {
      g_scope_cfg.ch1_vdiv_idx = idx;
      g_scope_cfg_dirty = true;
    }
  } else if (channel == 2U) {
    if (g_scope_cfg.ch2_vdiv_idx != idx) {
      g_scope_cfg.ch2_vdiv_idx = idx;
      g_scope_cfg_dirty = true;
    }
  }
}

static void ScopeCfg_SetSdivIdx(uint8_t idx) {
  uint8_t max_idx =
      (uint8_t)(sizeof(kSdivOptions_us) / sizeof(kSdivOptions_us[0]) - 1U);
  if (idx > max_idx) {
    idx = max_idx;
  }
  if (g_scope_cfg.sdiv_idx != idx) {
    g_scope_cfg.sdiv_idx = idx;
    g_scope_cfg_dirty = true;
  }
}

void ScopeCtl_OnUsbRx(const uint8_t *buf, uint32_t len) {
  static char line[64];
  static uint8_t line_len = 0;

  for (uint32_t i = 0; i < len; ++i) {
    char c = (char)buf[i];
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line[line_len] = '\0';
      if (line_len > 0U) {
        uint32_t value = 0U;
        char *end = NULL;
        if (strncmp(line, "SET CH1_VDIV_MV ", 16) == 0) {
          value = (uint32_t)strtoul(&line[16], &end, 10);
          if ((end != &line[16]) && (*end == '\0')) {
            int idx = find_nearest_u16_idx(
                kVdivOptions_mV,
                (int)(sizeof(kVdivOptions_mV) / sizeof(kVdivOptions_mV[0])),
                value);
            ScopeCfg_SetVdivIdx(1U, (uint8_t)idx);
          }
        } else if (strncmp(line, "SET CH2_VDIV_MV ", 16) == 0) {
          value = (uint32_t)strtoul(&line[16], &end, 10);
          if ((end != &line[16]) && (*end == '\0')) {
            int idx = find_nearest_u16_idx(
                kVdivOptions_mV,
                (int)(sizeof(kVdivOptions_mV) / sizeof(kVdivOptions_mV[0])),
                value);
            ScopeCfg_SetVdivIdx(2U, (uint8_t)idx);
          }
        } else if (strncmp(line, "SET SDIV_US ", 12) == 0) {
          value = (uint32_t)strtoul(&line[12], &end, 10);
          if ((end != &line[12]) && (*end == '\0')) {
            int idx = find_nearest_u32_idx(
                kSdivOptions_us,
                (int)(sizeof(kSdivOptions_us) / sizeof(kSdivOptions_us[0])),
                value);
            ScopeCfg_SetSdivIdx((uint8_t)idx);
          }
        }
      }
      line_len = 0U;
      continue;
    }

    if (line_len < (uint8_t)(sizeof(line) - 1U)) {
      line[line_len++] = c;
    } else {
      line_len = 0U;
    }
  }
}

static void ScopeCtl_FromFrontPanel_1ms(void) {
  bool changed = false;

  if (g_enc1.delta != 0) {
    int max_idx =
        (int)(sizeof(kVdivOptions_mV) / sizeof(kVdivOptions_mV[0])) - 1;
    int next = clamp_idx_with_delta((int)g_scope_cfg.ch1_vdiv_idx,
                                    (int)g_enc1.delta, 0, max_idx);
    if ((uint8_t)next != g_scope_cfg.ch1_vdiv_idx) {
      g_scope_cfg.ch1_vdiv_idx = (uint8_t)next;
      changed = true;
    }
    g_enc1.delta = 0;
    g_enc1.dirty = false;
  }

  if (g_enc4.delta != 0) {
    int max_idx =
        (int)(sizeof(kVdivOptions_mV) / sizeof(kVdivOptions_mV[0])) - 1;
    int next = clamp_idx_with_delta((int)g_scope_cfg.ch2_vdiv_idx,
                                    (int)g_enc4.delta, 0, max_idx);
    if ((uint8_t)next != g_scope_cfg.ch2_vdiv_idx) {
      g_scope_cfg.ch2_vdiv_idx = (uint8_t)next;
      changed = true;
    }
    g_enc4.delta = 0;
    g_enc4.dirty = false;
  }

  if (g_enc3.delta != 0) {
    int max_idx =
        (int)(sizeof(kSdivOptions_us) / sizeof(kSdivOptions_us[0])) - 1;
    int next = clamp_idx_with_delta((int)g_scope_cfg.sdiv_idx,
                                    (int)g_enc3.delta, 0, max_idx);
    if ((uint8_t)next != g_scope_cfg.sdiv_idx) {
      g_scope_cfg.sdiv_idx = (uint8_t)next;
      changed = true;
    }
    g_enc3.delta = 0;
    g_enc3.dirty = false;
  }

  // Toggle on press-edge.
  if (g_btn1.stable && !g_btn1_prev) {
    g_scope_cfg.ch1_enabled = (uint8_t)!g_scope_cfg.ch1_enabled;
    changed = true;
  }
  if (g_btn4.stable && !g_btn4_prev) {
    g_scope_cfg.ch2_enabled = (uint8_t)!g_scope_cfg.ch2_enabled;
    changed = true;
  }
  g_btn1_prev = g_btn1.stable;
  g_btn4_prev = g_btn4.stable;

  if (changed) {
    g_scope_cfg_dirty = true;
  }
}

void USB_Report_ScopeCfg_50ms(void) {
  static uint32_t last = 0U;
  uint32_t now = HAL_GetTick();
  if ((now - last) < 50U) {
    return;
  }
  last = now;

  if (!g_scope_cfg_dirty) {
    return;
  }

  char msg[120];
  int n = snprintf(
      msg, sizeof(msg),
      "cfg ch1_en:%u ch2_en:%u ch1_vdiv_mv:%u ch2_vdiv_mv:%u sdiv_us:%lu\r\n",
      (unsigned)g_scope_cfg.ch1_enabled, (unsigned)g_scope_cfg.ch2_enabled,
      (unsigned)kVdivOptions_mV[g_scope_cfg.ch1_vdiv_idx],
      (unsigned)kVdivOptions_mV[g_scope_cfg.ch2_vdiv_idx],
      (unsigned long)kSdivOptions_us[g_scope_cfg.sdiv_idx]);
  if (n > 0) {
    if (CDC_Transmit_FS((uint8_t *)msg, (uint16_t)n) == USBD_OK) {
      g_scope_cfg_dirty = false;
    }
  }
}

void USB_Report_Encoders_10ms(void) {
  static uint32_t last = 0;
  uint32_t now = HAL_GetTick();
  if ((now - last) < 10) {
    return;
  }
  last = now;

  char msg[128];
  int n = snprintf(msg, sizeof(msg),
                   "e1:%ld\te2:%ld\te3:%ld\te4:%ld d:%d,%d,%d,%d\r\n",
                   (long)g_enc1.count, (long)g_enc2.count, (long)g_enc3.count,
                   (long)g_enc4.count, (int)g_enc1.delta, (int)g_enc2.delta,
                   (int)g_enc3.delta, (int)g_enc4.delta);
  g_enc1.dirty = g_enc2.dirty = g_enc3.dirty = g_enc4.dirty = false;
  g_enc1.delta = g_enc2.delta = g_enc3.delta = g_enc4.delta = 0;

  CDC_Transmit_FS((uint8_t *)msg, (uint16_t)n);
}

void USB_Report_Buttons_20ms(void) {
  static uint32_t last = 0;
  uint32_t now = HAL_GetTick();
  if (now - last < 20)
    return;
  last = now;

  // Only print if something changed (optional, but nice)
  if (!(g_btn1.changed || g_btn2.changed || g_btn3.changed || g_btn4.changed))
    return;

  char msg[80];
  int n = snprintf(msg, sizeof msg, "btn:%d,%d,%d,%d\r\n", (int)g_btn1.stable,
                   (int)g_btn2.stable, (int)g_btn3.stable, (int)g_btn4.stable);

  g_btn1.changed = g_btn2.changed = g_btn3.changed = g_btn4.changed = false;
  (void)CDC_Transmit_FS((uint8_t *)msg, (uint16_t)n);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  Encoders_Init();
  Buttons_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    static uint32_t lastPoll = 0;
    uint32_t now = HAL_GetTick();
    if (now != lastPoll) {
      lastPoll = now;
      Encoders_Poll_1ms();
      Buttons_Poll_1ms();
      ScopeCtl_FromFrontPanel_1ms();
    }

    // USB_Report_Encoders_10ms();
    // USB_Report_Buttons_20ms();
    USB_Report_ScopeCfg_50ms();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
