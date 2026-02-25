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
#include "dma.h"
#include "gpio.h"
#include "scope_stream.h"
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
static const uint32_t kVdivOptions_mV[] = {100U,  200U,  500U,  1000U,
                                           2000U, 5000U, 10000U};
static const uint32_t kSdivOptions_us[] = {
    5U,    10U,   20U,   50U,    100U,   200U,   500U,
    1000U, 2000U, 5000U, 10000U, 20000U, 50000U, 100000U};

static scope_cfg_t g_scope_cfg = {
    .ch1_enabled = 1U,
    .ch2_enabled = 1U,
    .ch1_vdiv_idx = 2U, // 1.00 V/div
    .ch2_vdiv_idx = 2U, // 1.00 V/div
    .sdiv_idx = 2U,     // 200 us/div
    // trigger byte: each encoder step maps to 16 ADC codes, so 0..255 ->
    // 0..4080.
    .trig_idx = 128U,
};

static bool g_scope_cfg_dirty = true;
static bool g_btn1_prev = false;
static bool g_btn4_prev = false;

static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t lastPoll = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Apply a signed encoder delta to a menu index and clamp to valid bounds.
static int clamp_idx_with_delta(int current, int delta, int min, int max) {
  int next = current + delta;
  if (next < min) {
    next = min;
  } else if (next > max) {
    next = max;
  }
  return next;
}

// USB CDC RX hook; command parsing can be implemented here later.
void ScopeCtl_OnUsbRx(const uint8_t *buf, uint32_t len) {
  (void)buf;
  (void)len;
}

// Apply front-panel inputs to config state:
// ENC1 -> CH1 V/div, ENC3 -> S/div, ENC4 -> CH2 V/div,
// BTN1/BTN4 press-edge toggles channel enable.
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

  if (g_enc2.delta != 0) {
    int next = clamp_idx_with_delta((int)g_scope_cfg.trig_idx,
                                    (int)g_enc2.delta, 0, 255);
    if ((uint8_t)next != g_scope_cfg.trig_idx) {
      g_scope_cfg.trig_idx = (uint8_t)next;
      changed = true;
    }
    g_enc2.delta = 0;
    g_enc2.dirty = false;
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

// Debug helper: stream raw encoder counts/deltas every 10 ms.
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

// Debug helper: stream debounced button states on change (max every 20 ms).
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
  MX_DMA_Init();
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
  ScopeStream_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if ((HAL_GetTick() - s_last_heartbeat_ms) >= 1U) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
      s_last_heartbeat_ms = HAL_GetTick();
    }

    uint32_t now = HAL_GetTick();
    if (now != lastPoll) {
      lastPoll = now;
      Encoders_Poll_1ms();
      Buttons_Poll_1ms();
      ScopeCtl_FromFrontPanel_1ms();
    }
    ScopeStream_Task(&g_scope_cfg);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
