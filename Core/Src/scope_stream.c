#include "scope_stream.h"

#include <string.h>

#include "adc.h"
#include "usbd_cdc_if.h"

#define SCOPE_STREAM_TX_PERIOD_MS (40U)

static uint16_t s_adc1_dma_buffer[SCOPE_STREAM_ADC_BUFFER_SAMPLES];
static uint16_t s_adc2_dma_buffer[SCOPE_STREAM_ADC_BUFFER_SAMPLES];
static uint8_t s_tx_packet[SCOPE_STREAM_PACKET_MAX_BYTES];

static bool s_streaming_enabled = true;
static bool s_dma_started = false;
static uint32_t s_last_tx_ms = 0U;

static uint8_t ScopeStream_GetDecimationShift(uint8_t sdiv_idx);
static uint16_t ScopeStream_GetAdcWriteIndex(const ADC_HandleTypeDef *hadc);
static void ScopeStream_CopyLatestInterleaved(uint8_t *dst, uint16_t sample_count, uint8_t decimation_shift);

void ScopeStream_Init(void) {
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    return;
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    return;
  }

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma_buffer, SCOPE_STREAM_ADC_BUFFER_SAMPLES) != HAL_OK) {
    return;
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma_buffer, SCOPE_STREAM_ADC_BUFFER_SAMPLES) != HAL_OK) {
    (void)HAL_ADC_Stop_DMA(&hadc1);
    return;
  }

  s_dma_started = true;
}

void ScopeStream_SetStreamingEnabled(bool enabled) { s_streaming_enabled = enabled; }

void ScopeStream_Task(const scope_cfg_t *cfg) {
  ScopeStreamHeader header = {0};
  uint8_t decimation_shift;
  uint32_t now_ms;
  uint16_t tx_len;

  if ((cfg == NULL) || !s_streaming_enabled || !s_dma_started) {
    return;
  }

  now_ms = HAL_GetTick();
  if ((now_ms - s_last_tx_ms) < SCOPE_STREAM_TX_PERIOD_MS) {
    return;
  }

  decimation_shift = ScopeStream_GetDecimationShift(cfg->sdiv_idx);

  header.magic = SCOPE_STREAM_MAGIC;
  header.header_size_bytes = SCOPE_STREAM_HEADER_BYTES;
  header.sample_count = SCOPE_STREAM_FRAME_SAMPLES;
  header.trigger = cfg->trig_idx;
  header.ch_config = 0U;
  if (cfg->ch1_enabled) {
    header.ch_config |= SCOPE_STREAM_CH_CONFIG_CH1_EN;
  }
  if (cfg->ch2_enabled) {
    header.ch_config |= SCOPE_STREAM_CH_CONFIG_CH2_EN;
  }
  header.ch_config |= (uint8_t)((cfg->ch1_vdiv_idx << SCOPE_STREAM_CH_CONFIG_CH1_VDIV_POS) &
                                SCOPE_STREAM_CH_CONFIG_CH1_VDIV_MASK);
  header.ch_config |= (uint8_t)((cfg->ch2_vdiv_idx << SCOPE_STREAM_CH_CONFIG_CH2_VDIV_POS) &
                                SCOPE_STREAM_CH_CONFIG_CH2_VDIV_MASK);

  header.time_config = (uint8_t)((cfg->sdiv_idx << SCOPE_STREAM_TIME_CONFIG_SDIV_POS) &
                                 SCOPE_STREAM_TIME_CONFIG_SDIV_MASK);
  header.time_config |= (uint8_t)((decimation_shift << SCOPE_STREAM_TIME_CONFIG_DEC_POS) &
                                  SCOPE_STREAM_TIME_CONFIG_DEC_MASK);
  header.reserved0 = 0U;

  memcpy(s_tx_packet, &header, sizeof(header));
  ScopeStream_CopyLatestInterleaved(&s_tx_packet[sizeof(header)], SCOPE_STREAM_FRAME_SAMPLES, decimation_shift);

  tx_len = (uint16_t)(sizeof(header) + SCOPE_STREAM_PAYLOAD_BYTES);
  if (CDC_Transmit_FS(s_tx_packet, tx_len) == USBD_OK) {
    s_last_tx_ms = now_ms;
  }
}

static uint8_t ScopeStream_GetDecimationShift(uint8_t sdiv_idx) {
  if (sdiv_idx >= 6U) {
    return 3U;
  }
  if (sdiv_idx >= 4U) {
    return 2U;
  }
  if (sdiv_idx >= 2U) {
    return 1U;
  }
  return 0U;
}

static uint16_t ScopeStream_GetAdcWriteIndex(const ADC_HandleTypeDef *hadc) {
  uint32_t remaining;
  if ((hadc == NULL) || (hadc->DMA_Handle == NULL)) {
    return 0U;
  }

  remaining = __HAL_DMA_GET_COUNTER(hadc->DMA_Handle);
  return (uint16_t)(SCOPE_STREAM_ADC_BUFFER_SAMPLES - remaining) % SCOPE_STREAM_ADC_BUFFER_SAMPLES;
}

static void ScopeStream_CopyLatestInterleaved(uint8_t *dst, uint16_t sample_count, uint8_t decimation_shift) {
  uint16_t write_idx1;
  uint16_t write_idx2;
  uint16_t anchor_idx;
  uint16_t decimation;
  uint32_t span;
  uint32_t start_idx;

  if (sample_count > SCOPE_STREAM_FRAME_SAMPLES) {
    sample_count = SCOPE_STREAM_FRAME_SAMPLES;
  }

  decimation = (uint16_t)(1U << decimation_shift);
  span = (uint32_t)sample_count * (uint32_t)decimation;
  if (span > SCOPE_STREAM_ADC_BUFFER_SAMPLES) {
    span = SCOPE_STREAM_ADC_BUFFER_SAMPLES;
  }

  write_idx1 = ScopeStream_GetAdcWriteIndex(&hadc1);
  write_idx2 = ScopeStream_GetAdcWriteIndex(&hadc2);
  anchor_idx = (write_idx1 < write_idx2) ? write_idx1 : write_idx2;

  start_idx = (anchor_idx + SCOPE_STREAM_ADC_BUFFER_SAMPLES - span) % SCOPE_STREAM_ADC_BUFFER_SAMPLES;

  for (uint16_t i = 0U; i < sample_count; ++i) {
    uint32_t src_idx = (start_idx + ((uint32_t)i * (uint32_t)decimation)) % SCOPE_STREAM_ADC_BUFFER_SAMPLES;
    // Logical channel swap: CH1 is sourced from ADC2, CH2 from ADC1.
    uint16_t ch1 = s_adc2_dma_buffer[src_idx];
    uint16_t ch2 = s_adc1_dma_buffer[src_idx];
    uint32_t out = (uint32_t)i * 4U;

    dst[out + 0U] = (uint8_t)(ch1 & 0xFFU);
    dst[out + 1U] = (uint8_t)((ch1 >> 8U) & 0xFFU);
    dst[out + 2U] = (uint8_t)(ch2 & 0xFFU);
    dst[out + 3U] = (uint8_t)((ch2 >> 8U) & 0xFFU);
  }
}
