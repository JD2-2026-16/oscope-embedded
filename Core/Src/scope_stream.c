#include "scope_stream.h"

#include <string.h>

#include "adc.h"
#include "usbd_cdc_if.h"

// Limit USB packet transmit cadence.
// 40 ms -> ~25 FPS update rate
#define SCOPE_STREAM_TX_PERIOD_MS (40U)
// Keep copied windows this far behind the DMA write head to avoid in-flight
// sample corruption.
#define SCOPE_STREAM_DMA_GUARD_SAMPLES (64U)

// Circular DMA targets for ADC1/ADC2 continuous conversions.
// Each buffer is written by DMA in circular mode and read by the stream task.
static uint16_t s_adc1_dma_buffer[SCOPE_STREAM_ADC_BUFFER_SAMPLES];
static uint16_t s_adc2_dma_buffer[SCOPE_STREAM_ADC_BUFFER_SAMPLES];

// One full USB packet buffer: [header][interleaved payload].
static uint8_t s_tx_packet[SCOPE_STREAM_PACKET_MAX_BYTES];

// Runtime stream state.
static bool s_streaming_enabled = true;
static bool s_dma_started = false;
static uint32_t s_last_tx_ms = 0U;

// Internal helpers used by the periodic stream task.
static uint8_t ScopeStream_GetDecimationShift(uint8_t sdiv_idx);
static uint16_t ScopeStream_GetAdcWriteIndex(const ADC_HandleTypeDef *hadc);
static bool ScopeStream_CopyLatestInterleaved(uint8_t *dst,
                                              uint16_t sample_count,
                                              uint8_t decimation_shift,
                                              uint16_t trigger_code,
                                              uint16_t *trigger_index_out);

// Initialize ADC calibration and start both ADC DMA streams into circular
// buffers. If any step fails, the function returns early and leaves streaming
// disabled.
void ScopeStream_Init(void) {
  // Calibrate each ADC in single-ended mode for best offset accuracy.
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    return;
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    return;
  }

  // Start DMA for both ADCs. Buffer length matches the configured circular
  // depth.
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_dma_buffer,
                        SCOPE_STREAM_ADC_BUFFER_SAMPLES) != HAL_OK) {
    return;
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_dma_buffer,
                        SCOPE_STREAM_ADC_BUFFER_SAMPLES) != HAL_OK) {
    // If ADC2 start fails, stop ADC1 DMA so both channels remain in sync.
    (void)HAL_ADC_Stop_DMA(&hadc1);
    return;
  }

  // Both DMA streams are alive; periodic task may now package/transmit frames.
  s_dma_started = true;
}

// Runtime enable/disable gate for packet transmission.
void ScopeStream_SetStreamingEnabled(bool enabled) {
  s_streaming_enabled = enabled;
}

// Periodic producer for one scope frame packet.
// This function:
// 1) Converts front-panel config into packet header fields.
// 2) Copies the newest triggered (or rolling) data window from DMA buffers.
// 3) Sends the packet over USB CDC if endpoint is available.
void ScopeStream_Task(const scope_cfg_t *cfg) {
  ScopeStreamHeader header = {0};
  uint8_t decimation_shift;
  uint16_t trigger_code;
  uint16_t trigger_index;
  bool trigger_found;
  uint32_t now_ms;
  uint16_t tx_len;

  // Do nothing if configuration is missing, stream is disabled, or ADC DMA is
  // not ready.
  if ((cfg == NULL) || !s_streaming_enabled || !s_dma_started) {
    return;
  }

  // Throttle TX rate to avoid saturating USB and UI consumer.
  now_ms = HAL_GetTick();
  if ((now_ms - s_last_tx_ms) < SCOPE_STREAM_TX_PERIOD_MS) {
    return;
  }

  // Map selected S/div option to a decimation shift:
  // decimation = 2^shift samples between output points.
  decimation_shift = ScopeStream_GetDecimationShift(cfg->sdiv_idx);

  // Trigger knob is an 8-bit value where each encoder tick equals 16 ADC codes.
  // Convert to 12-bit ADC code space and clamp to ADC max.
  trigger_code = (uint16_t)((uint16_t)cfg->trig_idx * 16U);
  if (trigger_code > 4095U) {
    trigger_code = 4095U;
  }

  // Fill fixed header fields.
  header.magic = SCOPE_STREAM_MAGIC;
  header.header_size_bytes = SCOPE_STREAM_HEADER_BYTES;
  header.sample_count = SCOPE_STREAM_FRAME_SAMPLES;
  header.trigger = cfg->trig_idx;
  header.ch_config = 0U;

  // Pack channel enable bits.
  if (cfg->ch1_enabled) {
    header.ch_config |= SCOPE_STREAM_CH_CONFIG_CH1_EN;
  }
  if (cfg->ch2_enabled) {
    header.ch_config |= SCOPE_STREAM_CH_CONFIG_CH2_EN;
  }

  // Pack V/div indices.
  header.ch_config |=
      (uint8_t)((cfg->ch1_vdiv_idx << SCOPE_STREAM_CH_CONFIG_CH1_VDIV_POS) &
                SCOPE_STREAM_CH_CONFIG_CH1_VDIV_MASK);
  header.ch_config |=
      (uint8_t)((cfg->ch2_vdiv_idx << SCOPE_STREAM_CH_CONFIG_CH2_VDIV_POS) &
                SCOPE_STREAM_CH_CONFIG_CH2_VDIV_MASK);

  // Pack S/div index + effective decimation in time_config.
  header.time_config =
      (uint8_t)((cfg->sdiv_idx << SCOPE_STREAM_TIME_CONFIG_SDIV_POS) &
                SCOPE_STREAM_TIME_CONFIG_SDIV_MASK);
  header.time_config |=
      (uint8_t)((decimation_shift << SCOPE_STREAM_TIME_CONFIG_DEC_POS) &
                SCOPE_STREAM_TIME_CONFIG_DEC_MASK);

  // Build interleaved payload into the packet directly.
  // The helper returns whether a trigger crossing was found and where it
  // landed.
  trigger_index = 0U;
  trigger_found = ScopeStream_CopyLatestInterleaved(
      &s_tx_packet[sizeof(header)], SCOPE_STREAM_FRAME_SAMPLES,
      decimation_shift, trigger_code, &trigger_index);

  // reserved0 is used as metadata transport:
  // - bit 16: trigger found flag
  // - bits 15:0: trigger index in output samples
  header.reserved0 = 0U;
  if (trigger_found) {
    header.reserved0 |=
        (uint64_t)trigger_index & SCOPE_STREAM_META_TRIGGER_INDEX_MASK;
    header.reserved0 |= SCOPE_STREAM_META_TRIGGER_FOUND_BIT;
  }

  // Write header at packet start after payload generation.
  memcpy(s_tx_packet, &header, sizeof(header));

  // Attempt non-blocking USB CDC transmit.
  // Timestamp only updates on successful queueing.
  tx_len = (uint16_t)(sizeof(header) + SCOPE_STREAM_PAYLOAD_BYTES);
  if (CDC_Transmit_FS(s_tx_packet, tx_len) == USBD_OK) {
    s_last_tx_ms = now_ms;
  }
}

// Coarse decimation map from S/div index.
// Higher time/div means we skip more raw samples between plotted points.
static uint8_t ScopeStream_GetDecimationShift(uint8_t sdiv_idx) {
  // 14 S/div options map into 3-bit decimation field (0..7).
  // This keeps wider timebases changing visibly instead of flattening out.
  if (sdiv_idx >= 12U) {
    return 7U;
  }
  if (sdiv_idx >= 11U) {
    return 6U;
  }
  if (sdiv_idx >= 10U) {
    return 5U;
  }
  if (sdiv_idx >= 8U) {
    return 4U;
  }
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

// Read current producer position of a circular DMA stream.
// HAL DMA counter returns "remaining transfers", so write index is:
// buffer_len - remaining (wrapped to buffer length).
static uint16_t ScopeStream_GetAdcWriteIndex(const ADC_HandleTypeDef *hadc) {
  uint32_t remaining;
  if ((hadc == NULL) || (hadc->DMA_Handle == NULL)) {
    return 0U;
  }

  remaining = __HAL_DMA_GET_COUNTER(hadc->DMA_Handle);
  return (uint16_t)(SCOPE_STREAM_ADC_BUFFER_SAMPLES - remaining) %
         SCOPE_STREAM_ADC_BUFFER_SAMPLES;
}

// Copy one output frame from live circular DMA buffers into interleaved
// payload.
//
// Behavior:
// - If trigger crossing is found on CH1 source, frame starts so crossing
// appears
//   at a fixed pre-trigger position (~20% from left).
// - If no crossing is found, frame starts at newest rolling window.
//
// Output format per sample i:
//   dst[4*i + 0..1] = CH1 (little-endian uint16)
//   dst[4*i + 2..3] = CH2 (little-endian uint16)
//
// Returns:
// - true if trigger crossing found, false otherwise.
// - trigger_index_out (if non-null) receives trigger location in output sample
// space.
static bool ScopeStream_CopyLatestInterleaved(uint8_t *dst,
                                              uint16_t sample_count,
                                              uint8_t decimation_shift,
                                              uint16_t trigger_code,
                                              uint16_t *trigger_index_out) {
  uint16_t write_idx1;
  uint16_t write_idx2;
  uint16_t anchor_idx;
  uint16_t decimation;
  uint32_t span;
  uint32_t search_span;
  uint32_t search_start;
  int32_t trigger_offset;
  uint32_t left_margin_raw;
  uint32_t right_margin_raw;
  uint32_t trigger_idx_abs;
  uint32_t search_last_offset;
  uint32_t guard_raw;
  uint32_t start_idx;
  bool trigger_found;

  if (trigger_index_out != NULL) {
    *trigger_index_out = 0U;
  }

  // Clamp request to protocol frame limit and reject empty copy.
  if (sample_count > SCOPE_STREAM_FRAME_SAMPLES) {
    sample_count = SCOPE_STREAM_FRAME_SAMPLES;
  }
  if (sample_count == 0U) {
    return false;
  }

  decimation = (uint16_t)(1U << decimation_shift);

  // Raw span is how many DMA samples the frame consumes before decimation.
  span = (uint32_t)sample_count * (uint32_t)decimation;
  if (span > SCOPE_STREAM_ADC_BUFFER_SAMPLES) {
    span = SCOPE_STREAM_ADC_BUFFER_SAMPLES;
  }

  // Use the older write index as a safe anchor to avoid reading data that is
  // still being written.
  write_idx1 = ScopeStream_GetAdcWriteIndex(&hadc1);
  write_idx2 = ScopeStream_GetAdcWriteIndex(&hadc2);
  anchor_idx = (write_idx1 < write_idx2) ? write_idx1 : write_idx2;
  guard_raw = (uint32_t)decimation * SCOPE_STREAM_DMA_GUARD_SAMPLES;
  if (guard_raw >= SCOPE_STREAM_ADC_BUFFER_SAMPLES) {
    guard_raw = SCOPE_STREAM_ADC_BUFFER_SAMPLES - 1U;
  }
  anchor_idx =
      (uint16_t)((anchor_idx + SCOPE_STREAM_ADC_BUFFER_SAMPLES - guard_raw) %
                 SCOPE_STREAM_ADC_BUFFER_SAMPLES);

  // Search in a recent history window (up to 2x frame span) for a trigger
  // crossing.
  search_span = span * 2U;
  if (search_span > SCOPE_STREAM_ADC_BUFFER_SAMPLES) {
    search_span = SCOPE_STREAM_ADC_BUFFER_SAMPLES;
  }
  search_start = (anchor_idx + SCOPE_STREAM_ADC_BUFFER_SAMPLES - search_span) %
                 SCOPE_STREAM_ADC_BUFFER_SAMPLES;
  trigger_offset = -1;

  // Search backwards for the newest rising crossing on CH1 source (ADC2).
  // Small hysteresis (+/-8 codes around threshold) suppresses noise chatter.
  if (search_span > 1U) {
    left_margin_raw = span / 5U; // ~20% pre-trigger
    right_margin_raw = span - left_margin_raw;
    if (right_margin_raw < search_span) {
      search_last_offset = search_span - right_margin_raw;
    } else {
      search_last_offset = 0U;
    }
    for (uint32_t off = search_last_offset; off > 0U; --off) {
      uint32_t idx_prev =
          (search_start + off - 1U) % SCOPE_STREAM_ADC_BUFFER_SAMPLES;
      uint32_t idx_now = (search_start + off) % SCOPE_STREAM_ADC_BUFFER_SAMPLES;
      uint16_t s_prev = s_adc2_dma_buffer[idx_prev];
      uint16_t s_now = s_adc2_dma_buffer[idx_now];
      if ((s_prev + 8U) < trigger_code && s_now >= (trigger_code + 8U)) {
        trigger_offset = (int32_t)off;
        break;
      }
    }
  }

  trigger_found = (trigger_offset >= 0);
  if (trigger_offset >= 0) {
    // Place crossing at fixed pre-trigger location in raw-sample space.
    left_margin_raw = span / 5U; // ~20% pre-trigger
    trigger_idx_abs = (search_start + (uint32_t)trigger_offset) %
                      SCOPE_STREAM_ADC_BUFFER_SAMPLES;
    // Choose frame start so crossing remains visually stable between frames.
    start_idx = (trigger_idx_abs + SCOPE_STREAM_ADC_BUFFER_SAMPLES -
                 (left_margin_raw % SCOPE_STREAM_ADC_BUFFER_SAMPLES)) %
                SCOPE_STREAM_ADC_BUFFER_SAMPLES;
    if (trigger_index_out != NULL) {
      // Convert raw left margin to decimated sample index for GUI alignment
      // metadata.
      uint32_t trigger_sample_index = left_margin_raw / (uint32_t)decimation;
      if (trigger_sample_index >= sample_count) {
        trigger_sample_index = sample_count - 1U;
      }
      *trigger_index_out = (uint16_t)trigger_sample_index;
    }
  } else {
    // No crossing: stream a rolling latest window so waveform continues moving.
    start_idx = (anchor_idx + SCOPE_STREAM_ADC_BUFFER_SAMPLES - span) %
                SCOPE_STREAM_ADC_BUFFER_SAMPLES;
  }

  // Copy decimated samples into interleaved USB payload.
  for (uint16_t i = 0U; i < sample_count; ++i) {
    uint32_t src_idx = (start_idx + ((uint32_t)i * (uint32_t)decimation)) %
                       SCOPE_STREAM_ADC_BUFFER_SAMPLES;
    // Logical channel swap:
    // - protocol CH1 uses ADC2
    // - protocol CH2 uses ADC1
    uint16_t ch1 = s_adc2_dma_buffer[src_idx];
    uint16_t ch2 = s_adc1_dma_buffer[src_idx];
    uint32_t out = (uint32_t)i * 4U;

    dst[out + 0U] = (uint8_t)(ch1 & 0xFFU);
    dst[out + 1U] = (uint8_t)((ch1 >> 8U) & 0xFFU);
    dst[out + 2U] = (uint8_t)(ch2 & 0xFFU);
    dst[out + 3U] = (uint8_t)((ch2 >> 8U) & 0xFFU);
  }

  return trigger_found;
}
