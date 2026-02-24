/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SCOPE_STREAM_H__
#define __SCOPE_STREAM_H__

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Packet format sent over USB CDC data endpoint (bulk transfer under CDC):
 *
 * Offset  Size  Field
 * 0       2      magic = 0xA55A
 * 2       1      header_size_bytes
 * 3       2      sample_count
 * 5       1      trigger
 * 6       1      channel config (ch1/ch2 en & v/div)
 * 7       1      time config (s/div and decimation)
 * 8       8      reserved
 * 16      2*N    samples
 */

#define SCOPE_STREAM_MAGIC ((uint16_t)0xA55AU)
#define SCOPE_STREAM_ADC_BUFFER_SAMPLES (8192U)
#define SCOPE_STREAM_FRAME_SAMPLES (512U)
#define SCOPE_STREAM_HEADER_BYTES (16U)
#define SCOPE_STREAM_CHANNEL_COUNT (2U)
#define SCOPE_STREAM_PAYLOAD_BYTES                                             \
  (SCOPE_STREAM_FRAME_SAMPLES * SCOPE_STREAM_CHANNEL_COUNT * sizeof(uint16_t))
#define SCOPE_STREAM_PACKET_MAX_BYTES (SCOPE_STREAM_HEADER_BYTES + SCOPE_STREAM_PAYLOAD_BYTES)

#define SCOPE_STREAM_CH_CONFIG_CH1_EN (1U << 7)
#define SCOPE_STREAM_CH_CONFIG_CH1_VDIV_POS (4U)
#define SCOPE_STREAM_CH_CONFIG_CH1_VDIV_MASK ((uint8_t)0x70U)
#define SCOPE_STREAM_CH_CONFIG_CH2_EN (1U << 3)
#define SCOPE_STREAM_CH_CONFIG_CH2_VDIV_POS (0U)
#define SCOPE_STREAM_CH_CONFIG_CH2_VDIV_MASK ((uint8_t)0x07)

#define SCOPE_STREAM_TIME_CONFIG_SDIV_POS (3U)
#define SCOPE_STREAM_TIME_CONFIG_SDIV_MASK ((uint8_t)0xF8U)
#define SCOPE_STREAM_TIME_CONFIG_DEC_POS (0U)
#define SCOPE_STREAM_TIME_CONFIG_DEC_MASK ((uint8_t)0x07U)

#define SCOPE_STREAM_META_TRIGGER_INDEX_MASK ((uint64_t)0x0000FFFFULL)
#define SCOPE_STREAM_META_TRIGGER_FOUND_BIT ((uint64_t)1ULL << 16U)

typedef struct __attribute__((packed)) {
  uint16_t magic;            // 2 bytes
  uint8_t header_size_bytes; // 1 byte
  uint16_t sample_count;     // 2 bytes
  uint8_t trigger;           // 1 byte
  uint8_t ch_config;         // 1 byte
  uint8_t time_config;       // 1 byte
  uint64_t reserved0;        // 8 bytes
} ScopeStreamHeader;

void ScopeStream_Init(void);
void ScopeStream_Task(const scope_cfg_t *cfg);
void ScopeStream_SetStreamingEnabled(bool enabled);

#ifdef __cplusplus
}
#endif /* __SCOPE_STREAM_H__ */

#endif
