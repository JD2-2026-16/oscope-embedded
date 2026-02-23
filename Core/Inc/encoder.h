/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  volatile int32_t count;
  volatile int16_t delta;
  volatile bool dirty;
} encoder_t;

extern encoder_t g_enc1, g_enc2, g_enc3, g_enc4;

void Encoders_Init(void);
void Encoders_Poll_1ms(void);

#ifdef __cplusplus
}
#endif

#endif
