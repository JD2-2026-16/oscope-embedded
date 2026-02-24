#include "encoder.h"
#include "tim.h"

encoder_t g_enc1 = {0}, g_enc2 = {0}, g_enc3 = {0}, g_enc4 = {0};

static uint16_t p1, p2, p3, p4;
static int16_t r1, r2, r3, r4;

// Number of timer counts that should map to one logical UI step.
#define ENC_COUNTS_PER_STEP 2

static inline int16_t diff16(uint16_t now, uint16_t prev) {
  return (int16_t)(now - prev); // wrap-safe for 16-bit
}

static int16_t normalize_step_delta(int16_t raw_delta, int16_t *remainder) {
  int32_t acc = (int32_t)(*remainder) + (int32_t)raw_delta;
  int16_t steps = (int16_t)(acc / ENC_COUNTS_PER_STEP);
  *remainder = (int16_t)(acc - ((int32_t)steps * ENC_COUNTS_PER_STEP));
  return steps;
}

void Encoders_Init(void) {
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // ENC1
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // ENC2
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // ENC3
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // ENC4

  p1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  p2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  p3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
  p4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  r1 = r2 = r3 = r4 = 0;
}

void Encoders_Poll_1ms(void) {
  uint16_t n1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint16_t n2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  uint16_t n3 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
  uint16_t n4 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  int16_t d1 = diff16(n1, p1);
  p1 = n1;
  int16_t d2 = diff16(n2, p2);
  p2 = n2;
  int16_t d3 = diff16(n3, p3);
  p3 = n3;
  int16_t d4 = diff16(n4, p4);
  p4 = n4;

  d1 = normalize_step_delta(d1, &r1);
  d2 = normalize_step_delta(d2, &r2);
  d3 = normalize_step_delta(d3, &r3);
  d4 = normalize_step_delta(d4, &r4);

  if (d1) {
    g_enc1.count += d1;
    g_enc1.delta += d1;
    g_enc1.dirty = true;
  }
  if (d2) {
    g_enc2.count += d2;
    g_enc2.delta += d2;
    g_enc2.dirty = true;
  }
  if (d3) {
    g_enc3.count += d3;
    g_enc3.delta += d3;
    g_enc3.dirty = true;
  }
  if (d4) {
    g_enc4.count += d4;
    g_enc4.delta += d4;
    g_enc4.dirty = true;
  }
}
