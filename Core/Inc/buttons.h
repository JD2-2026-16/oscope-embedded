/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTONS_H__
#define __BUTTONS_H__

#ifdef __cplusplus
extern "C" {
#endif

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool stable;        // debounced state: true = pressed
  bool changed;       // went up/down since last report
  uint8_t integrator; // debounce accumulator
} button_t;

extern button_t g_btn1, g_btn2, g_btn3, g_btn4;

void Buttons_Init(void);
void Buttons_Poll_1ms(void); // call at 1 kHz

#ifdef __cplusplus
}
#endif

#endif
