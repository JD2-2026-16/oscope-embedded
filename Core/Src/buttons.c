#include "buttons.h"
#include "main.h" // has GPIO port/pin defines from CubeMX
#include "stm32g4xx_hal.h"

button_t g_btn1 = {0}, g_btn2 = {0}, g_btn3 = {0}, g_btn4 = {0};

// ---- CONFIG: set these to your actual button pins ----
// Logical order: BTN1, BTN2, BTN3, BTN4

#define BTN1_PORT GPIOA
#define BTN1_PIN GPIO_PIN_15

#define BTN2_PORT GPIOC
#define BTN2_PIN GPIO_PIN_10

#define BTN3_PORT GPIOC
#define BTN3_PIN GPIO_PIN_11 // PC11

#define BTN4_PORT GPIOC
#define BTN4_PIN GPIO_PIN_12 // PC12
// ------------------------------------------------------

static inline bool raw_pressed(GPIO_TypeDef *port, uint16_t pin) {
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET); // active-low
}

// Simple integrator debounce: 0..MAX, pressed when saturated
#define DB_MAX 8

static void debounce(button_t *b, bool raw) {
  if (raw) {
    if (b->integrator < DB_MAX)
      b->integrator++;
  } else {
    if (b->integrator > 0)
      b->integrator--;
  }

  bool new_stable = (b->integrator == DB_MAX);
  if (new_stable != b->stable) {
    b->stable = new_stable;
    b->changed = true;
  }
}

void Buttons_Init(void) {
  // nothing required; GPIO already configured by MX_GPIO_Init()
}

void Buttons_Poll_1ms(void) {
  debounce(&g_btn1, raw_pressed(BTN1_PORT, BTN1_PIN));
  debounce(&g_btn2, raw_pressed(BTN2_PORT, BTN2_PIN));
  debounce(&g_btn3, raw_pressed(BTN3_PORT, BTN3_PIN));
  debounce(&g_btn4, raw_pressed(BTN4_PORT, BTN4_PIN));
}