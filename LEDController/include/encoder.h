#pragma once

#include <stdint.h>
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_log.h"

/* the variable lives in encoder.c */
extern volatile int g_state;
extern volatile int isr_hit;

/* functions that encoder.c provides */
void setup_gpio_pcnt(void);
void pcnt_init(void);
void pcnt_isr_attach(void);
void pcnt_isr(void);