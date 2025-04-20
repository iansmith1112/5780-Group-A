#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "soc/gpio_sig_map.h"

// ---------------------------------------------------------------------------
// REGISTER & MACRO DEFINITIONS
// ---------------------------------------------------------------------------

#define RMT_CH0CONF0_REG           ((volatile uint32_t *)0x3ff56020)
#define RMT_CH0CONF1_REG           ((volatile uint32_t *)0x3ff56024)
#define RMT_CH0CARRIER_DUTY_REG    ((volatile uint32_t *)0x3FF560B0)
#define RMT_INT_ST_REG             ((volatile uint32_t *)0x3FF560A4)
#define RMT_INT_CLR_REG            ((volatile uint32_t *)0x3FF560AC)
#define RMT_INT_ENA_REG            ((volatile uint32_t *)0x3FF560A8)
#define RMT_APB_CONF_REG           ((volatile uint32_t *)0x3FF560F0)
#define RMT_DATA                   ((volatile uint32_t *)0x3FF56800)

#define DPORT_BASE                  0x3ff00000
#define DPORT_PERIP_CLK_EN_REG      (volatile uint32_t *)(DPORT_BASE + 0xc0)

#define GPIO_FUNC4_OUT_SEL_CFG_REG ((volatile uint32_t *)0x3FF44540)
#define GPIO_ENABLE_REG             (volatile uint32_t *)(0x3FF44020)
#define IO_MUX_GPIO4_REG           ((volatile uint32_t *)0x3FF49048)
#define RMT_SIG_OUT0_IDX           87

#define PCNT_BASE                   0x3ff57000
#define PCNT_U0_CNT_REG             (volatile uint32_t *)(PCNT_BASE + 0x60)



#define PCNT_BASE                   0x3ff57000
#define PCNT_U0_CONF0_REG           (volatile uint32_t *)(PCNT_BASE + 0x00)
#define PCNT_U0_CONF1_REG           (volatile uint32_t *)(PCNT_BASE + 0x04)
#define PCNT_U0_CONF2_REG           (volatile uint32_t *)(PCNT_BASE + 0x08)
#define PCNT_U0_CNT_REG             (volatile uint32_t *)(PCNT_BASE + 0x60)
#define PCNT_U0_STATUS_REG          (volatile uint32_t *)(PCNT_BASE + 0x90)
#define PCNT_CTRL_REG               (volatile uint32_t *)(PCNT_BASE + 0xB0)
#define PCNT_INT_RAW_REG            (volatile uint32_t *)(PCNT_BASE + 0x80)
#define PCNT_INT_ST_REG             (volatile uint32_t *)(PCNT_BASE + 0x84)
#define PCNT_INT_ENA_REG            (volatile uint32_t *)(PCNT_BASE + 0x88)
#define PCNT_INT_CLR_REG            (volatile uint32_t *)(PCNT_BASE + 0x8C)
#define IO_MUX_BASE                 0x3ff49000
#define IO_MUX_GPIO2                (volatile uint32_t *)(IO_MUX_BASE + 0x40)
#define IO_MUX_GPIO16               (volatile uint32_t *)(IO_MUX_BASE + 0x4c)
#define GPIO_FUNC_IN_BASE           0x3ff44000
#define GPIO_FUNC39_IN_SEL_CFG      (volatile uint32_t *)(GPIO_FUNC_IN_BASE + 0x1cc)
#define GPIO_FUNC41_IN_SEL_CFG      (volatile uint32_t *)(GPIO_FUNC_IN_BASE + 0x1d4)
#define GPIO_FUNC2_OUT_SEL_CFG      (volatile uint32_t *)(GPIO_FUNC_IN_BASE + 0x538)
#define GPIO_FUNC16_OUT_SEL_CFG     (volatile uint32_t *)(GPIO_FUNC_IN_BASE + 0x570)
#define GPIO_ENABLE_REG             (volatile uint32_t *)(0x3FF44020)
#define GPIO_ENABLE_W1TC_REG        (volatile uint32_t *)(0x3FF44028)
#define DPORT_BASE                  0x3ff00000
#define DPORT_PRO_PCNT_INTR_MAP     (volatile uint32_t *)(DPORT_BASE + 0x1C4)
#define DPORT_PERIP_CLK_EN_REG      (volatile uint32_t *)(DPORT_BASE + 0xc0)
#define DPORT_PERIP_RST_EN_REG      (volatile uint32_t *)(DPORT_BASE + 0xc4)

// ---------------------------------------------------------------------------
// Timing definitions for WS2812 LED (in units of RMT ticks).
// ---------------------------------------------------------------------------
#define T0H    3   // 0.3us (assuming 10MHz RMT clock = 100ns per tick)
#define T0L    8   // 0.8us
#define T1H    8   // 0.8us
#define T1L    4   // 0.4us
#define T_RESET 252 // ~25us * 2 (2 periods setup in rmt_item_t reset)

// ---------------------------------------------------------------------------
// RMT RAM data structure
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t duration0 : 15; 
    uint32_t level0    : 1;  
    uint32_t duration1 : 15; 
    uint32_t level1    : 1;  
} rmt_item32_t;

// RMT items for logical '0' and '1', plus a reset pulse
static rmt_item32_t bit0  = { .duration0 = T0H, .level0=1, .duration1=T0L, .level1=0 };
static rmt_item32_t bit1  = { .duration0 = T1H, .level0=1, .duration1=T1L, .level1=0 };
static rmt_item32_t reset = { .duration0 = T_RESET, .level0=0, .duration1=T_RESET, .level1=0 };

// ---------------------------------------------------------------------------
// Fill RMT memory with WS2812 data for a single RGB LED (24 bits: GRB order).
// ---------------------------------------------------------------------------
void rmt_fill_ws2812_data(uint8_t g, uint8_t r, uint8_t b)
{
    volatile rmt_item32_t* mem = (volatile rmt_item32_t*) RMT_DATA;
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

    // 24 bits total
    for (int i = 23; i >= 0; i--) {
        if (grb & (1 << i)) {
            mem[23 - i] = bit1;  // '1'
        } else {
            mem[23 - i] = bit0;  // '0'
        }
    }
    // Add one more item for the reset
    mem[24] = reset;
}

// ---------------------------------------------------------------------------
// Start RMT transmission
// ---------------------------------------------------------------------------
void rmt_tx(void)
{
    // Set tx_start for channel 0
    *RMT_CH0CONF1_REG |= 1;

    // Wait for TX end interrupt (bit0 of RMT_INT_ST_REG)
    while (!(*RMT_INT_ST_REG & 1)) { }
    // Clear the interrupt
    *RMT_INT_CLR_REG |= 1;

    // Optional: set idle bit or do any re-initialization
    *RMT_CH0CONF1_REG |= (1 << 3);
}

// ---------------------------------------------------------------------------
// Configure GPIO4 for RMT output
// ---------------------------------------------------------------------------
void setup_gpio(void)
{
    // Enable RMT peripheral clock
    *DPORT_PERIP_CLK_EN_REG |= (1 << 9);

    // Enable GPIO pin 4 for output
    *GPIO_ENABLE_REG |= (1 << 4);

    // Route RMT signal to GPIO4
    *GPIO_FUNC4_OUT_SEL_CFG_REG |= (1 << 10) | RMT_SIG_OUT0_IDX;

    // Clear function bits for GPIO4 mux
    *IO_MUX_GPIO4_REG &= ~(0x7 << 12);
}

// ---------------------------------------------------------------------------
// Configure RMT peripheral
// ---------------------------------------------------------------------------
void setup_rmt(void)
{
    // Reset some fields (simple clearing)
    *RMT_CH0CONF0_REG &= ~0xFF;
    // clk_div = 8 => 80MHz / 8 = 10MHz
    *RMT_CH0CONF0_REG |= 0x8;

    // set 1 block of memory
    *RMT_CH0CONF0_REG |= (0x1 << 24);

    // Disable carrier
    *RMT_CH0CONF0_REG &= ~(0x1 << 28);

    // Enable idle output
    *RMT_CH0CONF1_REG |= (0x1 << 17);

    // Memory owner => transmitter
    *RMT_CH0CONF1_REG &= ~(1 << 5);

    // Enable RMT interrupts
    *RMT_INT_ENA_REG |= 7;

    // Enable RMT (APB) clock
    *RMT_APB_CONF_REG |= 1;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
void app_main(void)
{
    setup_gpio();
    setup_rmt();
    setup_gpio_pcnt(); // setup gpio pins for pcnt input
    pcnt_init(); // setup the pcnt registers
    pcnt_isr_attach();

    // These are values that I got from Espressifs example after printing out the registers. These didn't change the fact I still can't increment the counter
    *PCNT_U0_CONF0_REG = 0x4649fc50;
    *PCNT_U0_CONF1_REG = 0xffce0032;
    *PCNT_U0_CONF2_REG = 0xff9c0064;
    
    ESP_LOGI("ENC", "PCNT_U0_CONF0_REG: 0x%x", (unsigned int)*PCNT_U0_CONF0_REG);
    ESP_LOGI("ENC", "PCNT_U0_CONF1_REG: 0x%x", (unsigned int)*PCNT_U0_CONF1_REG);
    ESP_LOGI("ENC", "PCNT_U0_CONF2_REG: 0x%x", (unsigned int)*PCNT_U0_CONF2_REG);
    ESP_LOGI("ENC", "PCNT_U0_STATUS_REG: 0x%x", (unsigned int)*PCNT_U0_STATUS_REG);
    ESP_LOGI("ENC", "PCNT_INT_ENA_REG: 0x%x", (unsigned int)*PCNT_INT_ENA_REG);
    ESP_LOGI("ENC", "PCNT_CTRL_REG: 0x%x", (unsigned int)*PCNT_CTRL_REG);
    ESP_LOGI("ENC", "GPIO_FUNC39_IN_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC39_IN_SEL_CFG);
    ESP_LOGI("ENC", "GPIO_FUNC41_IN_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC41_IN_SEL_CFG);
    ESP_LOGI("ENC", "GPIO_FUNC2_OUT_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC2_OUT_SEL_CFG);
    ESP_LOGI("ENC", "GPIO_FUNC16_OUT_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC16_OUT_SEL_CFG);
    ESP_LOGI("ENC", "GPIO_ENABLE_REG: 0x%x", (unsigned int)*GPIO_ENABLE_REG);
    ESP_LOGI("ENC", "DPORT_PRO_PCNT_INTR_MAP: 0x%x", (unsigned int)*DPORT_PRO_PCNT_INTR_MAP);
    ESP_LOGI("ENC", "DPORT_PERIP_CLK_EN_REG: 0x%x", (unsigned int)*DPORT_PERIP_CLK_EN_REG);
    ESP_LOGI("ENC", "IO_MUX_GPIO2: 0x%x", (unsigned int)*IO_MUX_GPIO2);
    ESP_LOGI("ENC", "IO_MUX_GPIO16: 0x%x", (unsigned int)*IO_MUX_GPIO16);



    // Right now I'm just getting stuck here and my cnt does not increment no matter what. 
    int16_t cnt = 0;
    while(!isr_hit){
        cnt = (int16_t)(*PCNT_U0_CNT_REG);
        ESP_LOGI("ENC", "%hx", cnt);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    *RMT_CH0CONF1_REG &= ~(1 << 6);

    // Transmit red
    rmt_fill_ws2812_data(0x0, 0x80, 0x0);
    rmt_tx();

    // Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Transmit green
    rmt_fill_ws2812_data(0x80, 0x0, 0x0);
    rmt_tx();

    // Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Transmit Blue
    rmt_fill_ws2812_data(0x0, 0x0, 0x80);
    rmt_tx();

    while (1) {
        // Do nothing
        asm volatile ("waiti 0");
    }
}

// Example code from espressif. Added some log statements + disable the pull up/down resistors as the ky040 encoder already has them.

// /*
//  * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
//  *
//  * SPDX-License-Identifier: CC0-1.0
//  */

//  #include "sdkconfig.h"
//  #include "freertos/FreeRTOS.h"
//  #include "freertos/task.h"
//  #include "freertos/queue.h"
//  #include "esp_log.h"
//  #include "driver/pulse_cnt.h"
//  #include "driver/gpio.h"
//  #include "esp_sleep.h"
 
//  static const char *TAG = "example";
 
//  #define EXAMPLE_PCNT_HIGH_LIMIT 100
//  #define EXAMPLE_PCNT_LOW_LIMIT  -100
 
//  #define EXAMPLE_EC11_GPIO_A 2
//  #define EXAMPLE_EC11_GPIO_B 16
 
//  static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
//  {
//      BaseType_t high_task_wakeup;
//      QueueHandle_t queue = (QueueHandle_t)user_ctx;
//      // send event data to queue, from this interrupt callback
//      xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
//      return (high_task_wakeup == pdTRUE);
//  }
 
//  void app_main(void)
//  {
//      ESP_LOGI(TAG, "install pcnt unit");
//      pcnt_unit_config_t unit_config = {
//          .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
//          .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
//      };
//      pcnt_unit_handle_t pcnt_unit = NULL;
//      ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
 
//      ESP_LOGI(TAG, "set glitch filter");
//      pcnt_glitch_filter_config_t filter_config = {
//          .max_glitch_ns = 1000,
//      };
//      ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
 
//      ESP_LOGI(TAG, "install pcnt channels");
//      pcnt_chan_config_t chan_a_config = {
//          .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
//          .level_gpio_num = EXAMPLE_EC11_GPIO_B,
//      };
//      pcnt_channel_handle_t pcnt_chan_a = NULL;
//      ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
//      pcnt_chan_config_t chan_b_config = {
//          .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
//          .level_gpio_num = EXAMPLE_EC11_GPIO_A,
//      };
//      pcnt_channel_handle_t pcnt_chan_b = NULL;
//      ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
 
//      ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
//      ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//      ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
//      ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
//      ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
 
//      ESP_LOGI(TAG, "add watch points and register callbacks");
//      int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
//      for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
//          ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
//      }
//      pcnt_event_callbacks_t cbs = {
//          .on_reach = example_pcnt_on_reach,
//      };
//      QueueHandle_t queue = xQueueCreate(10, sizeof(int));
//      ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));
 
//      ESP_LOGI(TAG, "enable pcnt unit");
//      ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
//      ESP_LOGI(TAG, "clear pcnt unit");
//      ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
//      ESP_LOGI(TAG, "start pcnt unit");
//      ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
 
//  #if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
//      // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
//      ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
//      ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
//      ESP_ERROR_CHECK(esp_light_sleep_start());
//  #endif
 
//      // Report counter value
//      int pulse_count = 0;
//      int event_count = 0;

//     ESP_LOGI("ENC", "PCNT_U0_CONF0_REG: 0x%x", (unsigned int)*PCNT_U0_CONF0_REG);
//     ESP_LOGI("ENC", "PCNT_U0_CONF1_REG: 0x%x", (unsigned int)*PCNT_U0_CONF1_REG);
//     ESP_LOGI("ENC", "PCNT_U0_CONF2_REG: 0x%x", (unsigned int)*PCNT_U0_CONF2_REG);
//     ESP_LOGI("ENC", "PCNT_U0_STATUS_REG: 0x%x", (unsigned int)*PCNT_U0_STATUS_REG);
//     ESP_LOGI("ENC", "PCNT_INT_ENA_REG: 0x%x", (unsigned int)*PCNT_INT_ENA_REG);
//     ESP_LOGI("ENC", "PCNT_CTRL_REG: 0x%x", (unsigned int)*PCNT_CTRL_REG);
//     ESP_LOGI("ENC", "GPIO_FUNC39_IN_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC39_IN_SEL_CFG);
//     ESP_LOGI("ENC", "GPIO_FUNC41_IN_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC41_IN_SEL_CFG);
//     ESP_LOGI("ENC", "GPIO_FUNC2_OUT_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC2_OUT_SEL_CFG);
//     ESP_LOGI("ENC", "GPIO_FUNC16_OUT_SEL_CFG: 0x%x", (unsigned int)*GPIO_FUNC16_OUT_SEL_CFG);
//     ESP_LOGI("ENC", "GPIO_ENABLE_REG: 0x%x", (unsigned int)*GPIO_ENABLE_REG);
//     ESP_LOGI("ENC", "DPORT_PRO_PCNT_INTR_MAP: 0x%x", (unsigned int)*DPORT_PRO_PCNT_INTR_MAP);
//     ESP_LOGI("ENC", "DPORT_PERIP_CLK_EN_REG: 0x%x", (unsigned int)*DPORT_PERIP_CLK_EN_REG);
//     ESP_LOGI("ENC", "IO_MUX_GPIO2: 0x%x", (unsigned int)*IO_MUX_GPIO2);
//     ESP_LOGI("ENC", "IO_MUX_GPIO16: 0x%x", (unsigned int)*IO_MUX_GPIO16);

//     *PCNT_U0_CONF0_REG = 0x4649fc50;
//     *PCNT_U0_CONF1_REG = 0xffce0032;
//     *PCNT_U0_CONF2_REG = 0xff9c0064;

//      while (1) {
//          if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
//              ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
//          } else {
//              ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
//              ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
//          }
//      }
//  }