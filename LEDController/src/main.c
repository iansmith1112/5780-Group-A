#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"
#include "soc/pcnt_reg.h"
#include "soc/rmt_reg.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"

#define REG_VAL(addr)  (*(volatile uint32_t *)(uintptr_t)(addr))
#define RMT_PIN        4
#define RMT_DATA       0x3FF56800

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
    REG_VAL(RMT_CH0CONF1_REG) |= RMT_TX_START_CH0;

    // Wait for TX end interrupt (bit0 of RMT_INT_ST_REG)
    while (!(REG_VAL(RMT_INT_ST_REG) & RMT_CH0_TX_END_INT_ST)) { }
    // Clear the interrupt
    REG_VAL(RMT_INT_CLR_REG) |= RMT_CH0_TX_END_INT_CLR;

    // Optional: set idle bit or do any re-initialization
    REG_VAL(RMT_CH0CONF1_REG) |= RMT_MEM_RD_RST_CH0;
}

// ---------------------------------------------------------------------------
// Configure GPIO4 for RMT output
// ---------------------------------------------------------------------------
void setup_gpio(void)
{
    // Enable RMT peripheral clock
    REG_VAL(DPORT_PERIP_CLK_EN_REG) |= DPORT_RMT_CLK_EN;

    // Enable GPIO pin 4 for output
    REG_VAL(GPIO_ENABLE_REG) |= (1 << RMT_PIN);

    // Route RMT signal to GPIO4
    REG_VAL(GPIO_FUNC4_OUT_SEL_CFG_REG) |= GPIO_FUNC4_OEN_SEL | RMT_SIG_OUT0_IDX;

    // Clear function bits for GPIO4 mux
    REG_VAL(IO_MUX_GPIO4_REG) &= ~MCU_SEL_M;
}

// ---------------------------------------------------------------------------
// Configure RMT peripheral
// ---------------------------------------------------------------------------
void setup_rmt(void)
{
    // Reset some fields (simple clearing)
    REG_VAL(RMT_CH0CONF0_REG) &= ~RMT_DIV_CNT_CH0;
    // clk_div = 8 => 80MHz / 8 = 10MHz
    REG_VAL(RMT_CH0CONF0_REG) |= 0x8;

    // set 1 block of memory
    REG_VAL(RMT_CH0CONF0_REG) |= (0x1 << RMT_MEM_SIZE_CH0_S);

    // Disable carrier
    REG_VAL(RMT_CH0CONF0_REG) &= ~RMT_CARRIER_EN_CH0;

    // Enable idle output
    REG_VAL(RMT_CH0CONF1_REG) |= RMT_REF_ALWAYS_ON_CH0;

    // Memory owner => transmitter
    REG_VAL(RMT_CH0CONF1_REG) &= ~RMT_MEM_OWNER_CH0;

    // Enable RMT interrupts
    REG_VAL(RMT_INT_ENA_REG) |= (RMT_CH0_ERR_INT_ENA | RMT_CH0_RX_END_INT_ENA | RMT_CH0_TX_END_INT_ENA);

    // Enable RMT (APB) clock
    REG_VAL(RMT_APB_CONF_REG) |= 1; // RMT_MEM_ACCESS_EN
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
    REG_VAL(PCNT_U0_CONF0_REG) = 0x4649fc50;
    REG_VAL(PCNT_U0_CONF1_REG) = 0xffce0032;
    REG_VAL(PCNT_U0_CONF2_REG) = 0xff9c0064;
    
    
    ESP_LOGI("ENC", "PCNT_U0_CONF0_REG: 0x%x", REG_VAL(PCNT_U0_CONF0_REG));
    ESP_LOGI("ENC", "PCNT_U0_CONF1_REG: 0x%x", REG_VAL(PCNT_U0_CONF1_REG));
    ESP_LOGI("ENC", "PCNT_U0_CONF2_REG: 0x%x", REG_VAL(PCNT_U0_CONF2_REG));
    ESP_LOGI("ENC", "PCNT_U0_STATUS_REG: 0x%x", REG_VAL(PCNT_U0_STATUS_REG));
    ESP_LOGI("ENC", "PCNT_INT_ENA_REG: 0x%x", REG_VAL(PCNT_INT_ENA_REG));
    ESP_LOGI("ENC", "PCNT_CTRL_REG: 0x%x", REG_VAL(PCNT_CTRL_REG));
    ESP_LOGI("ENC", "GPIO_FUNC39_IN_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC39_IN_SEL_CFG_REG)); 
    ESP_LOGI("ENC", "GPIO_FUNC41_IN_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC41_IN_SEL_CFG_REG));
    ESP_LOGI("ENC", "GPIO_FUNC2_OUT_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC2_OUT_SEL_CFG_REG));
    ESP_LOGI("ENC", "GPIO_FUNC16_OUT_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC16_OUT_SEL_CFG_REG));
    ESP_LOGI("ENC", "GPIO_ENABLE_REG: 0x%x", REG_VAL(GPIO_ENABLE_REG));
    ESP_LOGI("ENC", "DPORT_PRO_PCNT_INTR_MAP: 0x%x", REG_VAL(DPORT_PRO_PCNT_INTR_MAP));
    ESP_LOGI("ENC", "DPORT_PERIP_CLK_EN_REG: 0x%x", REG_VAL(DPORT_PERIP_CLK_EN_REG));
    ESP_LOGI("ENC", "IO_MUX_GPIO2: 0x%x", REG_VAL(IO_MUX_GPIO2_REG));
    ESP_LOGI("ENC", "IO_MUX_GPIO16: 0x%x", REG_VAL(IO_MUX_GPIO16_REG));



    // Right now I'm just getting stuck here and my cnt does not increment no matter what. 
    int16_t cnt = 0;
    while(!isr_hit){
        cnt = (int16_t)(REG_VAL(PCNT_U0_CNT_REG));
        ESP_LOGI("ENC", "%hx", cnt);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // disable conti mode
    REG_VAL(RMT_CH0CONF1_REG) &= ~RMT_TX_CONTI_MODE_CH0;

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

//     ESP_LOGI("ENC", "PCNT_U0_CONF0_REG: 0x%x", REG_VAL(PCNT_U0_CONF0_REG));
//     ESP_LOGI("ENC", "PCNT_U0_CONF1_REG: 0x%x", REG_VAL(PCNT_U0_CONF1_REG));
//     ESP_LOGI("ENC", "PCNT_U0_CONF2_REG: 0x%x", REG_VAL(PCNT_U0_CONF2_REG));
//     ESP_LOGI("ENC", "PCNT_U0_STATUS_REG: 0x%x", REG_VAL(PCNT_U0_STATUS_REG));
//     ESP_LOGI("ENC", "PCNT_INT_ENA_REG: 0x%x", REG_VAL(PCNT_INT_ENA_REG));
//     ESP_LOGI("ENC", "PCNT_CTRL_REG: 0x%x", REG_VAL(PCNT_CTRL_REG));
//     ESP_LOGI("ENC", "GPIO_FUNC39_IN_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC39_IN_SEL_CFG_REG)); 
//     ESP_LOGI("ENC", "GPIO_FUNC41_IN_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC41_IN_SEL_CFG_REG));
//     ESP_LOGI("ENC", "GPIO_FUNC2_OUT_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC2_OUT_SEL_CFG_REG));
//     ESP_LOGI("ENC", "GPIO_FUNC16_OUT_SEL_CFG: 0x%x", REG_VAL(GPIO_FUNC16_OUT_SEL_CFG_REG));
//     ESP_LOGI("ENC", "GPIO_ENABLE_REG: 0x%x", REG_VAL(GPIO_ENABLE_REG));
//     ESP_LOGI("ENC", "DPORT_PRO_PCNT_INTR_MAP: 0x%x", REG_VAL(DPORT_PRO_PCNT_INTR_MAP));
//     ESP_LOGI("ENC", "DPORT_PERIP_CLK_EN_REG: 0x%x", REG_VAL(DPORT_PERIP_CLK_EN_REG));
//     ESP_LOGI("ENC", "IO_MUX_GPIO2: 0x%x", REG_VAL(IO_MUX_GPIO2_REG));
//     ESP_LOGI("ENC", "IO_MUX_GPIO16: 0x%x", REG_VAL(IO_MUX_GPIO16_REG));

//     REG_VAL(IO_MUX_GPIO2_REG) &= ~(FUN_PU | FUN_PD | MCU_SEL);  // disable pull down and up 
//     REG_VAL(IO_MUX_GPIO16_REG) &= ~(FUN_PU | FUN_PD | MCU_SEL); // disable pull down and up

//      while (1) {
//          if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
//              ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
//          } else {
//              ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
//              ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
//          }
//      }
//  }