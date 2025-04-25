#include <stdint.h>
#include <stdlib.h>
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
#include "driver/gpio.h"

#define REG_VAL(addr)  (*(volatile uint32_t *)(uintptr_t)(addr))
#define RMT_PIN        4
#define RMT_DATA       0x3FF56800
#define LED_COUNT      3

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
    // Reset the write position
    REG_VAL(RMT_CH0CONF1_REG) &= ~RMT_MEM_WR_RST_CH0;

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

    // set idle bit or do any re-initialization
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

    // set 8 blocks of memory
    REG_VAL(RMT_CH0CONF0_REG) |= (0x8 << RMT_MEM_SIZE_CH0_S);

    // Disable carrier
    REG_VAL(RMT_CH0CONF0_REG) &= ~RMT_CARRIER_EN_CH0;

    // Enable idle output
    REG_VAL(RMT_CH0CONF1_REG) |= RMT_REF_ALWAYS_ON_CH0;

    // Memory owner => transmitter
    REG_VAL(RMT_CH0CONF1_REG) &= ~RMT_MEM_OWNER_CH0;

    // Disable conti mode
    REG_VAL(RMT_CH0CONF1_REG) &= ~RMT_TX_CONTI_MODE_CH0;

    // Enable RMT interrupts
    REG_VAL(RMT_INT_ENA_REG) |= (RMT_CH0_ERR_INT_ENA | RMT_CH0_RX_END_INT_ENA | RMT_CH0_TX_END_INT_ENA);

    // Enable RMT (APB) clock
    REG_VAL(RMT_APB_CONF_REG) |= 1; // RMT_MEM_ACCESS_EN
    REG_VAL(RMT_INT_ENA_REG) |= RMT_CH0_TX_END_INT_ENA;
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

    ESP_LOGI("ENC", "RMT_CH0CONF0_REG: 0x%x", (unsigned int)REG_VAL(RMT_CH0CONF0_REG));
    ESP_LOGI("ENC", "RMT_CH0CONF1_REG: 0x%x", (unsigned int)REG_VAL(RMT_CH0CONF1_REG));
    ESP_LOGI("ENC", "RMT_INT_ENA_REG: 0x%x", (unsigned int)REG_VAL(RMT_INT_ENA_REG));
    ESP_LOGI("ENC", "RMT_APB_CONF_REG: 0x%x", (unsigned int)REG_VAL(RMT_APB_CONF_REG));
    ESP_LOGI("ENC", "IO_MUX_GPIO4: 0x%x", (unsigned int)REG_VAL(IO_MUX_GPIO4_REG));

    // Clear RMT channel 0 memory
    uint32_t* rmt_data = (uint32_t*)RMT_DATA;
    for(int i = 0; i < 48; i++) {
        rmt_data[i] = 0;
    }

    int prev_state = -3; // set to -3 as we need it to not be equal to g_state initially
    while (1) {
        if(g_state != prev_state)
        {
            prev_state=g_state;
            switch(g_state)
            {
                case 0: 
                    ESP_LOGI("ENC", "green");
                    rmt_fill_ws2812_data(0x80, 0x0, 0x0);
                    rmt_tx(); 
                    break;
                case 1: 
                    ESP_LOGI("ENC", "red");
                    rmt_fill_ws2812_data(0x0, 0x80, 0x0);
                    rmt_tx();
                break;
                case 2:
                    ESP_LOGI("ENC", "blue");
                    rmt_fill_ws2812_data(0x0, 0x0, 0x80);
                    rmt_tx(); 
                    break;
                default:
                    ESP_LOGI("ENC", "unknown");
                    rmt_fill_ws2812_data(0x99, 0x99, 0xff);
                    rmt_tx(); 
                    break;
            }
        }
    }
}