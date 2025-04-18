#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/pulse_cnt.h"

// ---------------------------------------------------------------------------
// DUMMY REGISTER & MACRO DEFINITIONS (addresses/values not necessarily accurate)
// ---------------------------------------------------------------------------


#define RMT_CH0CONF0_REG           ((volatile uint32_t *)0x3ff56020)
#define RMT_CH0CONF1_REG           ((volatile uint32_t *)0x3ff56024)
#define RMT_CH0CARRIER_DUTY_REG    ((volatile uint32_t *)0x3FF560B0)
#define RMT_INT_ST_REG             ((volatile uint32_t *)0x3FF560A4)
#define RMT_INT_CLR_REG            ((volatile uint32_t *)0x3FF560AC)
#define RMT_INT_ENA_REG            ((volatile uint32_t *)0x3FF560A8)
#define RMT_APB_CONF_REG           ((volatile uint32_t *)0x3FF560F0)

#define GPIO_ENABLE_REG            ((volatile uint32_t *)0x3FF44020)
#define GPIO_FUNC4_OUT_SEL_CFG_REG ((volatile uint32_t *)0x3FF44540)
#define IO_MUX_GPIO4_REG           ((volatile uint32_t *)0x3FF49048)
#define RMT_SIG_OUT0_IDX           87

#define DPORT_PERIP_CLK_EN_REG     ((volatile uint32_t *)0x3FF000C0)
#define RMT_DATA                   ((volatile uint32_t *)0x3FF56800)

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
    }
}