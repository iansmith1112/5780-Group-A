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

#include "driver/gpio.h"

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
    
    
    // ESP_LOGI("ENC", "PCNT_U0_CONF0_REG: 0x%x", (unsigned int)REG_VAL(PCNT_U0_CONF0_REG));
    // ESP_LOGI("ENC", "PCNT_U0_CONF1_REG: 0x%x", (unsigned int)REG_VAL(PCNT_U0_CONF1_REG));
    // ESP_LOGI("ENC", "PCNT_U0_CONF2_REG: 0x%x", (unsigned int)REG_VAL(PCNT_U0_CONF2_REG));
    // ESP_LOGI("ENC", "PCNT_U0_STATUS_REG: 0x%x", (unsigned int)REG_VAL(PCNT_U0_STATUS_REG));
    // ESP_LOGI("ENC", "PCNT_INT_ENA_REG: 0x%x", (unsigned int)REG_VAL(PCNT_INT_ENA_REG));
    // ESP_LOGI("ENC", "PCNT_CTRL_REG: 0x%x", (unsigned int)REG_VAL(PCNT_CTRL_REG));
    // ESP_LOGI("ENC", "GPIO_FUNC39_IN_SEL_CFG: 0x%x", (unsigned int)(GPIO_FUNC39_IN_SEL_CFG_REG)); 
    // ESP_LOGI("ENC", "GPIO_FUNC41_IN_SEL_CFG: 0x%x", (unsigned int)REG_VAL(GPIO_FUNC41_IN_SEL_CFG_REG));
    // ESP_LOGI("ENC", "GPIO_FUNC2_OUT_SEL_CFG: 0x%x", (unsigned int)REG_VAL(GPIO_FUNC2_OUT_SEL_CFG_REG));
    // ESP_LOGI("ENC", "GPIO_FUNC16_OUT_SEL_CFG: 0x%x", (unsigned int)REG_VAL(GPIO_FUNC16_OUT_SEL_CFG_REG));
    // ESP_LOGI("ENC", "GPIO_ENABLE_REG: 0x%x", (unsigned int)REG_VAL(GPIO_ENABLE_REG));
    // ESP_LOGI("ENC", "DPORT_PRO_PCNT_INTR_MAP: 0x%x", (unsigned int)REG_VAL(DPORT_PRO_PCNT_INTR_MAP_REG));
    // ESP_LOGI("ENC", "DPORT_PERIP_CLK_EN_REG: 0x%x", (unsigned int)REG_VAL(DPORT_PERIP_CLK_EN_REG));
    // ESP_LOGI("ENC", "IO_MUX_GPIO2: 0x%x", (unsigned int)REG_VAL(IO_MUX_GPIO2_REG));
    // ESP_LOGI("ENC", "IO_MUX_GPIO16: 0x%x", (unsigned int)REG_VAL(IO_MUX_GPIO16_REG));

    // Clear RMT channel 0 memory
    uint32_t* rmt_data = (uint32_t*)RMT_DATA;
    for(int i = 0; i < 48; i++) {
        rmt_data[i] = 0;
    }


    // // Right now I'm just getting stuck here and my cnt does not increment no matter what. 
    // int16_t cnt = 0;
    // while(1){
    //     while(!isr_hit){
    //         cnt = (int16_t)(REG_VAL(PCNT_U0_CNT_REG));
    //         ESP_LOGI("ENC", "cnt: %hx", cnt);
    //         ESP_LOGI("ENC", "g_state: %x", g_state);
    //         ESP_LOGI("ENC", "status: %x", (unsigned int)(REG_VAL(PCNT_INT_ST_REG) & PCNT_CNT_THR_EVENT_U0_INT_ST));
    //         ESP_LOGI("ENC", "ctrl_reg: %x", (unsigned int)REG_VAL(PCNT_CTRL_REG));
    //         vTaskDelay(pdMS_TO_TICKS(1000));
    //     }
    //     ESP_LOGI("ENC", "ISR ran");
    //     isr_hit = 0;
    // }
    
    
    // disable conti mode
    REG_VAL(RMT_CH0CONF1_REG) &= ~RMT_TX_CONTI_MODE_CH0;

    // // Transmit red
    // rmt_fill_ws2812_data(0x0, 0x80, 0x0);
    // rmt_tx();

    // // Delay 1 second
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // // Transmit green
    // rmt_fill_ws2812_data(0x80, 0x0, 0x0);
    // rmt_tx();

    // // Delay 1 second
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // // Transmit Blue
    // rmt_fill_ws2812_data(0x0, 0x0, 0x80);
    // rmt_tx();
    int prev_state = -3; // set to -3 as we need it to not be equal to g_state
    while (1) {
        if(g_state != prev_state)
        {
            switch(g_state)
            {
                case 0: 
                    ESP_LOGI("ENC", "green");
                    rmt_fill_ws2812_data(0x80, 0x0, 0x0);
                    rmt_tx(); 
                    prev_state=g_state;
                    break;
                case 1: 
                    ESP_LOGI("ENC", "red");
                    rmt_fill_ws2812_data(0x0, 0x80, 0x0);
                    rmt_tx();
                    prev_state=g_state;
                break;
                case 2:
                    ESP_LOGI("ENC", "blue");
                    rmt_fill_ws2812_data(0x0, 0x0, 0x80);
                    rmt_tx(); 
                    prev_state=g_state;
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

// #include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// // ---------------------------------------------------------------------------
// // DUMMY REGISTER & MACRO DEFINITIONS (addresses/values not necessarily accurate)
// // ---------------------------------------------------------------------------
// #define RMT_CH0CONF0_REG           ((volatile uint32_t *)0x3ff56020)
// #define RMT_CH0CONF1_REG           ((volatile uint32_t *)0x3ff56024)
// #define RMT_CH0CARRIER_DUTY_REG    ((volatile uint32_t *)0x3FF560B0)
// #define RMT_INT_ST_REG             ((volatile uint32_t *)0x3FF560A4)
// #define RMT_INT_CLR_REG            ((volatile uint32_t *)0x3FF560AC)
// #define RMT_INT_ENA_REG            ((volatile uint32_t *)0x3FF560A8)
// #define RMT_APB_CONF_REG           ((volatile uint32_t *)0x3FF560F0)

// #define GPIO_ENABLE_REG            ((volatile uint32_t *)0x3FF44020)
// #define GPIO_FUNC4_OUT_SEL_CFG_REG ((volatile uint32_t *)0x3FF44540)
// #define IO_MUX_GPIO4_REG           ((volatile uint32_t *)0x3FF49048)
// #define RMT_SIG_OUT0_IDX           87

// #define DPORT_PERIP_CLK_EN_REG     ((volatile uint32_t *)0x3FF000C0)
// #define RMT_DATA                   ((volatile uint32_t *)0x3FF56800)

// // ---------------------------------------------------------------------------
// // Timing definitions for WS2812 LED (in units of RMT ticks).
// // ---------------------------------------------------------------------------
// #define T0H    3   // 0.3us (assuming 10MHz RMT clock = 100ns per tick)
// #define T0L    8   // 0.8us
// #define T1H    8   // 0.8us
// #define T1L    4   // 0.4us
// #define T_RESET 252 // ~25us * 2 (2 periods setup in rmt_item_t reset)

// // ---------------------------------------------------------------------------
// // RMT RAM data structure
// // ---------------------------------------------------------------------------
// typedef struct {
//     uint32_t duration0 : 15; 
//     uint32_t level0    : 1;  
//     uint32_t duration1 : 15; 
//     uint32_t level1    : 1;  
// } rmt_item32_t;

// // RMT items for logical '0' and '1', plus a reset pulse
// static rmt_item32_t bit0  = { .duration0 = T0H, .level0=1, .duration1=T0L, .level1=0 };
// static rmt_item32_t bit1  = { .duration0 = T1H, .level0=1, .duration1=T1L, .level1=0 };
// static rmt_item32_t reset = { .duration0 = T_RESET, .level0=0, .duration1=T_RESET, .level1=0 };

// // ---------------------------------------------------------------------------
// // Fill RMT memory with WS2812 data for a single RGB LED (24 bits: GRB order).
// // ---------------------------------------------------------------------------
// void rmt_fill_ws2812_data(uint8_t g, uint8_t r, uint8_t b)
// {
//     volatile rmt_item32_t* mem = (volatile rmt_item32_t*) RMT_DATA;
//     uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;

//     // 24 bits total
//     for (int i = 23; i >= 0; i--) {
//         if (grb & (1 << i)) {
//             mem[23 - i] = bit1;  // '1'
//         } else {
//             mem[23 - i] = bit0;  // '0'
//         }
//     }
//     // Add one more item for the reset
//     mem[24] = reset;
// }

// // ---------------------------------------------------------------------------
// // Start RMT transmission
// // ---------------------------------------------------------------------------
// void rmt_tx(void)
// {
//     // Set tx_start for channel 0
//     *RMT_CH0CONF1_REG |= 1;

//     // Wait for TX end interrupt (bit0 of RMT_INT_ST_REG)
//     while (!(*RMT_INT_ST_REG & 1)) { }
//     // Clear the interrupt
//     *RMT_INT_CLR_REG |= 1;

//     // Optional: set idle bit or do any re-initialization
//     *RMT_CH0CONF1_REG |= (1 << 3);
// }

// // ---------------------------------------------------------------------------
// // Configure GPIO4 for RMT output
// // ---------------------------------------------------------------------------
// void setup_gpio(void)
// {
//     // Enable RMT peripheral clock
//     *DPORT_PERIP_CLK_EN_REG |= (1 << 9);

//     // Enable GPIO pin 4 for output
//     *GPIO_ENABLE_REG |= (1 << 4);

//     // Route RMT signal to GPIO4
//     *GPIO_FUNC4_OUT_SEL_CFG_REG |= (1 << 10) | RMT_SIG_OUT0_IDX;

//     // Clear function bits for GPIO4 mux
//     *IO_MUX_GPIO4_REG &= ~(0x7 << 12);
// }

// // ---------------------------------------------------------------------------
// // Configure RMT peripheral
// // ---------------------------------------------------------------------------
// void setup_rmt(void)
// {
//     // Reset some fields (simple clearing)
//     *RMT_CH0CONF0_REG &= ~0xFF;
//     // clk_div = 8 => 80MHz / 8 = 10MHz
//     *RMT_CH0CONF0_REG |= 0x8;

//     // Example: set 1 block of memory (bit24=1 -> that’s a placeholder, not strictly correct)
//     *RMT_CH0CONF0_REG |= (0x1 << 24);

//     // Disable carrier
//     *RMT_CH0CONF0_REG &= ~(0x1 << 28);

//     // Enable idle output, etc. (bit17=1 is just an example)
//     *RMT_CH0CONF1_REG |= (0x1 << 17);

//     // Memory owner => transmitter
//     *RMT_CH0CONF1_REG &= ~(1 << 5);

//     // Enable RMT interrupts
//     *RMT_INT_ENA_REG |= 7;

//     // Enable RMT (APB) clock
//     *RMT_APB_CONF_REG |= 1;
//     *RMT_INT_ENA_REG  |= 1;
// }

// // ---------------------------------------------------------------------------
// // Main
// // ---------------------------------------------------------------------------
// void app_main(void)
// {
//     setup_gpio();
//     setup_rmt();

//     ESP_LOGI("ENC", "RMT_CH0CONF0_REG: 0x%x", (unsigned int)*RMT_CH0CONF0_REG);
//     ESP_LOGI("ENC", "RMT_CH0CONF1_REG: 0x%x", (unsigned int)*RMT_CH0CONF1_REG);
//     ESP_LOGI("ENC", "RMT_INT_ENA_REG: 0x%x", (unsigned int)*RMT_INT_ENA_REG);
//     ESP_LOGI("ENC", "RMT_APB_CONF_REG: 0x%x", (unsigned int)*RMT_APB_CONF_REG);
//     ESP_LOGI("ENC", "IO_MUX_GPIO4: 0x%x", (unsigned int)*IO_MUX_GPIO4_REG);

//     // Clear RMT channel 0 memory
//     *RMT_CH0CONF1_REG &= ~(1 << 2);
//     uint32_t* rmt_data = (uint32_t*)RMT_DATA;
//     for(int i = 0; i < 48; i++) {
//         rmt_data[i] = 0;
//     }
//     *RMT_CH0CONF1_REG &= ~(1 << 6);

//     // Transmit red
//     rmt_fill_ws2812_data(0x0, 0x80, 0x0);
//     rmt_tx();

//     // Delay 1 second
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     // Transmit green
//     rmt_fill_ws2812_data(0x80, 0x0, 0x0);
//     rmt_tx();

//     // Delay 1 second
//     vTaskDelay(pdMS_TO_TICKS(1000));

//     // Transmit Blue
//     rmt_fill_ws2812_data(0x0, 0x0, 0x80);
//     rmt_tx();

//     while (1) {
//         // Do nothing
//     }
// }