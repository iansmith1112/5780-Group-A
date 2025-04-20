#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_log.h"

/* ─────────────── Register addresses ─────────────── */
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

#define STEPS_PER_DETENT  2     // 2 edges → 1 click

volatile int g_state = 0;
volatile int isr_hit = 0;

// interrupt handler
void IRAM_ATTR pcnt_isr(void)
{
    isr_hit = 1;
    /* Determine which unit fired; we only enabled U0 so read bit 0. */
    if (*PCNT_INT_ST_REG & 0x1) {
        int16_t cnt = (int16_t)(*PCNT_U0_CNT_REG);

        /* Update application state. Positive count → clockwise. */
        if (cnt >= STEPS_PER_DETENT) {
            g_state = (g_state + 1) % 3;
        } else if (cnt <= -STEPS_PER_DETENT) {
            g_state = (g_state + 2) % 3; /* equivalent to −1 mod 3 */
            
        }

        /* Clear & reset */
        *PCNT_INT_CLR_REG = 0x1;           /* acknowledge */
        // *PCNT_CTRL_REG |= (1 << 0);      /* PLUS_CNT_RST_U0 */
    }
}

//
void setup_gpio_pcnt(void)
{
    // signal 39 ctrl 41
    // pin d2 / gpio 2 = encoder a (clk)
    // pin d16 / gpio 16 = encoder b (dt) / RX2
    // pin d17 / gpio 17 = button // will do later

    // Enable PCNT peripheral clock
    *DPORT_PERIP_CLK_EN_REG |= (1 << 10);
    *DPORT_PERIP_RST_EN_REG &= ~(1 << 10);

    /// Configure the GPIO_FUNCy_IN_SEL_CFG register corresponding to peripheral signal Y in the GPIO Matrix:
        // Set GPIO_SIGy_IN_SEL to enable peripheral signal input via GPIO matrix.
        // Set the GPIO_FUNCy_IN_SEL field in this register, corresponding to the GPIO pin X to read from.
    *GPIO_FUNC39_IN_SEL_CFG &= ~(0x3f);
    *GPIO_FUNC41_IN_SEL_CFG &= ~(0x3f);
    *GPIO_FUNC39_IN_SEL_CFG |= (1<<7) | 2;
    *GPIO_FUNC41_IN_SEL_CFG |= (1<<7) | 16;

    /// Configure the GPIO_FUNCx_OUT_SEL_CFG register and clear the GPIO_ENABLE_DATA[x] field corresponding to GPIO pin X in the GPIO Matrix:
        // Set the GPIO_FUNCx_OEN_SEL bit in the GPIO_FUNCx_OUT_SEL_CFG register to force the pin’s output state to be determined always by the GPIO_ENABLE_DATA[x] field.
    *GPIO_FUNC2_OUT_SEL_CFG |= (1 << 10);
    *GPIO_FUNC16_OUT_SEL_CFG |= (1 << 10);
        // The GPIO_ENABLE_DATA[x] field is a bit in either GPIO_ENABLE_REG(GPIOs0-31) or GPIO_ENABLE1_REG (GPIOs 32-39). Clear this bit to disable the output driver for the GPIO pin.
    *GPIO_ENABLE_REG &= ~(1 << 2);
    *GPIO_ENABLE_REG &= ~(1 << 16);

    /// Configure the IO MUX to select the GPIO Matrix. Set the IO_MUX_x_REG register corresponding to GPIO pin X as follows:
        // Set the function field (MCU_SEL) to the IO MUX function corresponding to GPIO X (this is Function 2—numeric value 2—for all pins).
        // Enable the input by setting the FUN_IE bit.
        // Set or clear the FUN_WPU and FUN_WPD bits, as desired, to enable/disable internal pull-up/pull down resistors.
    *IO_MUX_GPIO2 &= ~(1<<8 | 1<<7 | 3<<12); // disable pull down and up
    *IO_MUX_GPIO16 &= ~(1<<8 | 1<<7 | 3<<12); // disable pull down and up
    *IO_MUX_GPIO2 |= ((1<<9) | (2<<12)); // enable fun_ie and AF to 2
    *IO_MUX_GPIO16 |= ((1<<9) | (2<<12));
}

void pcnt_init(void)
{
    /* Pause and reset counter */
    *PCNT_CTRL_REG |= (1<<1)|(1<<0);

    uint32_t conf0 = 0;
    // conf0 |= (1 << 16);              // increment on falling edge
    // conf0 |= (2 << 18);              // decrement on rising edge      
    // conf0 |= (1 << 22);              // invert if B low → dec   
    // conf0 |= (1 << 10);              // simple de‑glitch filter  
    // conf0 |= (1 << 14);              // PCNT_THR_THRES0_EN
    // conf0 |= 500;                    // Set threshold to 6.25us which is standard for encoders
    conf0 |= (1 << 16);              // increment on falling edge
    *PCNT_U0_CONF0_REG = conf0;

    // Set thresholds ±STEPS_PER_DETENT
    *PCNT_U0_CONF1_REG = (STEPS_PER_DETENT << 16) | (uint16_t)(-STEPS_PER_DETENT);

    // High/low limit values
    *PCNT_U0_CONF2_REG = (STEPS_PER_DETENT << 0) | ((uint16_t)(-STEPS_PER_DETENT) << 16);

    // Resume counting
    *PCNT_CTRL_REG &= ~((1<<1)|(1<<0)); // clear CNT_PAUSE_U0 

    // Enable interrupt for unit 0 threshold events
    *PCNT_INT_ENA_REG |= 0x1;
}

void pcnt_isr_attach(void)
{
    const int flags = ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM;

    // Using IDF framework to install interrupt handler because dealing with the vector table will take me way too long 
    // and I don't have the time to spend on that.
    esp_err_t err = esp_intr_alloc(48,
                                flags,
                                (intr_handler_t)pcnt_isr,
                                NULL,
                                NULL);
    // check for errors
    if (err != ESP_OK) {
        while (1) { }
    }
}