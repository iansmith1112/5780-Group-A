#include <stdint.h>
#include <stdbool.h>
#include "encoder.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_reg.h"
#include "soc/pcnt_reg.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"

#define REG_VAL(addr)  (*(volatile uint32_t *)(uintptr_t)(addr))
#define ENCAPIN 23               // pin used for encoder a
#define ENCBPIN 16              // pin used for encoder b
#define STEPS_PER_DETENT  2     // 2 edges → 1 click

volatile int g_state = 0;
volatile int isr_hit = 0;

// interrupt handler
void IRAM_ATTR pcnt_isr(void)
{
    isr_hit = 1;
    /* Determine which unit fired; we only enabled U0 so read bit 0. */
    if (REG_VAL(PCNT_INT_ST_REG) & PCNT_CNT_THR_EVENT_U0_INT_ST) {
        int16_t cnt = REG_VAL(PCNT_U0_CNT_REG);

        /* Update application state. Positive count → clockwise. */
        if (cnt >= STEPS_PER_DETENT) {
            g_state = (g_state + 1) % 3;
        } else if (cnt <= -STEPS_PER_DETENT) {
            g_state = (g_state + 2) % 3; /* equivalent to −1 mod 3 */
        }

        /* Clear & reset */
        REG_VAL(PCNT_INT_CLR_REG) |= PCNT_CNT_THR_EVENT_U0_INT_CLR;           /* acknowledge */

        /* reset counter */
        REG_VAL(PCNT_CTRL_REG) |= PCNT_PLUS_CNT_RST_U0;
    }
}

void pcnt_init(void)
{
    /* Pause and reset counter */
    REG_VAL(PCNT_CTRL_REG) |= PCNT_PLUS_CNT_RST_U0 | PCNT_CNT_PAUSE_U0;

    uint32_t conf0 = 0;
    // disabled most for debugging purposes. Just want it to increase the counter for now
    conf0 |= 1 << PCNT_CH0_NEG_MODE_U0_S;              // increment on falling edge
    conf0 |= 2 << PCNT_CH0_POS_MODE_U0_S;              // decrement on rising edge      
    conf0 |= 1 << PCNT_CH0_LCTRL_MODE_U0_S;            // invert if B low → dec   
    conf0 |= PCNT_FILTER_EN_U0;                        // simple de‑glitch filter  
    conf0 |= PCNT_THR_THRES0_EN_U0;                     // PCNT_THR_THRES0_EN
    conf0 |= PCNT_THR_THRES1_EN_U0;                     // PCNT_THR_THRES1_EN
    conf0 |= 500;                                      // Set threshold to 6.25us which is standard for encoders
    REG_VAL(PCNT_U0_CONF0_REG) = conf0;

    // Now count both edges unconditionally
    // uint32_t conf0 = (1 << PCNT_CH0_POS_MODE_U0_S)
    // | (1 << PCNT_CH0_NEG_MODE_U0_S);


    // REG_VAL(PCNT_U0_CONF2_REG) = ((uint32_t)(-1000) << 16) | (1000 & 0xFFFF);

    // Set thresholds ±STEPS_PER_DETENT
    REG_VAL(PCNT_U0_CONF1_REG) = (STEPS_PER_DETENT << 16) | (uint16_t)(-STEPS_PER_DETENT);

    // High/low limit values
    // REG_VAL(PCNT_U0_CONF2_REG) = ((uint16_t)(-STEPS_PER_DETENT) << 16) | (STEPS_PER_DETENT);

    // Resume counting
    REG_VAL(PCNT_CTRL_REG) &= ~(PCNT_PLUS_CNT_RST_U0 | PCNT_CNT_PAUSE_U0); // clear CNT_PAUSE_U0 

    // Enable interrupt for unit 0 threshold events
    REG_VAL(PCNT_INT_ENA_REG) |= PCNT_CNT_THR_EVENT_U0_INT_ENA;
}

//
void setup_gpio_pcnt(void)
{
    // Enable PCNT peripheral clock
    REG_VAL(DPORT_PERIP_CLK_EN_REG) |= DPORT_PCNT_CLK_EN;
    REG_VAL(DPORT_PERIP_RST_EN_REG) &= ~DPORT_PCNT_RST;

    /// Configure the GPIO_FUNCy_IN_SEL_CFG register corresponding to peripheral signal Y in the GPIO Matrix:
        // Set GPIO_SIGy_IN_SEL to enable peripheral signal input via GPIO matrix.
        // Set the GPIO_FUNCy_IN_SEL field in this register, corresponding to the GPIO pin X to read from.
    REG_VAL(GPIO_FUNC39_IN_SEL_CFG_REG) &= ~(GPIO_FUNC39_IN_SEL_M);
    REG_VAL(GPIO_FUNC41_IN_SEL_CFG_REG) &= ~(GPIO_FUNC41_IN_SEL_M);
    REG_VAL(GPIO_FUNC39_IN_SEL_CFG_REG) |= GPIO_SIG39_IN_SEL | ENCAPIN;
    REG_VAL(GPIO_FUNC41_IN_SEL_CFG_REG) |= GPIO_SIG41_IN_SEL | ENCBPIN;

    /// Configure the GPIO_FUNCx_OUT_SEL_CFG register and clear the GPIO_ENABLE_DATA[x] field corresponding to GPIO pin X in the GPIO Matrix:
        // Set the GPIO_FUNCx_OEN_SEL bit in the GPIO_FUNCx_OUT_SEL_CFG register to force the pin’s output state to be determined always by the GPIO_ENABLE_DATA[x] field.
    REG_VAL(GPIO_FUNC23_OUT_SEL_CFG_REG) |= GPIO_FUNC2_OEN_SEL;
    REG_VAL(GPIO_FUNC16_OUT_SEL_CFG_REG) |= GPIO_FUNC16_OEN_SEL;
        // The GPIO_ENABLE_DATA[x] field is a bit in either GPIO_ENABLE_REG(GPIOs0-31) or GPIO_ENABLE1_REG (GPIOs 32-39). Clear this bit to disable the output driver for the GPIO pin.
    REG_VAL(GPIO_ENABLE_REG) &= ~((1 << ENCAPIN) | (1 << ENCBPIN));

    REG_VAL(GPIO_ENABLE_W1TC_REG) = (1 << ENCAPIN); // try to disable it further?

    /// Configure the IO MUX to select the GPIO Matrix. Set the IO_MUX_x_REG register corresponding to GPIO pin X as follows:
        // Set the function field (MCU_SEL) to the IO MUX function corresponding to GPIO X (this is Function 2—numeric value 2—for all pins).
        // Enable the input by setting the FUN_IE bit.
        // Set or clear the FUN_WPU and FUN_WPD bits, as desired, to enable/disable internal pull-up/pull down resistors.
    REG_VAL(IO_MUX_GPIO23_REG) &= ~(FUN_PU | FUN_PD | MCU_SEL);  // disable pull down and up 
    REG_VAL(IO_MUX_GPIO16_REG) &= ~(FUN_PU | FUN_PD | MCU_SEL); // disable pull down and up
    REG_VAL(IO_MUX_GPIO23_REG) |= FUN_IE | 2 << MCU_SEL_S;      // enable fun_ie and AF to 2
    REG_VAL(IO_MUX_GPIO16_REG) |= FUN_IE | 2 << MCU_SEL_S;     // enable fun_ie and AF to 2
}

void pcnt_isr_attach(void)
{
    const int flags = ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM;

    // Using IDF framework to install interrupt handler because dealing with the vector table will take me way too long 
    // and I don't have the time to spend on that and is relatively out of the scope for the class. 
    esp_err_t err = esp_intr_alloc(48, // signal of PCNT_INTR
                                flags,
                                (intr_handler_t)pcnt_isr,
                                NULL,
                                NULL);
    // check for errors
    if (err != ESP_OK) {
        while (1) { }
    }
}