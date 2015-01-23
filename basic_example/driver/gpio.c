/*
 *  Copyright (C) 2014 -2016  Espressif System
 *
 */

#include <espressif/esp8266/eagle_soc.h>
#include <espressif/esp8266/pin_mux_register.h>
#include <espressif/esp8266/gpio_register.h>
#include <espressif/esp8266/ets_sys.h>
#include <freertos/portmacro.h>

#include "gpio.h"

int all_gpios[] = {
    GPIO_PIN_REG_0,
    GPIO_PIN_REG_1,
    GPIO_PIN_REG_2,
    GPIO_PIN_REG_3,
    GPIO_PIN_REG_4,
    GPIO_PIN_REG_5,
    GPIO_PIN_REG_6,
    GPIO_PIN_REG_7,
    GPIO_PIN_REG_8,
    GPIO_PIN_REG_9,
    GPIO_PIN_REG_10,
    GPIO_PIN_REG_11,
    GPIO_PIN_REG_12,
    GPIO_PIN_REG_13,
    GPIO_PIN_REG_14,
    GPIO_PIN_REG_15
};

void ICACHE_FLASH_ATTR
gpio_config(gpio_config_t *cfg)
{
    int pin_reg;

    switch(cfg->GPIO_Mode) {
    case GPIO_Mode_Input:
	gpio_output_conf(0, 0, 0, cfg->GPIO_Pin);
	break;
    case GPIO_Mode_Output:
	gpio_output_conf(0, 0, cfg->GPIO_Pin, 0);
	break;
    default:
	break;
    };

    for(int i = 0; i < 16; i++) {
        if (cfg->GPIO_Pin & (1U << i)) {

            if ((1U << i) & 0b110101)
                PIN_FUNC_SELECT(all_gpios[i], 0);
	    else
	    	PIN_FUNC_SELECT(all_gpios[i], 3);

	    if (cfg->GPIO_Pullup)
                PIN_PULLUP_EN(all_gpios[i]);
	    else
                PIN_PULLUP_DIS(all_gpios[i]);

            if (cfg->GPIO_Mode == GPIO_Mode_Out_OD) {
                portENTER_CRITICAL();

                pin_reg = GPIO_REG_READ(GPIO_PIN_ADDR(i));
                pin_reg &= (~GPIO_PIN_DRIVER_MASK);
                pin_reg |= (GPIO_PAD_DRIVER_ENABLE << GPIO_PIN_DRIVER_LSB);
                GPIO_REG_WRITE(GPIO_PIN_ADDR(i), pin_reg);

                portEXIT_CRITICAL();
            }
	    else if (cfg->GPIO_Mode == GPIO_Mode_Sigma_Delta) {
                portENTER_CRITICAL();

                pin_reg = GPIO_REG_READ(GPIO_PIN_ADDR(i));
                pin_reg &= (~GPIO_PIN_SOURCE_MASK);
                pin_reg |= (0x1 << GPIO_PIN_SOURCE_LSB);
                GPIO_REG_WRITE(GPIO_PIN_ADDR(i), pin_reg);
                GPIO_REG_WRITE(GPIO_SIGMA_DELTA_ADDRESS, SIGMA_DELTA_ENABLE);

                portEXIT_CRITICAL();
            }

            gpio_pin_intr_state_set(i, cfg->GPIO_IntrType);
        }
    }
}

/*
 * Change GPIO pin output by setting, clearing, or disabling pins.
 * In general, it is expected that a bit will be set in at most one
 * of these masks.  If a bit is clear in all masks, the output state
 * remains unchanged.
 *
 * There is no particular ordering guaranteed; so if the order of
 * writes is significant, calling code should divide a single call
 * into multiple calls.
 */
void ICACHE_FLASH_ATTR
gpio_output_conf(int set_mask, int clear_mask, int enable_mask, int disable_mask)
{
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, set_mask);
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, clear_mask);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, enable_mask);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, disable_mask);
}

/*
 * Sample the value of GPIO input pins and returns a bitmask.
 */
int ICACHE_FLASH_ATTR
gpio_input_get(void)
{
    return GPIO_REG_READ(GPIO_IN_ADDRESS);
}

/*
 * Register an application-specific interrupt handler for GPIO pin
 * interrupts.  Once the interrupt handler is called, it will not
 * be called again until after a call to gpio_intr_ack.  Any GPIO
 * interrupts that occur during the interim are masked.
 *
 * The application-specific handler is called with a mask of
 * pending GPIO interrupts.  After processing pin interrupts, the
 * application-specific handler may wish to use gpio_intr_pending
 * to check for any additional pending interrupts before it returns.
 */
void ICACHE_FLASH_ATTR
gpio_intr_handler_register(void *fn)
{
    _xt_isr_attach(ETS_GPIO_INUM, fn);
}

/*
  only highlevel and lowlevel intr can use for wakeup
*/
void ICACHE_FLASH_ATTR
gpio_pin_wakeup_enable(int i, gpio_intr_t intr_state)
{
    int pin_reg;

    if ((intr_state == GPIO_PIN_INTR_LOLEVEL) || (intr_state == GPIO_PIN_INTR_HILEVEL)) {
        portENTER_CRITICAL();

        pin_reg = GPIO_REG_READ(GPIO_PIN_ADDR(i));
        pin_reg &= (~GPIO_PIN_INT_TYPE_MASK);
        pin_reg |= (intr_state << GPIO_PIN_INT_TYPE_LSB);
        pin_reg |= GPIO_PIN_WAKEUP_ENABLE_SET(GPIO_WAKEUP_ENABLE);
        GPIO_REG_WRITE(GPIO_PIN_ADDR(i), pin_reg);

        portEXIT_CRITICAL();
    }
}

void ICACHE_FLASH_ATTR
gpio_pin_wakeup_disable(void)
{
    int pin_reg;

    for (int i = 0; i < GPIO_PIN_COUNT; i++) {
        pin_reg = GPIO_REG_READ(GPIO_PIN_ADDR(i));

        if (pin_reg & GPIO_PIN_WAKEUP_ENABLE_MASK) {
            pin_reg &= (~GPIO_PIN_INT_TYPE_MASK);
            pin_reg |= (GPIO_PIN_INTR_DISABLE << GPIO_PIN_INT_TYPE_LSB);
            pin_reg &= ~(GPIO_PIN_WAKEUP_ENABLE_SET(GPIO_WAKEUP_ENABLE));
            GPIO_REG_WRITE(GPIO_PIN_ADDR(i), pin_reg);
        }
    }
}

void ICACHE_FLASH_ATTR
gpio_pin_intr_state_set(int i, gpio_intr_t intr_state)
{
    int pin_reg;

    portENTER_CRITICAL();

    pin_reg = GPIO_REG_READ(GPIO_PIN_ADDR(i));
    pin_reg &= (~GPIO_PIN_INT_TYPE_MASK);
    pin_reg |= (intr_state << GPIO_PIN_INT_TYPE_LSB);
    GPIO_REG_WRITE(GPIO_PIN_ADDR(i), pin_reg);

    portEXIT_CRITICAL();
}

void ICACHE_FLASH_ATTR
gpio16_output_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | 1); // mux configuration for XPD_DCDC to output rtc_gpio0

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & 0xfffffffe)); //mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   (READ_PERI_REG(RTC_GPIO_ENABLE) & 0xfffffffe) | 0x1); //out enable
}

void ICACHE_FLASH_ATTR
gpio16_output_set(int value)
{
    WRITE_PERI_REG(RTC_GPIO_OUT,
                   (READ_PERI_REG(RTC_GPIO_OUT) & 0xfffffffe) | (value & 1));
}

void ICACHE_FLASH_ATTR
gpio16_input_conf(void)
{
    WRITE_PERI_REG(PAD_XPD_DCDC_CONF,
                   (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | 1); // mux configuration for XPD_DCDC and rtc_gpio0 connection

    WRITE_PERI_REG(RTC_GPIO_CONF,
                   (READ_PERI_REG(RTC_GPIO_CONF) & 0xfffffffe)); //mux configuration for out enable

    WRITE_PERI_REG(RTC_GPIO_ENABLE,
                   READ_PERI_REG(RTC_GPIO_ENABLE) & 0xfffffffe); //out disable
}

int ICACHE_FLASH_ATTR
gpio16_input_get(void)
{
    return (READ_PERI_REG(RTC_GPIO_IN_DATA) & 1);
}

