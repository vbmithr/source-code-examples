/*
 *  Copyright (C) 2014 -2016  Espressif System
 *
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdint.h>

#define GPIO_PIN_REG_0          PERIPHS_IO_MUX_GPIO0_U
#define GPIO_PIN_REG_1          PERIPHS_IO_MUX_U0TXD_U
#define GPIO_PIN_REG_2          PERIPHS_IO_MUX_GPIO2_U
#define GPIO_PIN_REG_3          PERIPHS_IO_MUX_U0RXD_U
#define GPIO_PIN_REG_4          PERIPHS_IO_MUX_GPIO4_U
#define GPIO_PIN_REG_5          PERIPHS_IO_MUX_GPIO5_U
#define GPIO_PIN_REG_6          PERIPHS_IO_MUX_SD_CLK_U
#define GPIO_PIN_REG_7          PERIPHS_IO_MUX_SD_DATA0_U
#define GPIO_PIN_REG_8          PERIPHS_IO_MUX_SD_DATA1_U
#define GPIO_PIN_REG_9          PERIPHS_IO_MUX_SD_DATA2_U
#define GPIO_PIN_REG_10         PERIPHS_IO_MUX_SD_DATA3_U
#define GPIO_PIN_REG_11         PERIPHS_IO_MUX_SD_CMD_U
#define GPIO_PIN_REG_12         PERIPHS_IO_MUX_MTDI_U
#define GPIO_PIN_REG_13         PERIPHS_IO_MUX_MTCK_U
#define GPIO_PIN_REG_14         PERIPHS_IO_MUX_MTMS_U
#define GPIO_PIN_REG_15         PERIPHS_IO_MUX_MTDO_U

#define GPIO_PIN_ADDR(i)        (GPIO_PIN0_ADDRESS + i*4)

#define GPIO_ID_IS_PIN_REGISTER(reg_id) \
    ((reg_id >= GPIO_ID_PIN0) && (reg_id <= GPIO_ID_PIN(GPIO_PIN_COUNT-1)))

#define GPIO_REGID_TO_PINIDX(reg_id) ((reg_id) - GPIO_ID_PIN0)

#define ICACHE_FLASH_ATTR __attribute__((section(".irom0.text")))

typedef enum {
    GPIO_PIN_INTR_DISABLE,
    GPIO_PIN_INTR_POSEDGE,
    GPIO_PIN_INTR_NEGEDGE,
    GPIO_PIN_INTR_ANYEGDE,
    GPIO_PIN_INTR_LOLEVEL,
    GPIO_PIN_INTR_HILEVEL
} gpio_intr_t;

typedef enum {
    GPIO_Mode_Input,
    GPIO_Mode_Out_OD,
    GPIO_Mode_Output,
    GPIO_Mode_Sigma_Delta
} gpio_mode_t;

typedef struct {
    uint16_t GPIO_Pin;
    uint8_t GPIO_Mode;
    uint8_t GPIO_Pullup;
    uint8_t GPIO_IntrType;
} gpio_config_t;

void gpio16_output_conf(void);
void gpio16_output_set(int value);

void gpio16_input_conf(void);
int gpio16_input_get(void);

void gpio_output_conf(int set_mask, int clear_mask, int enable_mask, int disable_mask);

void gpio_intr_handler_register(void *fn);

void gpio_pin_wakeup_enable(int pin, gpio_intr_t intr_state);
void gpio_pin_wakeup_disable();

void gpio_pin_intr_state_set(int pin, gpio_intr_t intr_state);

int gpio_input_get(void);

#endif
