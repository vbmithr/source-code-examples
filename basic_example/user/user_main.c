#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>

#include "uart.h"
#include "gpio.h"

#define SSID "samarcande"
#define SSID_PASSWORD "fr4b4t043v3r=="

#define DEFAULT_PRIO 2
#define ICACHE_FLASH_ATTR __attribute__((section(".irom0.text")))

static void ICACHE_FLASH_ATTR setup_ap(void *pvParams)
{
    struct station_config stationConf;
    memset(&stationConf, 0, sizeof(struct station_config));

    printf("Task start.");

    /* Set station mode */
    wifi_set_opmode(STATION_MODE);

    /* Set ap settings */
    strncpy(stationConf.ssid, SSID, 32);
    strncpy(stationConf.password, SSID_PASSWORD, 64);
    wifi_station_set_config(&stationConf);


    printf("Deleting task.");
    vTaskDelete(NULL);
}

static int gpio_state = 0b10000;

static void ICACHE_FLASH_ATTR blink(xTimerHandle tmr_hdl)
{
    gpio_output_conf(gpio_state, gpio_state ^ 0b10000, 0b110000, 0);
    gpio_state ^= 0b10000;
}

/* Init function */
void ICACHE_FLASH_ATTR user_init()
{
    uart_init_new();
    gpio_output_conf(0, 0xffff, 0b110000, ~0b110000);

    xTimerHandle xt;
    xt = xTimerCreate((const signed char*) "blink",
		      ( 500 / portTICK_RATE_MS ),
		      pdTRUE,
		      NULL,
		      blink
		      );


    xTimerStart(xt, 46);

    /* Create task */
    xTaskCreate(setup_ap, "setup_ap", 512, NULL, DEFAULT_PRIO, NULL);
}
