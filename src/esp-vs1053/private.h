#ifndef __LIB__VS1053_PRIVATE_H
#define __LIB__VS1053_PRIVATE_H

#include <driver/gpio.h>
#include <driver/spi_master.h>

#include "vs1053.h"
#include "vs1053_player.h"

static const char* TAG = "vs1053";

typedef struct vs1053 {
    gpio_num_t pin_dreq;

    spi_device_handle_t spi_ctrl;
    spi_device_handle_t spi_data;
} vs1053_t;

#endif