#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <libesp.h>

#include "private.h"

#define CMD_READ 0b00000011
#define CMD_WRITE 0b00000010

void vs1053_init(spi_host_device_t host, gpio_num_t pin_cs, gpio_num_t pin_dcs, gpio_num_t pin_dreq, vs1053_handle_t* out_dev) {
    spi_device_interface_config_t devcfg_ctrl = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = pin_cs,
        .queue_size = 1,
        .input_delay_ns = 0,
    };

    spi_device_interface_config_t devcfg_data = {
        .clock_speed_hz = 3000000,
        .mode = 0,
        .spics_io_num = pin_dcs,
        .queue_size = 1,
        .input_delay_ns = 0,
    };

    vs1053_handle_t dev = malloc(sizeof(vs1053_t));
    dev->pin_dreq = pin_dreq;
    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg_ctrl, &dev->spi_ctrl));
    ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg_data, &dev->spi_data));

    *out_dev = dev;
}

#define VSxxxx_SHIFT_VER 4

#define VSxxxx_VER_1001 0
#define VSxxxx_VER_1011 1
#define VSxxxx_VER_1002 2
#define VSxxxx_VER_1003 3
#define VSxxxx_VER_1053_or_8053 4
#define VSxxxx_VER_1033 5
#define VSxxxx_VER_1063 6
#define VSxxxx_VER_1103 7

static esp_err_t check_version(vs1053_handle_t dev) {
    uint16_t ver = (vs1053_sci_read(dev, VS1053_SCI_STATUS) & VS1053_SS_VER) >> VSxxxx_SHIFT_VER;
    if (ver != VSxxxx_VER_1053_or_8053) {
        ESP_LOGE(TAG, "bad version (%d), check connected?", ver);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t vs1053_reconfigure_to_defaults(vs1053_handle_t dev) {
    esp_err_t ret;

    // Check this is a VS1053.
    if ((ret = check_version(dev)) != ESP_OK) {
        return ret;
    }

    vs1053_ctrl_soft_reset(dev);

    // Recommended default clock freq as per datasheet.
    vs1053_ctrl_set_clockfreq(dev, VS1053_SC_MULT_x3_5, VS1053_SC_ADD_x1_0, VS1053_SC_FREQ_DEFAULT);
    vs1053_sci_write(dev, VS1053_SCI_MODE, vs1053_sci_read(dev, VS1053_SCI_MODE) | VS1053_SM_LAYER12);

    // Note: the Adafruit VS1053 library does this---why should we?
    // Disable resync functionality
    // vs1053_sci_write(dev, VS1053_SCI_WRAMADDR, VS1053_WRAM_RESYNC);
    // vs1053_sci_write(dev, VS1053_SCI_WRAM, 0x0000);

    // Read the WRAM struct version, and ensure it is 0x0003 for the VS1053b, which is
    // the one we understand.
    vs1053_sci_write(dev, VS1053_SCI_WRAMADDR, VS1053_WRAM_VERSION);
    uint16_t struct_ver = vs1053_sci_read(dev, VS1053_SCI_WRAM);
    if (struct_ver != 0x0003) {
        ESP_LOGE(TAG, "unsupported extra parameter struct version: 0x%04X", struct_ver);
        return ESP_FAIL;
    }

    // Check this is a VS1053.
    if ((ret = check_version(dev)) != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

uint16_t vs1053_sci_read(vs1053_handle_t dev, vs1053_sci_reg_t addr) {
    spi_transaction_t trans = {
        .length = 32,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = {CMD_READ, addr, 0, 0},
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi_ctrl, &trans));
    util_wait_micros(20);

    uint16_t rx = (((uint16_t) trans.rx_data[2]) << 8) | (((uint16_t) trans.rx_data[3]) << 0);
    ESP_LOGD(TAG, "sci_read(0x%02X)=0x%04X", addr, rx);
    return rx;
}

void vs1053_sci_write(vs1053_handle_t dev, vs1053_sci_reg_t addr, uint16_t val) {
    spi_transaction_t trans = {
        .length = 32,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = {CMD_WRITE, addr, (val & (0xFFULL << (8 * 1))) >> (8 * 1), (val & (0xFFULL << (8 * 0))) >> (8 * 0)},
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi_ctrl, &trans));
    util_wait_micros(20);

    ESP_LOGD(TAG, "sci_write(0x%02X)=0x%04X", addr, val);
}

void vs1053_ctrl_soft_reset(vs1053_handle_t dev) {
    vs1053_sci_write(dev, VS1053_SCI_MODE, vs1053_sci_read(dev, VS1053_SCI_MODE) | VS1053_SM_RESET);

    // As per datasheet, `SS_DO_NOT_JUMP` may (erroneously) not be cleared during a soft reset.
    uint16_t status = vs1053_sci_read(dev, VS1053_SCI_STATUS);
    vs1053_sci_write(dev, VS1053_SCI_STATUS, status & ~VS1053_SS_DO_NOT_JUMP);

    util_wait_micros(2);

    while (gpio_get_level(dev->pin_dreq) == 0) {
        vTaskDelay(1);
    }
}

void vs1053_ctrl_allow_tests(vs1053_handle_t dev, bool enable) {
    uint16_t mode = vs1053_sci_read(dev, VS1053_SCI_MODE);
    vs1053_sci_write(dev, VS1053_SCI_MODE, (mode & ~VS1053_SM_TESTS) | (enable ? VS1053_SM_TESTS : 0));
}

void vs1053_ctrl_set_volume(vs1053_handle_t dev, uint8_t left, uint8_t right) {
    vs1053_sci_write(dev, VS1053_SCI_VOL, (((uint16_t) left) << 8) | (((uint16_t) right) << 0));
}

void vs1053_ctrl_set_clockfreq(vs1053_handle_t dev,
                               vs1053_sc_mult_t sc_mult, vs1053_sc_add_t sc_add, uint16_t sc_freq) {
    if (sc_freq & ~0x7FF) {
        ESP_LOGE(TAG, "vs1053_ctrl_set_clockfreq: sc_freq out of range");
        return;
    }

    vs1053_sci_write(dev, VS1053_SCI_CLOCKF,
                     (((uint16_t) sc_mult) << 13) | (((uint16_t) sc_add) << 11) | (((uint16_t) sc_freq) << 0));
    vTaskDelay(1);
}

void vs1053_data_transmit(vs1053_handle_t dev, uint8_t* buff, uint16_t len) {
    spi_transaction_t trans = {
        .length = 8 * len,
        .tx_buffer = buff,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi_data, &trans));
}

#define TEST_SINE_BYTE0 0x53
#define TEST_SINE_BYTE1 0xEF
#define TEST_SINE_BYTE2 0x6E

void vs1053_test_sine(vs1053_handle_t dev, uint8_t n) {
    uint8_t* buff = heap_caps_malloc(MALLOC_CAP_DMA, 8);
    buff[0] = TEST_SINE_BYTE0;
    buff[1] = TEST_SINE_BYTE1;
    buff[2] = TEST_SINE_BYTE2;
    buff[3] = n;
    buff[4] = 0x00;
    buff[5] = 0x00;
    buff[6] = 0x00;
    buff[7] = 0x00;

    vs1053_data_transmit(dev, buff, 8);

    free(buff);
}