#ifndef __LIB_VS1053_H
#define __LIB_VS1053_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>

typedef enum vs1053_sci_reg {
    VS1053_SCI_MODE = 0x00,
    VS1053_SCI_STATUS = 0x01,
    VS1053_SCI_CLOCKF = 0x03,
    VS1053_SCI_DECODE_TIME = 0x04,
    VS1053_SCI_WRAM = 0x06,
    VS1053_SCI_WRAMADDR = 0x07,
    VS1053_SCI_HDAT0 = 0x08,
    VS1053_SCI_HDAT1 = 0x09,
    VS1053_SCI_VOL = 0x0B,
} vs1053_sci_reg_t;

#define VS1053_SM_LAYER12 (1 << 1)
#define VS1053_SM_RESET (1 << 2)
#define VS1053_SM_CANCEL (1 << 3)
#define VS1053_SM_TESTS (1 << 5)

#define VS1053_SS_DO_NOT_JUMP (1 << 15)
#define VS1053_SS_VER (0xF << 4)

#define VS1053_WRAM_VERSION 0x1E02
#define VS1053_WRAM_ENDFILLBYTE 0x1E06
#define VS1053_WRAM_RESYNC 0x1E29

typedef struct vs1053 vs1053_t;
typedef vs1053_t* vs1053_handle_t;

// Register the VS1053 on the given SPI bus (including managing CS and DCS), but don't send any traffic yet.
void vs1053_init(spi_host_device_t host, gpio_num_t pin_cs, gpio_num_t pin_dcs, gpio_num_t pin_dreq, vs1053_handle_t* dev);

// Soft reset the device and program reasonable a configuration, e.g. clock speed defaults.
esp_err_t vs1053_reconfigure_to_defaults(vs1053_handle_t dev);

// These functions are not thread safe.
//
// Note that these functions delay for the maximum amount of time an SCI read
// or SCI write may take to be processed, so DREQ does not need to be polled
// before calling them due to the FIFO "safety area" (see VS1053) datasheet.
// However, an additional delay may be required if e.g. a soft reset is
// commanded by a call to `vs1053_sci_write()`.
uint16_t vs1053_sci_read(vs1053_handle_t dev, vs1053_sci_reg_t addr);
void vs1053_sci_write(vs1053_handle_t dev, vs1053_sci_reg_t addr, uint16_t val);

// `vs1053_sc_mult` is the multiplier applied to XTALI to produce CLKI.
// Note: When the multiplier is changed by more than x0.5, the chip runs at `1.0 x XTALI` for a few hundred clock cycles.
typedef enum vs1053_sc_mult {
    VS1053_SC_MULT_x1_0 = 0,
    VS1053_SC_MULT_x2_0 = 1,
    VS1053_SC_MULT_x2_5 = 2,
    VS1053_SC_MULT_x3_0 = 3,
    VS1053_SC_MULT_x3_5 = 4,
    VS1053_SC_MULT_x4_0 = 5,
    VS1053_SC_MULT_x4_5 = 6,
    VS1053_SC_MULT_x5_0 = 7,
} vs1053_sc_mult_t;

// `vs1053_sc_add` is how much the decoder firmware is alloed to add to the multiplier specified by
// `vs1053_sc_mult` if more cycles are temporarily needed to decode a WMA or AAC stream.
typedef enum vs1053_sc_add {
    VS1053_SC_ADD_x0_0 = 0,
    VS1053_SC_ADD_x1_0 = 1,
    VS1053_SC_ADD_x1_5 = 2,
    VS1053_SC_ADD_x2_0 = 3,
} vs1053_sc_add_t;

// Indicates that the input clock XTALI is running at 12.288 MHz. Otherwise, a certain formula
// from the datasheet must be used to compute the correct value.
#define VS1053_SC_FREQ_DEFAULT (0)

void vs1053_ctrl_set_clockfreq(vs1053_handle_t dev,
                               vs1053_sc_mult_t sc_mult, vs1053_sc_add_t sc_add, uint16_t sc_freq);

void vs1053_ctrl_soft_reset(vs1053_handle_t dev);

void vs1053_ctrl_allow_tests(vs1053_handle_t dev, bool enable);

// Note: 0x00 is loudest, 0xFE is quietest, 0xFF is analog powerdown mode.
void vs1053_ctrl_set_volume(vs1053_handle_t dev, uint8_t left, uint8_t right);

void vs1053_data_transmit(vs1053_handle_t dev, uint8_t* buff, uint16_t len);

void vs1053_test_sine(vs1053_handle_t dev, uint8_t n);

#endif