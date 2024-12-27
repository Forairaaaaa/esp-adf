/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2019 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>
#include "i2c_bus.h"
#include "board.h"
#include "esp_log.h"
#include "aw88298.h"
#include "audio_volume.h"

#define AW88298_ADDR         0x36

#define ES_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static i2c_bus_handle_t i2c_handle;

audio_hal_func_t AUDIO_CODEC_AW88298_DEFAULT_HANDLE = {
    .audio_codec_initialize = aw88298_codec_init,
    .audio_codec_deinitialize = NULL,
    .audio_codec_ctrl = NULL,
    .audio_codec_config_iface = NULL,
    .audio_codec_set_mute = NULL,
    .audio_codec_set_volume = NULL,
    .audio_codec_get_volume = NULL,
    .audio_codec_enable_pa = NULL,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

static char *TAG = "AW88298";

static int i2c_init()
{
    int res = 0;
    i2c_config_t es_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    res = get_i2c_pins(I2C_NUM_0, &es_i2c_cfg);
    ES_ASSERT(res, "getting i2c pins error", -1);
    i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
    return res;
}

// static esp_err_t aw88298_write_reg(int reg, int value)
// {
//     return i2c_bus_write_bytes(i2c_handle, AW88298_ADDR, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
// }

// static int aw88298_read_reg(int reg, int value)
// {
//     uint8_t data;
//     i2c_bus_read_bytes(i2c_handle, AW88298_ADDR, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
//     return (int)data;
// }

static esp_err_t aw88298_write_reg(uint8_t reg, int value)
{
    uint8_t write_data[2] = {(uint8_t) ((value & 0xFFFF) >> 8), (uint8_t) (value & 0xFF)};
    return i2c_bus_write_bytes(i2c_handle, AW88298_ADDR, &reg, 1, &write_data, sizeof(write_data));
}

static int aw88298_read_reg(uint8_t reg, int *value)
{
    int ret = 0;
    uint8_t read_data[2] = {0};
    i2c_bus_read_bytes(i2c_handle, AW88298_ADDR, &reg, 1, &read_data, sizeof(read_data));
    *value = ((int)read_data[0] << 8) | read_data[1];
    return ret;
}

esp_err_t aw88298_codec_init(audio_hal_codec_config_t *codec_cfg)
{
    esp_err_t ret = ESP_OK;
    i2c_init(); // ESP32 in master mode

    ret |= aw88298_write_reg(AW88298_RESET_REG00, 0x55aa); // Reset chip
    ret |= aw88298_write_reg(AW88298_SYSCTRL_REG04, 0x4040); // I2SEN=1 AMPPD=0 PWDN=0
    ret |= aw88298_write_reg(AW88298_SYSCTRL2_REG05, 0x0008); // RMSE=0 HAGCE=0 HDCCE=0 HMUTE=0
    ret |= aw88298_write_reg(AW88298_I2SCTRL_REG06, 0x3CC8); // I2SBCK=0 (BCK mode 16*2)
    ret |= aw88298_write_reg(AW88298_HAGCCFG4_REG0C, 0x3064); // volume setting
    ret |= aw88298_write_reg(AW88298_BSTCTRL2_REG61, 0x0673); // default:0x6673: BOOST mode disabled

    if (ret != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
