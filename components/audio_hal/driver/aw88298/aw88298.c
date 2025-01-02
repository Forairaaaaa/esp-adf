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
static codec_dac_volume_config_t *dac_vol_handle;

#define AW88298_DAC_VOL_CFG_DEFAULT() {                      \
    .max_dac_volume = 32,                                   \
    .min_dac_volume = -95.5,                                \
    .board_pa_gain = BOARD_PA_GAIN,                         \
    .volume_accuracy = 0.5,                                 \
    .dac_vol_symbol = 1,                                    \
    .zero_volume_reg = 0xBF,                                \
    .reg_value = 0,                                         \
    .user_volume = 0,                                       \
    .offset_conv_volume = NULL,                             \
}

audio_hal_func_t AUDIO_CODEC_AW88298_DEFAULT_HANDLE = {
    .audio_codec_initialize = aw88298_codec_init,
    .audio_codec_deinitialize = aw88298_codec_deinit,
    .audio_codec_ctrl = aw88298_codec_ctrl_state,
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

static int aw88298_set_bits_per_sample(uint8_t bits)
{
    int ret = 0;
    int dac_iface = 0;
    ret = aw88298_read_reg(AW88298_I2SCTRL_REG06, &dac_iface);
    dac_iface &= ~(0xF0);
    switch (bits) {
        case 16:
        default:
            break;
        case 24:
            dac_iface |= 0x90;
            break;
        case 32:
            dac_iface |= 0xE0;
            break;
    }
    ret |= aw88298_write_reg(AW88298_I2SCTRL_REG06, dac_iface);
    ESP_LOGD(TAG, "Bits %d", bits);
    return ret;
}

static int aw88298_config_sample(int sample_rate)
{
    int ret = 0;
    int dac_iface = 0;
    ret = aw88298_read_reg(AW88298_I2SCTRL_REG06, &dac_iface);
    dac_iface &= ~(0x0F);
    switch (sample_rate) {
        case 8000:
            break;
        case 11025:
            dac_iface |= 0x01;
            break;
        case 12000:
            dac_iface |= 0x02;
            break;
        case 16000:
            dac_iface |= 0x03;
            break;
        case 22050:
            dac_iface |= 0x04;
            break;
        case 24000:
            dac_iface |= 0x05;
            break;
        case 32000:
            dac_iface |= 0x06;
            break;
        case 44100:
            dac_iface |= 0x07;
            break;
        case 48000:
            dac_iface |= 0x08;
            break;
        case 96000:
            dac_iface |= 0x09;
            break;
        case 192000:
            dac_iface |= 0x0A;
            break;
        default:
            ESP_LOGE(TAG, "Sample rate(%d) can not support", sample_rate);
            return ESP_FAIL;
    }
    ESP_LOGD(TAG, "Current sample rate: %d", sample_rate);
    ret |= aw88298_write_reg(AW88298_I2SCTRL_REG06, dac_iface);
    return ret;
}

static int aw88298_stop()
{
    int ret = 0;
    int data = 0;
    ret |= aw88298_read_reg(AW88298_SYSCTRL_REG04, &data);
    data |= 0x03;
    data &= ~(1 << 6);
    ret |= aw88298_write_reg(AW88298_SYSCTRL_REG04, data);
    return (ret == 0) ? ESP_OK : ESP_FAIL;;
}

static int aw88298_start()
{
    int ret = 0;
    int data = 0;
    ret |= aw88298_read_reg(AW88298_SYSCTRL_REG04, &data);
    data &= ~0x03;
    data |= (1 << 6);
    ret |= aw88298_write_reg(AW88298_SYSCTRL_REG04, data);
    return (ret == 0) ? ESP_OK : ESP_FAIL;;
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

    // Sample rate
    audio_hal_codec_i2s_iface_t *i2s_cfg = &(codec_cfg->i2s_iface);
    int sample_fre = 0;
    switch (i2s_cfg->samples) {
        case AUDIO_HAL_08K_SAMPLES:
            sample_fre = 8000;
            break;
        case AUDIO_HAL_11K_SAMPLES:
            sample_fre = 11025;
            break;
        case AUDIO_HAL_16K_SAMPLES:
            sample_fre = 16000;
            break;
        case AUDIO_HAL_22K_SAMPLES:
            sample_fre = 22050;
            break;
        case AUDIO_HAL_24K_SAMPLES:
            sample_fre = 24000;
            break;
        case AUDIO_HAL_32K_SAMPLES:
            sample_fre = 32000;
            break;
        case AUDIO_HAL_44K_SAMPLES:
            sample_fre = 44100;
            break;
        case AUDIO_HAL_48K_SAMPLES:
            sample_fre = 48000;
            break;
        default:
            ESP_LOGE(TAG, "Unable to configure sample rate %dHz", sample_fre);
            break;
    }
    if (aw88298_config_sample(sample_fre) != ESP_OK) {
        return ESP_FAIL;
    }

    codec_dac_volume_config_t vol_cfg = AW88298_DAC_VOL_CFG_DEFAULT();
    dac_vol_handle = audio_codec_volume_init(&vol_cfg);

    return ESP_OK;
}

esp_err_t aw88298_codec_deinit()
{
    aw88298_stop();
    i2c_bus_delete(i2c_handle);
    audio_codec_volume_deinit(dac_vol_handle);
    return ESP_OK;
}

esp_err_t aw88298_codec_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    esp_err_t ret = ESP_OK;

    if (ctrl_state == AUDIO_HAL_CTRL_START) {
        ret |= aw88298_start();
    } else {
        ESP_LOGW(TAG, "The codec is about to stop");
        ret |= aw88298_stop();
    }

    return ret;
}
