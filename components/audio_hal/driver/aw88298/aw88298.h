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

#ifndef _AW88298_H
#define _AW88298_H

#include "audio_hal.h"
#include "esp_types.h"
#include "esxxx_common.h"

/* AW88298 register space */

/*
 * RESET Control
 */
#define AW88298_RESET_REG00           0x00

/*
 * System Control
 */
#define AW88298_SYSST_REG01           0x01

/*
 * System interrupt
 */
#define AW88298_SYSINT_REG02          0x02
#define AW88298_SYSINTM_REG03         0x03

/*
 * System Control
 */
#define AW88298_SYSCTRL_REG04         0x04
#define AW88298_SYSCTRL2_REG05        0x05

/*
 * I2S control and config
 */
#define AW88298_I2SCTRL_REG06         0x06
#define AW88298_I2SCFG1_REG07         0x07

/*
 * HAGC config
 */
#define AW88298_HAGCCFG1_REG09        0x09
#define AW88298_HAGCCFG2_REG0A        0x0a
#define AW88298_HAGCCFG3_REG0B        0x0b
#define AW88298_HAGCCFG4_REG0C        0x0C

/*
 * HAGC boost output voltage
 */
#define AW88298_HAGCST_REG10          0x10

/*
 * Detected voltage of battery
 */
#define AW88298_VDD_REG12             0x12

/*
 * Detected die temperature
 */
#define AW88298_TEMP_REG13            0x13

/*
 * Detected voltage of PVDD
 */
#define AW88298_PVDD_REG14            0x14

/*
 * Smart boost control
 */
#define AW88298_BSTCTRL1_REG60        0x60
#define AW88298_BSTCTRL2_REG61        0x61

/*
 * Chip Information
 */
#define AW88298_CHIP_VERSION_REG00    0x00 // ID: 1852h

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t aw88298_codec_init(audio_hal_codec_config_t *codec_cfg);
esp_err_t aw88298_codec_deinit();
esp_err_t aw88298_codec_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);
esp_err_t aw88298_codec_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface);
esp_err_t aw88298_set_voice_mute(bool enable);
esp_err_t aw88298_codec_set_voice_volume(int volume);
esp_err_t aw88298_codec_get_voice_volume(int *volume);
esp_err_t aw88298_pa_power(bool enable);

#ifdef __cplusplus
}
#endif

#endif
