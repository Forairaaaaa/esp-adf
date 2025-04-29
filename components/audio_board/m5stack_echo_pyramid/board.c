/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2022 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
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

#include "esp_log.h"
#include "board.h"
#include "audio_mem.h"
#include "periph_sdcard.h"
#include "periph_adc_button.h"
#include "tca9554.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "string.h"
#include "audio_hal/driver/es7210/es7210.h"

static const char *TAG = "Echo-Pyramid";

static audio_board_handle_t board_handle = 0;
static i2c_master_bus_handle_t board_i2c_bus_handle;
static i2c_master_dev_handle_t baord_si5351_handle;
static i2c_master_dev_handle_t baord_aw87559_handle;

#define setbit(data, bit) ((data) |= (1 << (bit)))
#define clrbit(data, bit) ((data) &= ~(1 << (bit)))

/* --------------------------------- I2C Bus -------------------------------- */
static void audio_board_i2c_init()
{
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_38,
        .scl_io_num = GPIO_NUM_39,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &board_i2c_bus_handle));
}

static void audio_board_i2c_deinit()
{
    ESP_ERROR_CHECK(i2c_del_master_bus(board_i2c_bus_handle));
}

static void i2c_detect(i2c_master_bus_handle_t bus_handle)
{
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(bus_handle, address, pdMS_TO_TICKS(200));
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
}

static esp_err_t i2c_dev_write_regs(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t w_buf[len + 1];
    w_buf[0] = reg;
    memcpy(w_buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(i2c_dev, w_buf, len + 1, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    return ret;
}

static esp_err_t i2c_dev_write_reg_8(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t data)
{
    uint8_t w_buf[2];
    w_buf[0] = reg;
    w_buf[1] = data;
    esp_err_t ret = i2c_master_transmit(i2c_dev, w_buf, 2, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    return ret;
}

static esp_err_t i2c_dev_write_reg_16(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint16_t data)
{
    uint8_t w_buf[3];
    w_buf[0] = reg;
    w_buf[1] = (uint8_t)((data & 0xFFFF) >> 8);
    w_buf[2] = (uint8_t)(data & 0xFF);
    esp_err_t ret = i2c_master_transmit(i2c_dev, w_buf, 3, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    return ret;
}

static uint8_t i2c_dev_read_reg_8(i2c_master_dev_handle_t i2c_dev, uint8_t reg)
{
    uint8_t w_buf[1];
    uint8_t r_buf[1];
    w_buf[0] = reg;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev, w_buf, 1, r_buf, 1, portMAX_DELAY));
    return r_buf[0];
}

/* --------------------------------- SI5351 --------------------------------- */
static void audio_board_si5351_init()
{
    ESP_LOGI(TAG, "Init SI5351");

    i2c_device_config_t si5351_config = {};
    si5351_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    si5351_config.device_address = 0x60;
    si5351_config.scl_speed_hz = 400000;
    i2c_master_bus_add_device(board_i2c_bus_handle, &si5351_config, &baord_si5351_handle);

    uint8_t w_buffer[10] = {0};

    // Disable all outputs
    i2c_dev_write_reg_8(baord_si5351_handle, 3, 0xff);
    printf("3 : %02X\n", i2c_dev_read_reg_8(baord_si5351_handle, 3));

    // power down output drivers
    w_buffer[0] = 0x80;  // 10000000
    w_buffer[1] = 0x80;
    w_buffer[2] = 0x80;
    i2c_dev_write_regs(baord_si5351_handle, 16, w_buffer, 3);
    for (int i = 0; i < 8; i++) {
        printf("%d : %02X\n", 16 + i, i2c_dev_read_reg_8(baord_si5351_handle, 16 + i));
    }

    // Crystal Internal Load Capacitance
    i2c_dev_write_reg_8(baord_si5351_handle, 183, 0xC0);  // 11000000 // Internal CL = 10 pF (default). 
    printf("183 : %02X\n", i2c_dev_read_reg_8(baord_si5351_handle, 183));

    // Multisynth NA Parameters
    w_buffer[0] = 0xFF;
    w_buffer[1] = 0xFD;
    w_buffer[2] = 0x00;
    w_buffer[3] = 0x09;
    w_buffer[4] = 0x26;
    w_buffer[5] = 0xF7;
    w_buffer[6] = 0x4F;
    w_buffer[7] = 0x72;
    i2c_dev_write_regs(baord_si5351_handle, 26, w_buffer, 8);
    for (int i = 0; i < 8; i++) {
        printf("%d : %02X\n", 26 + i, i2c_dev_read_reg_8(baord_si5351_handle, 26 + i));
    }

    // Multisynth1 Parameters
    w_buffer[0] = 0x00;
    w_buffer[1] = 0x01;
    w_buffer[2] = 0x00;
    w_buffer[3] = 0x2F;
    w_buffer[4] = 0x00;
    w_buffer[5] = 0x00;
    w_buffer[6] = 0x00;
    w_buffer[7] = 0x00;
    i2c_dev_write_regs(baord_si5351_handle, 50, w_buffer, 8);
    for (int i = 0; i < 8; i++) {
        printf("%d : %02X\n", 50 + i, i2c_dev_read_reg_8(baord_si5351_handle, 50 + i));
    }

    // CLK1 Control
    i2c_dev_write_reg_8(baord_si5351_handle, 17, ((3 << 2) | (1 << 6)));  // 01001100 // 0x4c
    // 1: MS1 operates in integer mode.
    // 11: Select MultiSynth 1 as the source for CLK1. Select this option when using the Si5351 to generate
    // free-running or synchronous clocks.
    printf("17 : %02X\n", i2c_dev_read_reg_8(baord_si5351_handle, 17));

    // PLL Reset
    i2c_dev_write_reg_8(baord_si5351_handle, 177, 0xA0);  // 10100000
    // WriteReg(177, 0xAC);  // 10101100
    printf("177 : %02X\n", i2c_dev_read_reg_8(baord_si5351_handle, 177));

    // Enable all outputs
    i2c_dev_write_reg_8(baord_si5351_handle, 3, 0x00);
    printf("3 : %02X\n", i2c_dev_read_reg_8(baord_si5351_handle, 3));
}

/* --------------------------------- AW87559 -------------------------------- */
static void audio_board_aw87559_init()
{
    ESP_LOGI(TAG, "Init AW87559");

    i2c_device_config_t aw87559_config = {};
    aw87559_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    aw87559_config.device_address = 0x5B;
    aw87559_config.scl_speed_hz = 400000;
    i2c_master_bus_add_device(board_i2c_bus_handle, &aw87559_config, &baord_aw87559_handle);

    printf("aw87559 id: %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x00));

    uint8_t data;

    data = i2c_dev_read_reg_8(baord_aw87559_handle, 0x01);
    setbit(data, 6);
    setbit(data, 4);
    setbit(data, 3);
    i2c_dev_write_reg_8(baord_aw87559_handle, 0x01, data);
    printf("0x01 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x01));

    i2c_dev_write_reg_8(baord_aw87559_handle, 0x02, 0b01111011);
    printf("0x02 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x02));

    i2c_dev_write_reg_8(baord_aw87559_handle, 0x03, 0b00001100);
    printf("0x03 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x03));

    data = i2c_dev_read_reg_8(baord_aw87559_handle, 0x05);
    data &= 0b11110000;
    data |= 0b00001100;
    i2c_dev_write_reg_8(baord_aw87559_handle, 0x05, data);
    printf("0x05 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x05));

    i2c_dev_write_reg_8(baord_aw87559_handle, 0x06, 0b00010010);  // 27db
    printf("0x06 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x06));

    data = i2c_dev_read_reg_8(baord_aw87559_handle, 0x07);
    data &= 0b11100000;
    data |= 0b00001111;
    i2c_dev_write_reg_8(baord_aw87559_handle, 0x07, data);
    printf("0x07 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x07));

    i2c_dev_write_reg_8(baord_aw87559_handle, 0x09, 0b00001010);
    printf("0x09 : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x09));

    i2c_dev_write_reg_8(baord_aw87559_handle, 0x0c, 0b00000001);
    printf("0x0c : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x0c));

    i2c_dev_write_reg_8(baord_aw87559_handle, 0x0f, 0b00000000);
    printf("0x0f : %02X\n", i2c_dev_read_reg_8(baord_aw87559_handle, 0x0f));
}

audio_board_handle_t audio_board_init(void)
{
    if (board_handle) {
        ESP_LOGW(TAG, "The board has already been initialized!");
        return board_handle;
    }
    board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);

    vTaskDelay(pdMS_TO_TICKS(100));
    audio_board_i2c_init();
    i2c_detect(board_i2c_bus_handle);
    audio_board_si5351_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    audio_board_aw87559_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    audio_board_i2c_deinit();

    board_handle->audio_hal = audio_board_codec_init();
    board_handle->adc_hal = audio_board_adc_init();
    return board_handle;
}

audio_hal_handle_t audio_board_codec_init(void)
{
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_hal_handle_t codec_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8311_DEFAULT_HANDLE);
    AUDIO_NULL_CHECK(TAG, codec_hal, return NULL);
    return codec_hal;
}

audio_hal_handle_t audio_board_adc_init(void)
{
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_hal_handle_t adc_hal = NULL;
    adc_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES7210_DEFAULT_HANDLE);
    AUDIO_NULL_CHECK(TAG, adc_hal, return NULL);
    return adc_hal;
}

esp_err_t audio_board_key_init(esp_periph_set_handle_t set)
{
    periph_adc_button_cfg_t adc_btn_cfg = PERIPH_ADC_BUTTON_DEFAULT_CONFIG();
    adc_arr_t adc_btn_tag = ADC_DEFAULT_ARR();
    adc_btn_tag.total_steps = 6;
    adc_btn_tag.adc_ch = ADC1_CHANNEL_4;
    int btn_array[7] = {190, 600, 1000, 1375, 1775, 2195, 3000};
    adc_btn_tag.adc_level_step = btn_array;
    adc_btn_cfg.arr = &adc_btn_tag;
    adc_btn_cfg.arr_size = 1;
    esp_periph_handle_t adc_btn_handle = periph_adc_button_init(&adc_btn_cfg);
    AUDIO_NULL_CHECK(TAG, adc_btn_handle, return ESP_ERR_ADF_MEMORY_LACK);
    return esp_periph_start(set, adc_btn_handle);

}

audio_board_handle_t audio_board_get_handle(void)
{
    return board_handle;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board)
{
    esp_err_t ret = ESP_OK;
    ret |= audio_hal_deinit(audio_board->audio_hal);
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}

esp_err_t audio_board_sdcard_init(esp_periph_set_handle_t set, periph_sdcard_mode_t mode)
{
    ESP_LOGI(TAG, "no sdcard support");
    return ESP_FAIL;
}

