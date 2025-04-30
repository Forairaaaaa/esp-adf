// #include <math.h>
// // #include "i2c_bus.h"
// #include "driver/i2c_master.h"
// #include "board.h"
// #include "esp_log.h"

// #define TAG "stm32"

// /* ---------------------------------- STM32 --------------------------------- */
// #define TOUCH_BUTTON_STATUS_REG_ADDR (0x00)
// #define RGB1_BRIGHTNESS_REG_ADDR     (0x10)
// #define RGB2_BRIGHTNESS_REG_ADDR     (0x11)
// #define RGB1_SHOW_MODE_REG_ADDR      (0x20)
// #define RGB2_SHOW_MODE_REG_ADDR      (0x21)
// #define RGB1_STATUS_REG_ADDR         (0x30)
// #define RGB2_STATUS_REG_ADDR         (0x70)
// #define SPK_RESTART_REG_ADDR         (0xB0)
// #define FLASH_WRITE_BACK_REG_ADDR    (0xF0)
// #define IAP_UPDATE_REG_ADDR          (0xFD)
// #define SW_VER_REG_ADDR              (0xFE)
// #define I2C_ADDR_REG_ADDR            (0xFF)


// // static i2c_bus_handle_t i2c_handle_stm32;
// static i2c_master_dev_handle_t baord_stm32_handle;

// void audio_board_stm32_init()
// {
//     ESP_LOGI(TAG, "Init STM32");

//     i2c_device_config_t stm32_config = {};
//     stm32_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
//     stm32_config.device_address = 0x1A;
//     stm32_config.scl_speed_hz = 400000;
//     i2c_master_bus_add_device(board_i2c_bus_handle, &stm32_config, &baord_stm32_handle);

//     // esp_err_t ret = ESP_OK;
//     // i2c_config_t config = {
//     //     .mode = I2C_MODE_MASTER,
//     //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
//     //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
//     //     .master.clk_speed = 100000
//     // };
//     // ret |= get_i2c_pins(I2C_NUM_0, &config);
//     // i2c_handle_stm32 = i2c_bus_create(I2C_NUM_0, &config);
//     // if (ret != ESP_OK) {
//     //     ESP_LOGE(TAG, "Fail to init stm32");
//     // }
// }

// static char stm32_read_reg(uint8_t reg_addr)
// {
//     uint8_t data;
//     i2c_bus_read_bytes(i2c_handle_stm32, 0x1A, &reg_addr, sizeof(reg_addr), &data, sizeof(data));
//     return data;
// }

// void audio_board_stm32_set_rgb_color(uint8_t channel, uint8_t item, uint32_t color)
// {
//     if (channel == 0) {
//         i2c_dev_write_regs(baord_stm32_handle, RGB1_STATUS_REG_ADDR + item * 4, (uint8_t*)&color, sizeof(color));
//     } else {
//         i2c_dev_write_regs(baord_stm32_handle, RGB2_STATUS_REG_ADDR + item * 4, (uint8_t*)&color, sizeof(color));
//     }

//     if (channel == 0) {
//         uint8_t dev_addr = 0x1A;
//         uint8_t reg_addr = RGB1_STATUS_REG_ADDR + item * 4;
//         i2c_bus_write_bytes(i2c_handle_stm32, dev_addr, &reg_addr, 1, (uint8_t*)&color, 4);
//     } else {
//         uint8_t dev_addr = 0x1A;
//         uint8_t reg_addr = RGB2_STATUS_REG_ADDR + item * 4;
//         i2c_bus_write_bytes(i2c_handle_stm32, dev_addr, &reg_addr, 1, (uint8_t*)&color, 4);
//     }

//     // uint8_t data = stm32_read_reg(SW_VER_REG_ADDR);
//     // ESP_LOGI(TAG, "SW_VER_REG_ADDR: %02X", data);

//     // if (channel == 0) {
//     //     uint8_t dev_addr = 0x1A;
//     //     uint8_t reg_addr = RGB1_STATUS_REG_ADDR + item * 4;
//     //     i2c_bus_write_bytes(i2c_handle_stm32, dev_addr, &reg_addr, 1, (uint8_t*)&color, 4);
//     // } else {
//     //     uint8_t dev_addr = 0x1A;
//     //     uint8_t reg_addr = RGB2_STATUS_REG_ADDR + item * 4;
//     //     i2c_bus_write_bytes(i2c_handle_stm32, dev_addr, &reg_addr, 1, (uint8_t*)&color, 4);
//     // }
// }

// uint32_t _color_list[] = {
//     0xADD8E6, // 冰蓝
//     0xE6E6FA, // 薰衣草紫
//     0x98FF98, // 薄荷绿
//     0xFFB6C1, // 樱花粉
//     0xFF7F50, // 珊瑚橙
//     0x6495ED, // 珊瑚蓝
//     0xFFD700, // 金黄
//     0x2F4F4F  // 深空灰
// };

// // 伽马缩放 RGB888（γ校正，默认γ=2.2）
// uint32_t scale_color_gamma(uint32_t color, float scale) {
//     if (scale < 0.0f) scale = 0.0f;
//     if (scale > 1.0f) scale = 1.0f;

//     const float gamma = 2.2f;

//     // 提取 R、G、B
//     uint8_t r = (color >> 16) & 0xFF;
//     uint8_t g = (color >> 8)  & 0xFF;
//     uint8_t b =  color        & 0xFF;

//     // sRGB to linear
//     float rf = powf(r / 255.0f, gamma);
//     float gf = powf(g / 255.0f, gamma);
//     float bf = powf(b / 255.0f, gamma);

//     // 缩放亮度
//     rf *= scale;
//     gf *= scale;
//     bf *= scale;

//     // linear to sRGB
//     r = (uint8_t)(powf(rf, 1.0f / gamma) * 255.0f + 0.5f);
//     g = (uint8_t)(powf(gf, 1.0f / gamma) * 255.0f + 0.5f);
//     b = (uint8_t)(powf(bf, 1.0f / gamma) * 255.0f + 0.5f);

//     return (r << 16) | (g << 8) | b;
// }

// static void set_all_rgb_color(uint32_t color)
// {
//     uint32_t color_scaled = scale_color_gamma(color, 0.3f);

//     ESP_LOGI(TAG, "set_all_rgb_color: %06lX\n", color_scaled);
//     for (int i = 0; i < 14; i++) {
//         audio_board_stm32_set_rgb_color(0, i, color_scaled);
//         audio_board_stm32_set_rgb_color(1, i, color_scaled);
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// static void led_task(void* param)
// {
//     // vTaskDelay(pdMS_TO_TICKS(4000));

//     // audio_board_stm32_init();

//     // const gpio_num_t btn_io = GPIO_NUM_39;
//     // gpio_reset_pin(btn_io);
//     // gpio_set_direction(btn_io, GPIO_MODE_INPUT);
//     // gpio_pullup_en(btn_io);

//     // int color_index = 0;

//     // set_all_rgb_color(_color_list[color_index]);

//     // while (1) {
//     //     if (gpio_get_level(btn_io) == 0) {
//     //         vTaskDelay(pdMS_TO_TICKS(20));
//     //         if (gpio_get_level(btn_io) == 0) {
//     //             while (gpio_get_level(btn_io) == 0) {
//     //                 vTaskDelay(pdMS_TO_TICKS(10));
//     //             }

//     //             ESP_LOGI(TAG, "Button clicked");

//     //             color_index++;
//     //             if (color_index >= sizeof(_color_list) / sizeof(_color_list[0])) {
//     //                 color_index = 0;
//     //             }

//     //             set_all_rgb_color(_color_list[color_index]);
//     //         }
//     //     } 
//     //     vTaskDelay(pdMS_TO_TICKS(50));
//     // }
//     // vTaskDelete(NULL);
// }

// void stm32_init()
// {
//     // xTaskCreate(led_task, "led", 4000, NULL, 10, NULL);

//     audio_board_stm32_init();
//     set_all_rgb_color(_color_list[0]);
// }
