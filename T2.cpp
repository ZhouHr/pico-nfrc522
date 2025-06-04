#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "MFRC522.h"

// 可选：如果需要自定义引脚或波特率
#define USE_CUSTOM_CONFIG 0 // 1: 自定义配置, 0: 使用默认配置

int main()
{
    stdio_init_all();
    sleep_ms(10 * 1000);

    printf("RFID RC522 test start!\r\n");

    // 自定义配置示例
    MFRC522_Config custom_config = {
        .spi = spi0,
        .pin_miso = 16,
        .pin_cs = 17,
        .pin_sck = 18,
        .pin_mosi = 19,
        .pin_rst = 20,               // 自定义 RST 引脚
        .baud_rate = 2 * 1000 * 1000 // 提高到 2MHz
    };
    MFRC522_Init(&custom_config);

    // #if USE_CUSTOM_CONFIG
    //     // 自定义配置示例
    //     MFRC522_Config custom_config = {
    //         .spi = spi0,
    //         .pin_miso = 16,
    //         .pin_cs = 17,
    //         .pin_sck = 18,
    //         .pin_mosi = 19,
    //         .pin_rst = 20,               // 自定义 RST 引脚
    //         .baud_rate = 2 * 1000 * 1000 // 提高到 2MHz
    //     };
    //     MFRC522_Init(&custom_config);
    // #else
    //     // 使用默认配置 (spi0 + 默认引脚)
    //     MFRC522_DefaultInit();
    // #endif

    // 可选复位
    MFRC522_Reset();

    printf("MFRC522 initialized.\n");

    while (1)
    {
        printf("Reading RFID card data...\n");
        RFID_Read_Data(3); // 示例：读取扇区 3 的数据
        sleep_ms(1000);
    }
}
