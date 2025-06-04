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
    MFRC522 rfid(custom_config);

    printf("MFRC522 initialized with C++ driver.\n");

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
    // MFRC522_Reset();

    // printf("MFRC522 initialized.\n");

    uint8_t card_data_buffer[MFRC522::CARD_BLOCK_DATA_LEN];
    uint8_t card_uid_buffer[4]; // For 4-byte UID

    while (true)
    {
        // printf("Reading RFID card data...\n");
        // RFID_Read_Data(3); // 示例：读取扇区 3 的数据
        // sleep_ms(1000);

        printf("Attempting to read RFID card: UID and data from block 3...\n");

        MFRC522::StatusCode status = rfid.ReadCardUIDAndData(3, card_data_buffer, card_uid_buffer);

        if (status == MFRC522::StatusCode::MI_OK)
        {
            printf("Card UID: ");
            for (int i = 0; i < 4; i++)
            {
                printf("%02X ", card_uid_buffer[i]);
            }
            printf("\n");

            printf("Data from block 3: ");
            for (int i = 0; i < MFRC522::CARD_BLOCK_DATA_LEN; i++)
            {
                printf("%02X ", card_data_buffer[i]);
            }
            printf("\nOperation read successful!\n");

            rfid.HaltTag(); // Halt the card after successful operation to allow other cards to be found
        }
        else if (status == MFRC522::StatusCode::MI_NOTAGERR)
        {
            // This is a common occurrence, just means no card was presented or a timeout.
            // printf("No card found or communication timeout.\n");
        }
        else
        {
            // Other errors
            printf("Failed to read card. Status code: %u\n", static_cast<unsigned int>(status));
            // Consider resetting the MFRC522 for the next attempt on persistent errors
            // rfid.SoftwareReset();
        }

        sleep_ms(1000); // Wait before next attempt
    }
}
