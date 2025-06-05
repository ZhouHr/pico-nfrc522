#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "MFRC522.h"

using namespace RFID;

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
    rfid.EnableDebug(true); // 启用调试输出

    printf("MFRC522 initialized with C++ driver.\n");

    uint8_t data_buffer[MFRC522::CARD_BLOCK_DATA_LEN];

    while (true)
    {
        // 尝试读取卡片信息
        auto cardInfo = rfid.ReadCardInfo();
        if (cardInfo)
        {
            printf("检测到卡片!\n");

            // 显示卡片信息
            rfid.DumpCardInfo(*cardInfo);

            // 根据卡片类型执行不同操作
            switch (cardInfo->type)
            {
            case MFRC522::PICC_Type::MIFARE_1K:
            case MFRC522::PICC_Type::MIFARE_4K:
                printf("读取Mifare Classic块 4...\n");
                if (rfid.ReadBlock(4, data_buffer) == MFRC522::StatusCode::MI_OK)
                {
                    // 显示数据
                }
                break;

            case MFRC522::PICC_Type::MIFARE_UL:
            case MFRC522::PICC_Type::NTAG21X:
                printf("读取NTAG/Ultralight页 4...\n");
                if (rfid.ReadNTAG(4, data_buffer) == MFRC522::StatusCode::MI_OK)
                {
                    // 显示数据
                }
                break;

            case MFRC522::PICC_Type::MIFARE_DESFIRE:
                printf("检测到DESFire卡\n");
                // 使用ISO14443-4协议读取
                break;

            default:
                printf("未知卡片类型，尝试通用读取...\n");
                rfid.ReadBlock(4, data_buffer);
                break;
            }

            rfid.HaltTag(); // 使卡片进入休眠状态
        }

        sleep_ms(5 * 1000); // Wait before next attempt
    }
}
