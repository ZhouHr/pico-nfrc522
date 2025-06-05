#include "MFRC522.h"
#include "string.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <stdio.h>

namespace RFID
{

    constexpr uint8_t MAX_COMM_BUFFER_LEN = 18; // 命令和响应的缓冲区（例如16数据+2 CRC）
    const uint8_t MFRC522::DefaultKeyA[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // 自定义配置的构造函数
    MFRC522::MFRC522(const MFRC522_Config &config)
        : m_spi(config.spi),
          m_pin_miso(config.pin_miso),
          m_pin_cs(config.pin_cs),
          m_pin_sck(config.pin_sck),
          m_pin_mosi(config.pin_mosi),
          m_pin_rst(config.pin_rst),
          m_baud_rate(config.baud_rate)
    {
        Init();
        Reset();
        PcdReset();
    }

    // 默认构造函数
    MFRC522::MFRC522()
        : m_spi(spi_default), // 使用Pico SDK的默认SPI实例（通常是spi0）
          m_pin_miso(DEFAULT_PIN_MISO),
          m_pin_cs(DEFAULT_PIN_CS),
          m_pin_sck(DEFAULT_PIN_SCK),
          m_pin_mosi(DEFAULT_PIN_MOSI),
          m_pin_rst(DEFAULT_PIN_RST),
          m_baud_rate(DEFAULT_BAUDRATE)
    {
        Init();
        Reset();
        PcdReset();
    }

    void MFRC522::Init()
    {
        gpio_init(m_pin_rst);
        gpio_set_dir(m_pin_rst, GPIO_OUT);
        gpio_put(m_pin_rst, 1);

        gpio_set_function(m_pin_miso, GPIO_FUNC_SPI);
        gpio_set_function(m_pin_sck, GPIO_FUNC_SPI);
        gpio_set_function(m_pin_mosi, GPIO_FUNC_SPI);

        gpio_init(m_pin_cs);
        gpio_set_dir(m_pin_cs, GPIO_OUT);
        gpio_put(m_pin_cs, 1);

        // 只有在未初始化的情况下才初始化 SPI
        if (!m_spi_initialized)
        {
            spi_init(m_spi, m_baud_rate);
            spi_set_format(m_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
            m_spi_initialized = true;
        }
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：Reset
    // 功    能：通过 RST 引脚复位 MFRC522 模块
    /////////////////////////////////////////////////////////////////////
    void MFRC522::Reset()
    {
        gpio_put(m_pin_rst, GPIO_PIN_RESET); // 拉低 RST
        sleep_ms(50);                        // 保持复位低电平一段时间
        gpio_put(m_pin_rst, GPIO_PIN_SET);   // 拉高，释放复位
        sleep_ms(50);                        // 等待芯片稳定
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：PcdReset
    // 功    能：复位RC522
    // 返 回 值：成功，返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::PcdReset()
    {
        Reset(); // 使用硬件复位
        WriteRegister(PCD_Register::CommandReg, static_cast<uint8_t>(PCD_Command::RESET_PHASE));

        // 通过读取VersionReg来检查复位是否成功
        uint8_t version = ReadRegister(PCD_Register::VersionReg);
        if (version == 0x00 || version == 0xFF)
        {
            // 如果读取到0x00或0xFF,说明芯片未正确响应,复位失败
            return StatusCode::MI_RESET_FAIL;
        }

        // 检查PowerDown位是否已清除
        if (ReadRegister(PCD_Register::CommandReg) & (1 << 4))
        {
            // PowerDown位未清除,说明芯片仍处于掉电状态
            return StatusCode::MI_POWERDOWN_ERR;
        }

        sleep_ms(10);
        WriteRegister(PCD_Register::ModeReg, 0x3d);
        sleep_ms(10);
        WriteRegister(PCD_Register::TReloadRegL, 30);
        WriteRegister(PCD_Register::TReloadRegH, 0);
        WriteRegister(PCD_Register::TModeReg, 0x8d);
        WriteRegister(PCD_Register::TPrescalerReg, 0x3e);
        WriteRegister(PCD_Register::TxAskReg, 0x40);

        return StatusCode::MI_OK;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：WriteRegister
    // 功    能：向RC522寄存器中写入值
    // 参数说明：addr 寄存器地址
    //			val  要写入的值
    /////////////////////////////////////////////////////////////////////
    void MFRC522::WriteRegister(PCD_Register reg, uint8_t val)
    {
        // unsigned char i, ucaddr;
        gpio_put(m_pin_sck, GPIO_PIN_RESET); // 将CLK拉低
        gpio_put(m_pin_cs, GPIO_PIN_RESET);  // 将CSS拉低

        uint8_t addr_byte = (static_cast<uint8_t>(reg) << 1) & 0x7E; // Bit 0 is 0 for write. MSB is 0.
        spi_write_blocking(m_spi, &addr_byte, 1);
        spi_write_blocking(m_spi, &val, 1);

        gpio_put(m_pin_sck, GPIO_PIN_SET); // 将CLK拉高
        gpio_put(m_pin_cs, GPIO_PIN_SET);  // 将CSS拉高
        sleep_ms(10);
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ReadRegister
    // 功    能：读RC522寄存器中的值
    // 参数说明：addr 寄存器地址
    /////////////////////////////////////////////////////////////////////
    uint8_t MFRC522::ReadRegister(PCD_Register reg)
    {
        gpio_put(m_pin_sck, GPIO_PIN_RESET); // 将CLK拉低
        gpio_put(m_pin_cs, GPIO_PIN_RESET);  // 将CSS拉低
        /*地址处理：bit7读写位(1读/0写)，bit6~bit1地址，bit0必须为0*/
        // ucaddr = ((addr << 1) & 0x7e) | 0x80;
        uint8_t addr_byte = (static_cast<uint8_t>(reg) << 1) | 0x80; // Bit 0 is 0. MSB is 1 for read.
        uint8_t rx_data;

        spi_write_blocking(m_spi, &addr_byte, 1);
        spi_read_blocking(m_spi, 0x00, &rx_data, 1); // Send dummy byte 0x00 to read

        gpio_put(m_pin_sck, GPIO_PIN_SET); // 将CLK拉高
        gpio_put(m_pin_cs, GPIO_PIN_SET);  // 将CSS拉高
        sleep_ms(10);
        return rx_data;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ClearRegisterBitMask
    // 功    能：清RC522寄存器位
    // 参数说明：reg[IN]  寄存器地址
    //           mask[IN] 清位值
    /////////////////////////////////////////////////////////////////////
    void MFRC522::ClearRegisterBitMask(PCD_Register reg, uint8_t mask)
    {
        char temp = 0x0;
        temp = ReadRegister(reg);         // 读Reg寄存器的状态
        WriteRegister(reg, temp & ~mask); // 向Reg寄存器写入tmp&~mask位
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：SetRegisterBitMask
    // 功    能：置RC522寄存器位
    // 参数说明：reg[IN]  寄存器地址
    //           mask[IN] 置位值
    /////////////////////////////////////////////////////////////////////
    void MFRC522::SetRegisterBitMask(PCD_Register reg, uint8_t mask)
    {
        char temp = 0x0;
        temp = ReadRegister(reg);        // 读Reg寄存器的状态
        WriteRegister(reg, temp | mask); // 向Reg寄存器写入 MASK|tmp位
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：AntennaOn
    // 功    能：开启天线发射
    // 说    明：每次启动或关闭天险发射之间应至少有1ms的间隔
    /////////////////////////////////////////////////////////////////////
    void MFRC522::AntennaOn()
    {
        uint8_t val = ReadRegister(PCD_Register::TxControlReg);
        if ((val & 0x03) != 0x03)
        {
            SetRegisterBitMask(PCD_Register::TxControlReg, 0x03);
        }
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：AntennaOff
    // 功    能：开启天线发射
    // 说    明：每次启动或关闭天险发射之间应至少有1ms的间隔
    /////////////////////////////////////////////////////////////////////
    void MFRC522::AntennaOff(void)
    {
        ClearRegisterBitMask(PCD_Register::TxControlReg, 0x03);
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：CommunicateWithTag
    // 功    能：通过RC522和ISO14443卡通讯
    // 参数说明：Command[IN]:     RC522命令字
    //           pInData[IN]:     通过RC522发送到卡片的数据
    //           InLenByte[IN]:   发送数据的字节长度
    //           pOutData[OUT]:   接收到的卡片返回数据
    //           *pOutLenBit[OUT]:返回数据的位长度
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::CommunicateWithTag(PCD_Command Command, const uint8_t *pInData,
                                                    uint8_t InLenByte, uint8_t *pOutData, uint32_t *pOutLenBit)
    {
        StatusCode status = StatusCode::MI_ERR;
        uint8_t irqEn = 0x00;
        uint8_t waitFor = 0x00;
        uint8_t lastBits;
        uint8_t n;
        int i;
        switch (Command)
        {
        case PCD_Command::AUTHENT: // 0x0E  验证密钥
            irqEn = 0x12;          // 0001 0010
            waitFor = 0x10;        // 0001 0000
            break;
        case PCD_Command::TRANSCEIVE: // 0x0C  发送并接收数据
            irqEn = 0x77;             // 0111 0111
            waitFor = 0x30;           // 0011 0000
            break;
        default:
            return StatusCode::MI_ERR;
            break;
        }

        WriteRegister(PCD_Register::ComIEnReg, irqEn | 0x80);                             // 使能相应中断
        ClearRegisterBitMask(PCD_Register::ComIrqReg, 0x80);                              // 清零所有中断标志
        WriteRegister(PCD_Register::CommandReg, static_cast<uint8_t>(PCD_Command::IDLE)); // 命令寄存器复零
        SetRegisterBitMask(PCD_Register::FIFOLevelReg, 0x80);                             // 复位FIFO读写指针

        /*把要发送给卡的数据写入522的FIFO数据寄存器*/
        for (i = 0; i < InLenByte; i++)
        {
            WriteRegister(PCD_Register::FIFODataReg, pInData[i]);
        }
        /*RC522执行发送命令*/
        WriteRegister(PCD_Register::CommandReg, static_cast<uint8_t>(Command));

        /*若执行的是发送数据命令*/
        if (Command == PCD_Command::TRANSCEIVE)
        {
            SetRegisterBitMask(PCD_Register::BitFramingReg, 0x80); // 启动传输
        }

        // 退出do-while循环的IRQ条件:
        // 1.i=0
        // 2.定时器减到零中断
        // 3.接收器检测到有效的数据中断OR命令本身终止中断
        i = 500; // 根据时钟频率调整，操作M1卡最大等待时间25ms
        do
        {
            n = ReadRegister(PCD_Register::ComIrqReg); // 读中断标志寄存器
            i--;
        } while ((i != 0) && !(n & 0x01) && !(n & waitFor));

        ClearRegisterBitMask(PCD_Register::BitFramingReg, 0x80); // 结束发送数据命令

        if (i != 0) // 射频区无卡
        {
            // 读错误标志寄存器&0001 1011不是以下错误：
            // 数据FIFO已满、碰撞错误、奇偶校验错误、SOF错误
            if (!(ReadRegister(PCD_Register::ErrorReg) & 0x1B))
            {
                status = StatusCode::MI_OK; // 暂时无错误
                if (n & irqEn & 0x01)       // 是否为定时器中断
                {
                    status = StatusCode::MI_NOTAGERR;
                } // 错误

                if (Command == PCD_Command::TRANSCEIVE)
                {
                    n = ReadRegister(PCD_Register::FIFOLevelReg); // 共接收了几个字节
                    // 接收的最后一个字节的有效位
                    lastBits = ReadRegister(PCD_Register::ControlReg) & 0x07;
                    if (lastBits)
                    {
                        *pOutLenBit = (n - 1) * 8 + lastBits;
                    } // N*8+LASTBITS位
                    else
                    {
                        *pOutLenBit = n * 8;
                    } // N*8位

                    if (n == 0)
                    {
                        n = 1;
                    }

                    if (n > MAX_COMM_BUFFER_LEN)
                    {
                        n = MAX_COMM_BUFFER_LEN;
                    } // 最多可接收18个字节
                    for (i = 0; i < n; i++)
                    {
                        pOutData[i] = ReadRegister(PCD_Register::FIFODataReg);
                    } // 取出接收到的数据放到数组POUTDATA
                }
            }
            else
            {
                status = StatusCode::MI_ERR;
            } // 错误标志
        }

        SetRegisterBitMask(PCD_Register::ControlReg, 0x80);                               // 停止定时器
        WriteRegister(PCD_Register::CommandReg, static_cast<uint8_t>(PCD_Command::IDLE)); // 复位PCD_Register::CommandReg
        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：CalulateCRC
    // 功    能：用MF522计算CRC16函数
    // 参数说明:
    /////////////////////////////////////////////////////////////////////
    StatusCode MFRC522::CalulateCRC(const uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
    {
        uint8_t i, n;
        ClearRegisterBitMask(PCD_Register::DivIrqReg, 0x04);                              // 禁止CRC
        WriteRegister(PCD_Register::CommandReg, static_cast<uint8_t>(PCD_Command::IDLE)); // 复位522PCD_Register::CommandReg寄存器
        SetRegisterBitMask(PCD_Register::FIFOLevelReg, 0x80);                             // 复位FIFO的读写指针
        for (i = 0; i < len; i++)
        {
            WriteRegister(PCD_Register::FIFODataReg, *(pIndata + i));
        } // 把*pIndata缓冲区的值写如PCD_Register::FIFODataReg
        WriteRegister(PCD_Register::CommandReg, static_cast<uint8_t>(PCD_Command::CALC_CRC)); // 执行CRC校验
        i = 0xFF;                                                                             // 等待255us
        do
        {
            n = ReadRegister(PCD_Register::DivIrqReg); // 读中断请求标志寄存器
            i--;
        } while ((i != 0) && !(n & 0x04));

        if (i == 0)
        {
            // 如果超时,返回错误
            return StatusCode::MI_TIMEOUT;
        }

        // 读取CRC结果
        pOutData[0] = ReadRegister(PCD_Register::CRCResultRegL); // CRC校验的低8位
        pOutData[1] = ReadRegister(PCD_Register::CRCResultRegM); // CRC校验的高8位

        return StatusCode::MI_OK;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：Request
    // 功    能：寻卡
    // 参数说明: req_code[IN]:寻卡方式
    //                 0x52 = 寻感应区内所有符合14443A标准的卡
    //                 0x26 = 寻未进入休眠状态的卡
    //           pTagType[OUT]：卡片类型代码
    //                 0x4400 = Mifare_UltraLight
    //                 0x0400 = Mifare_One(S50)
    //                 0x0200 = Mifare_One(S70)
    //                 0x0800 = Mifare_Pro(X)
    //                 0x4403 = Mifare_DESFire
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::Request(PICC_Command req_code, uint8_t *pTagType)
    {
        StatusCode status;
        uint32_t unLen;
        uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN];

        ClearRegisterBitMask(PCD_Register::Status2Reg, 0x08); // 清零PCD_Register::Status2Reg的MFAuthent Command执行成功标志位
        WriteRegister(PCD_Register::BitFramingReg, 0x07);     // 清零Transceive命令开始位
        SetRegisterBitMask(PCD_Register::TxControlReg, 0x03); // 开启天线
        ucComMF522Buf[0] = {static_cast<uint8_t>(req_code)};  // 取522要执行的命令
        // 向PICC发送寻天线区内全部卡命令，并接收PICC返回的数据
        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);
        // 如果通信本身就失败了，直接返回CommunicateWithTag的错误码
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        if ((status == StatusCode::MI_OK) && (unLen == 0x10)) // 没有错误并接接收为2个字节
        {
            *pTagType = ucComMF522Buf[0];       // 取接收缓冲区的第一个字节
            *(pTagType + 1) = ucComMF522Buf[1]; // 取接收缓冲区的第二个字节
        }
        else
        {
            return StatusCode::MI_NOTAGERR; // 未寻到卡片
        } // 错误

        WriteRegister(PCD_Register::BitFramingReg, 0x00); // Reset to 0 bits for subsequent commands

        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：Anticollision
    // 功    能：防冲撞
    // 参数说明: pSnr[OUT]:卡片序列号，4字节
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::Anticollision(uint8_t *pSnr, uint8_t cascadeLevel, uint8_t *pSak)
    {
        StatusCode status;
        uint32_t unLen;
        uint8_t i, snr_check = 0;
        uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN];
        PICC_Command antiCollCmd;
        // 根据级联级别选择相应的防冲撞命令
        switch (cascadeLevel)
        {
        case 1:
            antiCollCmd = PICC_Command::ANTICOLL1;
            break;
        case 2:
            antiCollCmd = PICC_Command::ANTICOLL2;
            break;
        case 3:
            antiCollCmd = PICC_Command::ANTICOLL3;
            break;
        default:
            antiCollCmd = PICC_Command::ANTICOLL1;
            break;
        }

        ClearRegisterBitMask(PCD_Register::Status2Reg, 0x08); // 清除标志位
        WriteRegister(PCD_Register::BitFramingReg, 0x00);     // 000 指示最后一个字节的所有位将被发送。
        ClearRegisterBitMask(PCD_Register::CollReg, 0x80);    // 发生碰撞所有接收位将被清除
        ucComMF522Buf[0] = static_cast<uint8_t>(antiCollCmd); // 0x93 防冲撞 发到卡里的命令
        ucComMF522Buf[1] = 0x20;
        // 获得卡的序列号，ucComMF522Buf[]
        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

        // 如果通信本身就失败了，直接返回CommunicateWithTag的错误码
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        if (status == StatusCode::MI_OK)
        {
            // 循环处理4字节的卡序列号
            for (i = 0; i < 4; i++)
            {
                *(pSnr + i) = ucComMF522Buf[i]; // 将接收缓冲区中的卡序列号复制到输出缓冲区
                snr_check ^= ucComMF522Buf[i];  // 使用异或运算计算BCC(Block Check Character)校验码
                                                // BCC校验码是所有UID字节的异或结果
            }
            if (snr_check != ucComMF522Buf[i])
            {
                status = StatusCode::MI_BBCERR;
            } // 有错误
            // 如果需要获取SAK值，调用SelectTag
            if (pSak != nullptr && status == StatusCode::MI_OK)
            {
                status = SelectTag(pSnr, pSak);
            }
        }

        SetRegisterBitMask(PCD_Register::CollReg, 0x80); // 置位防碰撞位
        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：SelectTag
    // 功    能：选定卡片
    // 参数说明: pSerialNumber[IN]:卡片序列号，4字节
    //           pSakBuffer[OUT]:存储SAK值
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::SelectTag(const uint8_t *pSerialNumber, uint8_t *pSakBuffer)
    {
        StatusCode status = StatusCode::MI_NOTAGERR;
        uint32_t received_bits;
        uint8_t i;
        uint8_t command_data[MAX_COMM_BUFFER_LEN]; // MAX_COMM_BUFFER_LEN = 18

        command_data[0] = static_cast<uint8_t>(PICC_Command::ANTICOLL1); // 防冲撞命令
        command_data[1] = 0x70;
        command_data[6] = 0;
        for (i = 0; i < 4; i++)
        {
            command_data[i + 2] = *(pSerialNumber + i); // 填充卡的序列号
            command_data[6] ^= *(pSerialNumber + i);    // 计算校验码
        }
        // memcpy(&command_data[2], pSerialNumber, 5);     // UID(4) + BCC(1)
        status = CalulateCRC(command_data, 7, &command_data[7]); // 获得CRC校验结果的16位值
                                                                 // 放入command_data【0，1】

        if (m_debug_enabled)
        {
            printf("SelectTag: Sending command_data (9 bytes): ");
            for (int i = 0; i < 7; ++i)
            {
                printf("%02X ", command_data[i]);
            }
            printf("\n");
        }

        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        ClearRegisterBitMask(PCD_Register::Status2Reg, 0x08); // 清零MFAuthent Command执行成功标志位
        // 把CRC值和卡号发的卡里
        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, command_data, 9, command_data, &received_bits);
        if (m_debug_enabled)
        {
            printf("SelectTag CommunicateWithTag. Status code: %u\n", static_cast<unsigned int>(status));
            printf("SelectTag: SAK = 0x%02X\n", command_data[0]);
        }
        // 如果通信本身就失败了，直接返回CommunicateWithTag的错误码
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        if (status == StatusCode::MI_OK && received_bits > 0 && pSakBuffer)
        {
            unsigned int len_bytes = received_bits / 8;
            if (received_bits % 8 != 0)
                len_bytes++; // account for partial bytes if any
            if (len_bytes > MAX_COMM_BUFFER_LEN)
                len_bytes = MAX_COMM_BUFFER_LEN;

            if (m_debug_enabled)
            {
                printf("SelectTag: Response buffer (first %u bytes): ", len_bytes);
                for (unsigned int i = 0; i < len_bytes; ++i)
                {
                    printf("%02X ", command_data[i]);
                }
                printf("\n");
            }
        }
        if ((status == StatusCode::MI_OK) && (received_bits > 0)) // 返回24个字节&状态为无错误 SAK (1 byte) + CRC_A (2 bytes) = 24 bits
        {
            if (pSakBuffer != nullptr)
            {
                *pSakBuffer = command_data[0]; // 存储SAK值
            }
            if (command_data[0] & 0x04)
            { // UID not complete, further cascading needed
                return StatusCode::MI_ERR;
            }
            // status = StatusCode::MI_OK;
        }
        else
        {
            status = StatusCode::MI_ERR;
        }

        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：Authenticate
    // 功    能：验证卡片密码
    // 参数说明: auth_mode[IN]: 密码验证模式
    //                  		   0x60 = 验证A密钥
    //                  		   0x61 = 验证B密钥
    //           addr[IN]：块地址
    //           pKey[IN]：密码
    //           pSnr[IN]：卡片序列号，4字节
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::Authenticate(PICC_Command auth_mode, uint8_t addr, const uint8_t *pKey, const uint8_t *pSnr)
    {
        StatusCode status;
        uint32_t unLen;
        uint8_t i, command_data[MAX_COMM_BUFFER_LEN];

        command_data[0] = static_cast<uint8_t>(auth_mode); // PICC验证A密钥指令
        command_data[1] = addr;                            // 块地址
        for (i = 0; i < 6; i++)
        {
            command_data[i + 2] = *(pKey + i);
        } // 向缓冲区填充密码
        for (i = 0; i < 6; i++)
        {
            command_data[i + 8] = *(pSnr + i);
        } // 向缓冲区填充与密码对应的卡的序列号，有效4个字节
        // memcpy(&command_data[2], pKey, 6);
        // memcpy(&command_data[8], pSnr, 4); // Use first 4 bytes of UID

        // status = CommunicateWithTag(PCD_Command::AUTHENT, command_data, 12, command_data, &unLen); // 验证密码和卡号
        status = CommunicateWithTag(PCD_Command::AUTHENT, command_data, 12, nullptr, &unLen); // 验证密码和卡号
        if (m_debug_enabled)
        {
            printf("Authenticate CommunicateWithTag read card. Status code: %u\n", static_cast<unsigned int>(status));
        }

        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        if (!(ReadRegister(PCD_Register::Status2Reg) & 0x08)) // 密码验证是否成功
        {
            status = StatusCode::MI_AUTHERR;
        }

        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ReadBlock
    // 功    能：读取M1卡一块数据
    // 参数说明: addr[IN]：块地址
    //           pData[OUT]：读出的数据，16字节
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::ReadBlock(uint8_t addr, uint8_t *pData)
    {
        StatusCode status;
        uint32_t unLen;
        uint8_t i, ucComMF522Buf[MAX_COMM_BUFFER_LEN];

        ucComMF522Buf[0] = static_cast<uint8_t>(PICC_Command::READ);
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        if ((status == StatusCode::MI_OK) && (unLen == 0x90))
        {
            for (i = 0; i < 16; i++)
            {
                *(pData + i) = ucComMF522Buf[i];
            }
        }
        else
        {
            status = StatusCode::MI_ERR;
        }

        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：WriteBlock
    // 功    能：写数据到M1卡一块
    // 参数说明: addr[IN]：块地址
    //           pData[IN]：写入的数据，16字节
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::WriteBlock(uint8_t addr, const uint8_t *pData)
    {
        StatusCode status;
        uint32_t unLen;
        uint8_t i, ucComMF522Buf[MAX_COMM_BUFFER_LEN];

        ucComMF522Buf[0] = static_cast<uint8_t>(PICC_Command::WRITE); // PICC_Command::WRITE  0xA0   写块
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]); // 对数据进行CRC校验
                                                          // 存放于ucCoMF522Buf【0，1】
        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        if ((unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            status = StatusCode::MI_ERR;
        }

        if (status == StatusCode::MI_OK)
        {
            for (i = 0; i < 16; i++)
            {
                ucComMF522Buf[i] = *(pData + i);
            } // 要充值的内容
            CalulateCRC(ucComMF522Buf, 16, &ucComMF522Buf[16]);                                             // 对数据进行CRC校验校验值
                                                                                                            // 存放于ucCoMF522Buf【0，1】
            status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf, &unLen); // 发送数据，并接收卡返回的数据

            if (status != StatusCode::MI_OK)
            {
                return status;
            }

            if ((unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
            {
                status = StatusCode::MI_ERR;
            }
        }

        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：HaltTag
    // 功    能：命令卡片进入休眠状态
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::HaltTag()
    {
        uint32_t unLen;
        uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN];

        ucComMF522Buf[0] = static_cast<uint8_t>(PICC_Command::HALT);
        ucComMF522Buf[1] = 0;
        CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

        StatusCode status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
        if (status == StatusCode::MI_NOTAGERR)
        {
            return StatusCode::MI_OK;
        }
        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ReadCompleteUID
    // 功    能：读取完整的多级UID
    // 参数说明: pUidBuffer[OUT]: 存储UID的缓冲区，至少10字节
    //           pUidLength[OUT]: 返回UID的实际长度
    //           pSak[OUT]: 可选，存储最后一级的SAK值
    //           pAtqa[OUT]: 可选，存储ATQA值
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::ReadCompleteUID(uint8_t *pUidBuffer, uint8_t *pUidLength, uint8_t *pSak, uint8_t *pAtqa)
    {
        StatusCode status;
        uint8_t currentUid[5]; // 4字节UID + 1字节BCC
        uint8_t currentSak = 0;
        uint8_t cascadeLevel = 1;
        uint8_t uidIndex = 0;
        bool uidComplete = false;

        // 初始化UID长度为0
        *pUidLength = 0;

        // 寻卡
        uint8_t tag_type_buffer[2];
        status = Request(PICC_Command::REQALL, tag_type_buffer);
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        // 保存ATQA值
        if (pAtqa != nullptr)
        {
            pAtqa[0] = tag_type_buffer[0];
            pAtqa[1] = tag_type_buffer[1];
        }

        // 循环处理多级UID
        while (!uidComplete && cascadeLevel <= 3)
        {
            // 防冲突
            status = Anticollision(currentUid, cascadeLevel);
            if (status != StatusCode::MI_OK)
            {
                return status;
            }

            // 选卡并获取SAK
            status = SelectTag(currentUid, &currentSak);
            if (status != StatusCode::MI_OK)
            {
                return status;
            }

            // 检查SAK的bit2，判断UID是否完整
            // 如果bit2为0，表示UID已完整；如果为1，表示需要继续级联
            if ((currentSak & 0x04) == 0)
            {
                uidComplete = true;
            }

            // 复制UID到输出缓冲区
            // 对于级联的情况，第一个字节是级联标记(0x88)，不是UID的一部分
            if (cascadeLevel == 1 || (currentUid[0] != 0x88))
            {
                // 普通情况，复制全部4字节
                for (int i = 0; i < 4; i++)
                {
                    pUidBuffer[uidIndex++] = currentUid[i];
                }
                *pUidLength += 4;
            }
            else
            {
                // 级联情况，跳过第一个字节
                for (int i = 1; i < 4; i++)
                {
                    pUidBuffer[uidIndex++] = currentUid[i];
                }
                *pUidLength += 3;
            }

            // 增加级联级别
            cascadeLevel++;
        }

        // 如果需要返回SAK值
        if (pSak != nullptr)
        {
            *pSak = currentSak;
        }

        return StatusCode::MI_OK;
    }

    MFRC522::StatusCode MFRC522::ReadCardUIDAndData(uint8_t blockAddr, uint8_t *pDataBuffer16Bytes, uint8_t *pUidBuffer4Bytes)
    {
        StatusCode status;
        uint8_t tag_type_buffer[2];
        uint8_t serial_buffer_with_bcc[5]; // 4 UID + 1 BCC

        status = Request(PICC_Command::REQALL, tag_type_buffer);
        if (m_debug_enabled)
        {
            printf("Request card. Status code: %u\n", static_cast<unsigned int>(status));
        }
        if (status != StatusCode::MI_OK)
            return status;

        status = Anticollision(serial_buffer_with_bcc);
        if (m_debug_enabled)
        {
            printf("Anticollision card. Status code: %u\n", static_cast<unsigned int>(status));
        }
        if (status != StatusCode::MI_OK)
            return status;
        memcpy(pUidBuffer4Bytes, serial_buffer_with_bcc, 4); // Copy only the 4 UID bytes
        if (m_debug_enabled)
        {
            printf("ReadCardUIDAndData UID: ");
            for (int i = 0; i < 4; i++)
            {
                printf("%02X ", pUidBuffer4Bytes[i]);
            }
            printf("\n");
        }

        status = SelectTag(serial_buffer_with_bcc); // Select using full UID + BCC
        if (m_debug_enabled)
        {
            printf("SelectTag card. Status code: %u\n", static_cast<unsigned int>(status));
        }
        if (status != StatusCode::MI_OK)
            return status;

        status = Authenticate(PICC_Command::AUTHENT1A, blockAddr, DefaultKeyA, pUidBuffer4Bytes);
        if (m_debug_enabled)
        {
            printf("Authenticate card. Status code: %u\n", static_cast<unsigned int>(status));
        }
        if (status != StatusCode::MI_OK)
            return status;

        status = ReadBlock(blockAddr, pDataBuffer16Bytes);
        if (m_debug_enabled)
        {
            printf("ReadBlock card. Status code: %u\n", static_cast<unsigned int>(status));
        }
        if (status != StatusCode::MI_OK)
            return status;

        // Optionally, halt the tag after operation
        // HaltTag();

        return StatusCode::MI_OK;
    }

    // 实现新的 ReadIso14443aData 方法
    MFRC522::StatusCode MFRC522::ReadIso14443aData(uint8_t pageAddr, uint8_t *pDataBuffer16Bytes, uint8_t *pUidBuffer, uint8_t *pUidLength)
    {
        StatusCode status;
        uint8_t tag_type_buffer[2];
        uint8_t uid_length = 0;
        uint8_t sak_value = 0xFF; // 初始化为一个无效的SAK值
        uint8_t atqa[2] = {0};

        // 1. 寻卡 (Request)
        // status = Request(PICC_Command::REQALL, tag_type_buffer);
        // if (m_debug_enabled)
        // {
        //     printf("Request card. Status code: %u\n", static_cast<unsigned int>(status));
        // }
        // if (status != StatusCode::MI_OK)
        // {
        //     // printf("ReadIso14443aData: Request failed (%u)\n", static_cast<unsigned int>(status));
        //     return status;
        // }

        // 1. 读取完整UID和SAK
        status = ReadCompleteUID(pUidBuffer, &uid_length, &sak_value, atqa);
        if (m_debug_enabled)
        {
            printf("ReadCompleteUID. Status code: %u, UID length: %u\n", static_cast<unsigned int>(status), uid_length);
        }
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        // 如果调用者提供了pUidLength指针，返回实际UID长度
        if (pUidLength != nullptr)
        {
            *pUidLength = uid_length;
        }

        // 如果UID长度小于4，用0填充剩余部分
        if (uid_length < 4)
        {
            memset(pUidBuffer + uid_length, 0, 4 - uid_length);
        }

        if (m_debug_enabled)
        {
            printf("ReadIso14443aData: SAK = 0x%02X, UID length = %u\n", sak_value, uid_length);
            printf("Complete UID: ");
            for (int i = 0; i < uid_length; i++)
            {
                printf("%02X ", pUidBuffer[i]);
            }
            printf("\n");
        }

        /*
        // 4. 根据SAK值判断是否需要Mifare认证
        bool needs_authentication = false;
        // SAK for Mifare Classic 1K: 0x08
        // SAK for Mifare Classic 4K: 0x18 (bit 2 for 1k/4k, bit 4 for UID complete)
        // SAK for Mifare Plus, etc. might also indicate Classic compatibility.
        // Common SAK for NTAG/Ultralight (NFC Type 2) is 0x00.
        // NTAGs that support ISO14443-4 might have SAK 0x20.
        // For simplicity, we assume SAK 0x08 and 0x18 require Mifare Auth for block read.
        // Other SAK values (like 0x00 for typical NTAG/Ultralight) will skip auth for ReadBlock.
        if (sak_value == 0x08 || sak_value == 0x18)
        {
            needs_authentication = true;
        }

        if (needs_authentication)
        {
            // printf("ReadIso14443aData: Needs Mifare Authentication for block/page %u.\n", pageAddr);
            status = Authenticate(PICC_Command::AUTHENT1A, pageAddr, DefaultKeyA, pUidBuffer);
            printf("Authenticate card. Status code: %u\n", static_cast<unsigned int>(status));
            if (status != StatusCode::MI_OK)
            {
                // printf("ReadIso14443aData: Authenticate failed (%u)\n", static_cast<unsigned int>(status));
                return status;
            }
        }

        // 5. 读取数据块/页面
        // ReadBlock 方法读取16字节。对于NTAG/Ultralight, pageAddr是起始页地址，它会读取4个连续页。
        status = ReadBlock(pageAddr, pDataBuffer16Bytes);
        printf("ReadBlock card. Status code: %u\n", static_cast<unsigned int>(status));
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        return StatusCode::MI_OK;
        */
        // 2. 识别卡片类型
        PICC_Type cardType = IdentifyCardType(sak_value, atqa);

        // 3. 根据卡片类型选择适当的读取方法
        switch (cardType)
        {
        case PICC_Type::MIFARE_1K:
        case PICC_Type::MIFARE_4K:
            // Mifare Classic需要认证
            status = Authenticate(PICC_Command::AUTHENT1A, pageAddr, DefaultKeyA, pUidBuffer);
            if (m_debug_enabled)
            {
                printf("Authenticate card. Status code: %u\n", static_cast<unsigned int>(status));
            }
            if (status != StatusCode::MI_OK)
            {
                return status;
            }
            return ReadBlock(pageAddr, pDataBuffer16Bytes);

        case PICC_Type::MIFARE_UL:
        case PICC_Type::NTAG21X:
            // Ultralight/NTAG不需要认证，直接读取
            return ReadBlock(pageAddr, pDataBuffer16Bytes);

        case PICC_Type::ISO_14443_4:
        case PICC_Type::MIFARE_DESFIRE:
            // 对于DESFire和其他ISO14443-4卡，需要使用ISO14443-4协议
            // 这里简化处理，尝试直接读取
            return ReadBlock(pageAddr, pDataBuffer16Bytes);

        default:
            // 未知卡片类型，尝试直接读取
            return ReadBlock(pageAddr, pDataBuffer16Bytes);
        }
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：IdentifyCardType
    // 功    能：根据SAK和ATQA值识别卡片类型
    // 参数说明: sak[IN]: SAK值
    //           atqa[IN]: ATQA值，2字节
    // 返    回: 卡片类型
    /////////////////////////////////////////////////////////////////////
    MFRC522::PICC_Type MFRC522::IdentifyCardType(uint8_t sak, const uint8_t *atqa)
    {
        // 检查ISO/IEC 14443-4兼容性
        if (sak & 0x20)
        {
            return PICC_Type::ISO_14443_4;
        }

        // Mifare Classic检测
        if ((sak & 0x1F) == 0x08)
        {
            return PICC_Type::MIFARE_1K;
        }

        if ((sak & 0x1F) == 0x18)
        {
            return PICC_Type::MIFARE_4K;
        }

        // Mifare Ultralight或NTAG检测
        if (sak == 0x00)
        {
            // 检查ATQA以区分Ultralight和NTAG
            if (atqa[0] == 0x44 && atqa[1] == 0x00)
            {
                // 可以进一步区分NTAG类型，但需要读取特定页面
                return PICC_Type::MIFARE_UL;
            }
            // NTAG21x系列通常也有SAK=0x00，但ATQA可能不同
            // 这里简化处理，可以通过读取特定页面进一步区分
            return PICC_Type::NTAG21X;
        }

        // DESFire检测
        if ((sak & 0x1F) == 0x20)
        {
            if (atqa[0] == 0x44 && atqa[1] == 0x03)
            {
                return PICC_Type::MIFARE_DESFIRE;
            }
        }

        return PICC_Type::UNKNOWN;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ReadNTAG
    // 功    能：读取NTAG/Ultralight页面
    // 参数说明: page[IN]: 页地址
    //           data[OUT]: 数据缓冲区
    //           length[IN]: 要读取的字节数，默认为4（一个页面）
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::ReadNTAG(uint8_t page, uint8_t *data, uint8_t length)
    {
        // NTAG/Ultralight每页4字节，使用ReadBlock读取
        // ReadBlock实际上会读取16字节（4页），我们只取需要的部分
        uint8_t buffer[16];
        StatusCode status = ReadBlock(page, buffer);
        if (status != StatusCode::MI_OK)
        {
            return status;
        }

        // 只复制需要的字节数，最多16字节
        uint8_t bytesToCopy = (length > 16) ? 16 : length;
        memcpy(data, buffer, bytesToCopy);

        return StatusCode::MI_OK;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：WriteNTAG
    // 功    能：写入NTAG/Ultralight页面
    // 参数说明: page[IN]: 页地址
    //           data[IN]: 要写入的数据
    //           length[IN]: 要写入的字节数，默认为4（一个页面）
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::WriteNTAG(uint8_t page, const uint8_t *data, uint8_t length)
    {
        StatusCode status;
        uint32_t unLen;
        uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN];

        // NTAG/Ultralight使用不同的写命令，每次只能写4字节（一页）
        ucComMF522Buf[0] = static_cast<uint8_t>(PICC_Command::WRITE);
        ucComMF522Buf[1] = page;
        CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

        if ((status != StatusCode::MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            return StatusCode::MI_ERR;
        }

        // 准备要写入的数据，最多4字节
        uint8_t bytesToWrite = (length > 4) ? 4 : length;
        for (uint8_t i = 0; i < bytesToWrite; i++)
        {
            ucComMF522Buf[i] = data[i];
        }

        // 如果不足4字节，用0填充
        for (uint8_t i = bytesToWrite; i < 4; i++)
        {
            ucComMF522Buf[i] = 0;
        }

        CalulateCRC(ucComMF522Buf, 4, &ucComMF522Buf[4]);
        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 6, ucComMF522Buf, &unLen);

        if ((status != StatusCode::MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            return StatusCode::MI_ERR;
        }

        return StatusCode::MI_OK;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ReadCardInfo
    // 功    能：读取卡片完整信息，包括类型、UID、SAK、ATQA等
    // 返    回: 成功返回包含卡片信息的CardInfo结构体，失败返回std::nullopt
    /////////////////////////////////////////////////////////////////////
    std::optional<MFRC522::CardInfo> MFRC522::ReadCardInfo()
    {
        CardInfo info;
        StatusCode status;
        uint8_t tag_type_buffer[2];

        // 初始化CardInfo结构体
        info.type = PICC_Type::UNKNOWN;
        info.uidLength = 0;
        info.sak = 0;
        info.atqa[0] = 0;
        info.atqa[1] = 0;
        info.atsLength = 0;
        memset(info.uid, 0, MAX_UID_LENGTH);
        memset(info.ats, 0, sizeof(info.ats));

        // 1. 寻卡
        status = Request(PICC_Command::REQALL, tag_type_buffer);
        if (status != StatusCode::MI_OK)
        {
            return std::nullopt;
        }

        // 保存ATQA值
        info.atqa[0] = tag_type_buffer[0];
        info.atqa[1] = tag_type_buffer[1];

        // 2. 读取完整UID和SAK
        status = ReadCompleteUID(info.uid, &info.uidLength, &info.sak, info.atqa);
        if (status != StatusCode::MI_OK)
        {
            return std::nullopt;
        }

        // 3. 识别卡片类型
        info.type = IdentifyCardType(info.sak, info.atqa);

        // 4. 如果是ISO14443-4兼容卡，尝试获取ATS
        if (info.type == PICC_Type::ISO_14443_4 || info.type == PICC_Type::MIFARE_DESFIRE)
        {
            // 发送RATS命令
            uint8_t rats_command[2] = {static_cast<uint8_t>(PICC_Command::RATS), 0x80}; // 0x80表示FSD=256字节，CID=0
            uint8_t ats_buffer[20];
            uint8_t ats_length = sizeof(ats_buffer);

            status = TransceiveISO14443_4(rats_command, sizeof(rats_command), ats_buffer, &ats_length);
            if (status == StatusCode::MI_OK && ats_length > 0)
            {
                // 复制ATS数据
                info.atsLength = (ats_length > sizeof(info.ats)) ? sizeof(info.ats) : ats_length;
                memcpy(info.ats, ats_buffer, info.atsLength);
            }
        }

        return info;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：ReadDESFireFile
    // 功    能：读取DESFire卡文件内容
    // 参数说明: fileNo[IN]: 文件编号
    //           data[OUT]: 数据缓冲区
    //           length[IN/OUT]: 输入缓冲区大小，输出实际读取的字节数
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::ReadDESFireFile(uint8_t fileNo, uint8_t *data, uint16_t *length)
    {
        StatusCode status;
        uint8_t command[7];
        uint8_t response[256];
        uint8_t responseLength = 255;

        // 1. 选择应用（这里使用主应用AID=000000）
        command[0] = 0x5A; // SELECT_APPLICATION命令
        command[1] = 0x00; // AID低字节
        command[2] = 0x00; // AID中字节
        command[3] = 0x00; // AID高字节

        status = TransceiveISO14443_4(command, 4, response, &responseLength);
        if (status != StatusCode::MI_OK || responseLength < 1 || response[0] != 0x00)
        {
            if (m_debug_enabled)
            {
                printf("SELECT_APPLICATION failed: %02X\n", (responseLength > 0) ? response[0] : 0xFF);
            }
            return StatusCode::MI_ERR;
        }

        // 2. 读取文件数据
        command[0] = 0xBD;   // READ_DATA命令
        command[1] = fileNo; // 文件ID
        command[2] = 0x00;   // 偏移量（3字节，低字节）
        command[3] = 0x00;   // 偏移量（中字节）
        command[4] = 0x00;   // 偏移量（高字节）
        command[5] = 0x00;   // 长度（3字节，低字节）
        command[6] = 0x00;   // 长度（中字节）
        // 如果长度为0，表示读取整个文件

        responseLength = 255;
        status = TransceiveISO14443_4(command, 7, response, &responseLength);
        if (status != StatusCode::MI_OK || responseLength < 1 || response[0] != 0x00)
        {
            if (m_debug_enabled)
            {
                printf("READ_DATA failed: %02X\n", (responseLength > 0) ? response[0] : 0xFF);
            }
            return StatusCode::MI_ERR;
        }

        // 3. 复制数据到输出缓冲区
        uint16_t dataLength = responseLength - 1; // 减去状态字节
        if (dataLength > *length)
        {
            dataLength = *length; // 确保不超出缓冲区
        }

        memcpy(data, &response[1], dataLength);
        *length = dataLength;

        return StatusCode::MI_OK;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：DumpRegister
    // 功    能：打印指定寄存器的内容
    // 参数说明: reg[IN]: 寄存器地址
    /////////////////////////////////////////////////////////////////////
    void MFRC522::DumpRegister(PCD_Register reg)
    {
        uint8_t value = ReadRegister(reg);
        printf("Register %02X (%s): %02X\n", static_cast<uint8_t>(reg), GetRegisterName(reg), value);

        // 对某些重要寄存器显示更详细的信息
        switch (reg)
        {
        case PCD_Register::ComIrqReg:
            printf("  Set1 = %d (Timer interrupt)\n", (value & 0x01) ? 1 : 0);
            printf("  TxIRq = %d (Transmit interrupt)\n", (value & 0x02) ? 1 : 0);
            printf("  RxIRq = %d (Receive interrupt)\n", (value & 0x04) ? 1 : 0);
            printf("  IdleIRq = %d (Idle interrupt)\n", (value & 0x08) ? 1 : 0);
            printf("  HiAlertIRq = %d (High alert interrupt)\n", (value & 0x10) ? 1 : 0);
            printf("  LoAlertIRq = %d (Low alert interrupt)\n", (value & 0x20) ? 1 : 0);
            printf("  ErrIRq = %d (Error interrupt)\n", (value & 0x40) ? 1 : 0);
            printf("  TimerIRq = %d (Timer interrupt)\n", (value & 0x80) ? 1 : 0);
            break;

        case PCD_Register::ErrorReg:
            printf("  ProtocolErr = %d (Protocol error)\n", (value & 0x01) ? 1 : 0);
            printf("  ParityErr = %d (Parity error)\n", (value & 0x02) ? 1 : 0);
            printf("  CRCErr = %d (CRC error)\n", (value & 0x04) ? 1 : 0);
            printf("  CollErr = %d (Collision error)\n", (value & 0x08) ? 1 : 0);
            printf("  BufferOvfl = %d (Buffer overflow)\n", (value & 0x10) ? 1 : 0);
            printf("  TempErr = %d (Temperature error)\n", (value & 0x40) ? 1 : 0);
            printf("  WrErr = %d (Write error)\n", (value & 0x80) ? 1 : 0);
            break;

        case PCD_Register::Status1Reg:
            printf("  TRunning = %d (Timer running)\n", (value & 0x01) ? 1 : 0);
            printf("  CRCReady = %d (CRC ready)\n", (value & 0x20) ? 1 : 0);
            printf("  CRCOk = %d (CRC OK)\n", (value & 0x40) ? 1 : 0);
            break;

        case PCD_Register::Status2Reg:
            printf("  ModemState = %d (Modem state)\n", value & 0x07);
            printf("  MFCrypto1On = %d (MF Crypto1 on)\n", (value & 0x08) ? 1 : 0);
            break;

        case PCD_Register::VersionReg:
            printf("  Version: ");
            switch (value)
            {
            case 0x88:
                printf("FM17522\n");
                break;
            case 0x90:
                printf("Version 0.0\n");
                break;
            case 0x91:
                printf("Version 1.0\n");
                break;
            case 0x92:
                printf("Version 2.0\n");
                break;
            default:
                printf("Unknown\n");
                break;
            }
            break;
        }
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：GetRegisterName
    // 功    能：获取寄存器的名称字符串
    // 参数说明: reg[IN]: 寄存器地址
    // 返    回: 寄存器名称字符串
    /////////////////////////////////////////////////////////////////////
    const char *MFRC522::GetRegisterName(PCD_Register reg)
    {
        switch (reg)
        {
        case PCD_Register::CommandReg:
            return "CommandReg";
        case PCD_Register::ComIEnReg:
            return "ComIEnReg";
        case PCD_Register::DivlEnReg:
            return "DivlEnReg";
        case PCD_Register::ComIrqReg:
            return "ComIrqReg";
        case PCD_Register::DivIrqReg:
            return "DivIrqReg";
        case PCD_Register::ErrorReg:
            return "ErrorReg";
        case PCD_Register::Status1Reg:
            return "Status1Reg";
        case PCD_Register::Status2Reg:
            return "Status2Reg";
        case PCD_Register::FIFODataReg:
            return "FIFODataReg";
        case PCD_Register::FIFOLevelReg:
            return "FIFOLevelReg";
        case PCD_Register::WaterLevelReg:
            return "WaterLevelReg";
        case PCD_Register::ControlReg:
            return "ControlReg";
        case PCD_Register::BitFramingReg:
            return "BitFramingReg";
        case PCD_Register::CollReg:
            return "CollReg";
        case PCD_Register::ModeReg:
            return "ModeReg";
        case PCD_Register::TxModeReg:
            return "TxModeReg";
        case PCD_Register::RxModeReg:
            return "RxModeReg";
        case PCD_Register::TxControlReg:
            return "TxControlReg";
        case PCD_Register::TxAskReg:
            return "TxAskReg";
        case PCD_Register::TxSelReg:
            return "TxSelReg";
        case PCD_Register::RxSelReg:
            return "RxSelReg";
        case PCD_Register::RxThresholdReg:
            return "RxThresholdReg";
        case PCD_Register::DemodReg:
            return "DemodReg";
        case PCD_Register::MifareReg:
            return "MifareReg";
        case PCD_Register::SerialSpeedReg:
            return "SerialSpeedReg";
        case PCD_Register::CRCResultRegM:
            return "CRCResultRegM";
        case PCD_Register::CRCResultRegL:
            return "CRCResultRegL";
        case PCD_Register::ModWidthReg:
            return "ModWidthReg";
        case PCD_Register::RFCfgReg:
            return "RFCfgReg";
        case PCD_Register::GsNReg:
            return "GsNReg";
        case PCD_Register::CWGsCfgReg:
            return "CWGsCfgReg";
        case PCD_Register::ModGsCfgReg:
            return "ModGsCfgReg";
        case PCD_Register::TModeReg:
            return "TModeReg";
        case PCD_Register::TPrescalerReg:
            return "TPrescalerReg";
        case PCD_Register::TReloadRegH:
            return "TReloadRegH";
        case PCD_Register::TReloadRegL:
            return "TReloadRegL";
        case PCD_Register::TCounterValueRegH:
            return "TCounterValueRegH";
        case PCD_Register::TCounterValueRegL:
            return "TCounterValueRegL";
        case PCD_Register::TestSel1Reg:
            return "TestSel1Reg";
        case PCD_Register::TestSel2Reg:
            return "TestSel2Reg";
        case PCD_Register::TestPinEnReg:
            return "TestPinEnReg";
        case PCD_Register::TestPinValueReg:
            return "TestPinValueReg";
        case PCD_Register::TestBusReg:
            return "TestBusReg";
        case PCD_Register::AutoTestReg:
            return "AutoTestReg";
        case PCD_Register::VersionReg:
            return "VersionReg";
        case PCD_Register::AnalogTestReg:
            return "AnalogTestReg";
        case PCD_Register::TestDAC1Reg:
            return "TestDAC1Reg";
        case PCD_Register::TestDAC2Reg:
            return "TestDAC2Reg";
        case PCD_Register::TestADCReg:
            return "TestADCReg";
        default:
            return "Unknown";
        }
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：DumpAllRegisters
    // 功    能：打印所有寄存器的内容
    /////////////////////////////////////////////////////////////////////
    void MFRC522::DumpAllRegisters()
    {
        printf("Register Dump:\n");
        printf("=============================================================\n");

        // 打印所有寄存器
        // PAGE 0
        DumpRegister(PCD_Register::CommandReg);
        DumpRegister(PCD_Register::ComIEnReg);
        DumpRegister(PCD_Register::DivlEnReg);
        DumpRegister(PCD_Register::ComIrqReg);
        DumpRegister(PCD_Register::DivIrqReg);
        DumpRegister(PCD_Register::ErrorReg);
        DumpRegister(PCD_Register::Status1Reg);
        DumpRegister(PCD_Register::Status2Reg);
        DumpRegister(PCD_Register::FIFOLevelReg);
        DumpRegister(PCD_Register::WaterLevelReg);
        DumpRegister(PCD_Register::ControlReg);
        DumpRegister(PCD_Register::BitFramingReg);
        DumpRegister(PCD_Register::CollReg);

        // PAGE 1
        DumpRegister(PCD_Register::ModeReg);
        DumpRegister(PCD_Register::TxModeReg);
        DumpRegister(PCD_Register::RxModeReg);
        DumpRegister(PCD_Register::TxControlReg);
        DumpRegister(PCD_Register::TxAskReg);
        DumpRegister(PCD_Register::TxSelReg);
        DumpRegister(PCD_Register::RxSelReg);
        DumpRegister(PCD_Register::RxThresholdReg);
        DumpRegister(PCD_Register::DemodReg);
        DumpRegister(PCD_Register::MifareReg);
        DumpRegister(PCD_Register::SerialSpeedReg);

        // PAGE 2
        DumpRegister(PCD_Register::CRCResultRegM);
        DumpRegister(PCD_Register::CRCResultRegL);
        DumpRegister(PCD_Register::ModWidthReg);
        DumpRegister(PCD_Register::RFCfgReg);
        DumpRegister(PCD_Register::GsNReg);
        DumpRegister(PCD_Register::CWGsCfgReg);
        DumpRegister(PCD_Register::ModGsCfgReg);
        DumpRegister(PCD_Register::TModeReg);
        DumpRegister(PCD_Register::TPrescalerReg);
        DumpRegister(PCD_Register::TReloadRegH);
        DumpRegister(PCD_Register::TReloadRegL);
        DumpRegister(PCD_Register::TCounterValueRegH);
        DumpRegister(PCD_Register::TCounterValueRegL);

        // PAGE 3
        DumpRegister(PCD_Register::TestSel1Reg);
        DumpRegister(PCD_Register::TestSel2Reg);
        DumpRegister(PCD_Register::TestPinEnReg);
        DumpRegister(PCD_Register::TestPinValueReg);
        DumpRegister(PCD_Register::TestBusReg);
        DumpRegister(PCD_Register::AutoTestReg);
        DumpRegister(PCD_Register::VersionReg);
        DumpRegister(PCD_Register::AnalogTestReg);
        DumpRegister(PCD_Register::TestDAC1Reg);
        DumpRegister(PCD_Register::TestDAC2Reg);
        DumpRegister(PCD_Register::TestADCReg);

        printf("=============================================================\n");
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：TransceiveISO14443_4
    // 功    能：使用ISO14443-4协议与卡片通信
    // 参数说明: sendData[IN]: 要发送的数据
    //           sendLen[IN]: 发送数据的长度
    //           recvData[OUT]: 接收数据的缓冲区
    //           recvLen[OUT]: 接收数据的长度
    // 返    回: 成功返回MI_OK
    /////////////////////////////////////////////////////////////////////
    MFRC522::StatusCode MFRC522::TransceiveISO14443_4(const uint8_t *sendData, uint8_t sendLen,
                                                      uint8_t *recvData, uint8_t *recvLen)
    {
        // 这是一个简化的ISO14443-4协议实现
        // 实际上需要处理更多的协议细节，如块链接、CID、NAD等

        StatusCode status;
        uint32_t unLen;
        uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN];

        // 复制要发送的数据
        if (sendLen > MAX_COMM_BUFFER_LEN)
        {
            sendLen = MAX_COMM_BUFFER_LEN;
        }

        memcpy(ucComMF522Buf, sendData, sendLen);

        // 发送数据并接收响应
        status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, sendLen, ucComMF522Buf, &unLen);

        if (status == StatusCode::MI_OK)
        {
            // 计算接收到的字节数
            uint8_t bytesReceived = unLen / 8;
            if (unLen % 8 != 0)
            {
                bytesReceived++;
            }

            // 确保不超出接收缓冲区
            if (bytesReceived > *recvLen)
            {
                bytesReceived = *recvLen;
            }

            // 复制接收到的数据
            memcpy(recvData, ucComMF522Buf, bytesReceived);
            *recvLen = bytesReceived;
        }

        return status;
    }

    /////////////////////////////////////////////////////////////////////
    // 函数名称：DumpCardInfo
    // 功    能：打印卡片信息
    // 参数说明: info[IN]: 卡片信息结构体
    /////////////////////////////////////////////////////////////////////
    void MFRC522::DumpCardInfo(const CardInfo &info)
    {
        printf("Card Type: ");
        switch (info.type)
        {
        case PICC_Type::MIFARE_1K:
            printf("Mifare Classic 1K\n");
            break;
        case PICC_Type::MIFARE_4K:
            printf("Mifare Classic 4K\n");
            break;
        case PICC_Type::MIFARE_UL:
            printf("Mifare Ultralight\n");
            break;
        case PICC_Type::NTAG21X:
            printf("NTAG21x\n");
            break;
        case PICC_Type::MIFARE_DESFIRE:
            printf("Mifare DESFire\n");
            break;
        case PICC_Type::ISO_14443_4:
            printf("ISO/IEC 14443-4 compatible\n");
            break;
        case PICC_Type::ISO_18092:
            printf("ISO/IEC 18092 (NFC) compatible\n");
            break;
        default:
            printf("Unknown\n");
            break;
        }

        printf("UID (length=%u): ", info.uidLength);
        for (uint8_t i = 0; i < info.uidLength; i++)
        {
            printf("%02X ", info.uid[i]);
        }
        printf("\n");

        printf("SAK: 0x%02X\n", info.sak);
        printf("ATQA: 0x%02X 0x%02X\n", info.atqa[0], info.atqa[1]);

        if (info.atsLength > 0)
        {
            printf("ATS: ");
            for (uint8_t i = 0; i < info.atsLength; i++)
            {
                printf("%02X ", info.ats[i]);
            }
            printf("\n");
        }
    }
}
