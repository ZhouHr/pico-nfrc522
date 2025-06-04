#include "MFRC522.h"
// #include "beep.h"
#include "string.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <stdio.h>

constexpr uint8_t MAX_COMM_BUFFER_LEN = 18; // Buffer for commands and responses (e.g. 16 data + 2 CRC)
const uint8_t MFRC522::DefaultKeyA[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Card_Data[30];
uint8_t WriteData[16] = {0};

// Constructor with custom configuration
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

// Default constructor
MFRC522::MFRC522()
    : m_spi(spi_default), // Use Pico SDK's default SPI instance (typically spi0)
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
    if (!spi_initialized)
    {
        spi_init(m_spi, m_baud_rate);
        spi_set_format(m_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    }
}

/////////////////////////////////////////////////////////////////////
// 函数名称：MFRC522_Reset
// 功    能：通过 RST 引脚复位 MFRC522 模块
/////////////////////////////////////////////////////////////////////
void MFRC522::Reset()
{
    gpio_put(m_pin_rst, 0); // 拉低 RST
    sleep_ms(50);           // Maintain reset low for a period
    gpio_put(m_pin_rst, 1); // 拉高，释放复位
    sleep_ms(50);           // Wait for the chip to stabilize
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

    // Check if CommandReg is cleared (IDLE)
    if (ReadRegister(PCD_Register::CommandReg) & (1 << 4))
    { // Check PowerDown bit, if still set, reset failed
      // This check might not be robust, RESET_PHASE should clear it.
      // A better check might be to see if VersionReg is readable.
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
    ;
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
    // unsigned char i, ucaddr;
    // unsigned char ucResult = 0;

    gpio_put(m_pin_sck, GPIO_PIN_RESET); // 将CLK拉低
    gpio_put(m_pin_cs, GPIO_PIN_RESET);  // 将CSS拉低
    /*地址处理：bit7读写位(1读/0写)，bit6~bit1地址，bit0必须为0*/
    // ucaddr = ((addr << 1) & 0x7e) | 0x80;
    uint8_t addr_byte = (static_cast<uint8_t>(reg) << 1) | 0x80; // Bit 0 is 0. MSB is 1 for read.
    uint8_t rx_data;

    // /*发送寄存器地址*/
    // for (i = 8; i > 0; i--)
    // {
    //     if (ucaddr & 0x80) // 通过判断地址MSB的值，将PB15置0或置1
    //     {
    //         gpio_put(mfrc522_pin_miso, GPIO_PIN_SET);
    //     }
    //     else
    //     {
    //         gpio_put(mfrc522_pin_miso, GPIO_PIN_RESET);
    //     }
    //     /*CLK跳变：完成1个bit的发送*/
    //     gpio_put(mfrc522_pin_sck, GPIO_PIN_SET);
    //     ucaddr <<= 1; // 地址左移1位后，继续判断MSB的值
    //     gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET);
    // }

    // /*读取寄存器返回的值*/
    // for (i = 8; i > 0; i--)
    // {
    //     /*CLK跳变：完成1个bit的读取*/
    //     gpio_put(mfrc522_pin_sck, GPIO_PIN_SET);
    //     ucResult <<= 1; // 左移一位，空出LSB位存放下一位读取值
    //     ucResult |= gpio_get(mfrc522_pin_miso);
    //     gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET);
    // }

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
void MFRC522::CalulateCRC(const uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
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
    pOutData[0] = ReadRegister(PCD_Register::CRCResultRegL); // CRC校验的低8位
    pOutData[1] = ReadRegister(PCD_Register::CRCResultRegM); // CRC校验的高8位
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

    if ((status == StatusCode::MI_OK) && (unLen == 0x10)) // 没有错误并接接收为2个字节
    {
        *pTagType = ucComMF522Buf[0];       // 取接收缓冲区的第一个字节
        *(pTagType + 1) = ucComMF522Buf[1]; // 取接收缓冲区的第二个字节
    }
    else
    {
        status = StatusCode::MI_ERR;
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
MFRC522::StatusCode MFRC522::Anticollision(uint8_t *pSnr)
{
    StatusCode status;
    uint32_t unLen;
    uint8_t i, snr_check = 0;
    uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN];

    ClearRegisterBitMask(PCD_Register::Status2Reg, 0x08);             // 清除标志位
    WriteRegister(PCD_Register::BitFramingReg, 0x00);                 // 000 指示最后一个字节的所有位将被发送。
    ClearRegisterBitMask(PCD_Register::CollReg, 0x80);                // 发生碰撞所有接收位将被清除
    ucComMF522Buf[0] = static_cast<uint8_t>(PICC_Command::ANTICOLL1); // 0x93 防冲撞 发到卡里的命令
    ucComMF522Buf[1] = 0x20;
    // 获得卡的序列号，ucComMF522Buf[]
    status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);
    if (status == StatusCode::MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = ucComMF522Buf[i]; // 返回卡的序列号
            snr_check ^= ucComMF522Buf[i];  // 计算校验码
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = StatusCode::MI_ERR;
        } // 有错误
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
    uint8_t ucComMF522Buf[MAX_COMM_BUFFER_LEN]; // MAX_COMM_BUFFER_LEN = 18

    ucComMF522Buf[0] = static_cast<uint8_t>(PICC_Command::ANTICOLL1); // 防冲撞命令
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSerialNumber + i); // 填充卡的序列号
        ucComMF522Buf[6] ^= *(pSerialNumber + i);    // 计算校验码
    }
    // memcpy(&command_data[2], pSerialNumber, 5);     // UID(4) + BCC(1)
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]); // 获得CRC校验结果的16位值
                                                    // 放入command_data【0，1】

    printf("SelectTag: Sending command_data (9 bytes): ");
    for (int i = 0; i < 7; ++i)
    {
        printf("%02X ", ucComMF522Buf[i]);
    }
    printf("\n");

    ClearRegisterBitMask(PCD_Register::Status2Reg, 0x08); // 清零MFAuthent Command执行成功标志位
    // 把CRC值和卡号发的卡里
    status = CommunicateWithTag(PCD_Command::TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &received_bits);
    printf("SelectTag CommunicateWithTag. Status code: %u\n", static_cast<unsigned int>(status));
    printf("SelectTag: SAK = 0x%02X\n", ucComMF522Buf[0]);
    if (status == StatusCode::MI_OK && received_bits > 0 && pSakBuffer)
    {
        unsigned int len_bytes = received_bits / 8;
        if (received_bits % 8 != 0)
            len_bytes++; // account for partial bytes if any
        if (len_bytes > MAX_COMM_BUFFER_LEN)
            len_bytes = MAX_COMM_BUFFER_LEN;

        printf("SelectTag: Response buffer (first %u bytes): ", len_bytes);
        for (unsigned int i = 0; i < len_bytes; ++i)
        {
            printf("%02X ", ucComMF522Buf[i]);
        }
        printf("\n");
    }
    if ((status == StatusCode::MI_OK) && (received_bits > 0)) // 返回24个字节&状态为无错误 SAK (1 byte) + CRC_A (2 bytes) = 24 bits
    {
        if (pSakBuffer != nullptr)
        {
            *pSakBuffer = ucComMF522Buf[0]; // 存储SAK值
        }
        if (ucComMF522Buf[0] & 0x04)
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
    printf("Authenticate CommunicateWithTag read card. Status code: %u\n", static_cast<unsigned int>(status));
    if ((status != StatusCode::MI_OK) || (!(ReadRegister(PCD_Register::Status2Reg) & 0x08))) // 密码验证是否成功
    {
        status = StatusCode::MI_ERR;
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

    if ((status != StatusCode::MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
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
        if ((status != StatusCode::MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
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

MFRC522::StatusCode MFRC522::ReadCardUIDAndData(uint8_t blockAddr, uint8_t *pDataBuffer16Bytes, uint8_t *pUidBuffer4Bytes)
{
    StatusCode status;
    uint8_t tag_type_buffer[2];
    uint8_t serial_buffer_with_bcc[5]; // 4 UID + 1 BCC

    // It's good practice to ensure antenna is on, but SoftwareReset usually handles this.
    // AntennaOn();

    status = Request(PICC_Command::REQALL, tag_type_buffer);
    printf("Request card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
        return status;

    status = Anticollision(serial_buffer_with_bcc);
    printf("Anticollision card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
        return status;
    memcpy(pUidBuffer4Bytes, serial_buffer_with_bcc, 4); // Copy only the 4 UID bytes
    printf("ReadCardUIDAndData UID: ");
    for (int i = 0; i < 4; i++)
    {
        printf("%02X ", pUidBuffer4Bytes[i]);
    }
    printf("\n");

    status = SelectTag(serial_buffer_with_bcc); // Select using full UID + BCC
    printf("SelectTag card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
        return status;

    status = Authenticate(PICC_Command::AUTHENT1A, blockAddr, DefaultKeyA, pUidBuffer4Bytes);
    printf("Authenticate card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
        return status;

    status = ReadBlock(blockAddr, pDataBuffer16Bytes);
    printf("ReadBlock card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
        return status;

    // Optionally, halt the tag after operation
    // HaltTag();

    return StatusCode::MI_OK;
}

// 实现新的 ReadIso14443aData 方法
MFRC522::StatusCode MFRC522::ReadIso14443aData(uint8_t pageAddr, uint8_t *pDataBuffer16Bytes, uint8_t *pUidBuffer4Bytes)
{
    StatusCode status;
    uint8_t tag_type_buffer[2];
    uint8_t serial_buffer_with_bcc[5]; // 4 UID + 1 BCC
    uint8_t sak_value = 0xFF;          // 初始化为一个无效的SAK值

    // 1. 寻卡 (Request)
    status = Request(PICC_Command::REQALL, tag_type_buffer);
    printf("Request card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
    {
        // printf("ReadIso14443aData: Request failed (%u)\n", static_cast<unsigned int>(status));
        return status;
    }

    // 2. 防冲突 (Anticollision)
    status = Anticollision(serial_buffer_with_bcc);
    printf("Anticollision card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
    {
        // printf("ReadIso14443aData: Anticollision failed (%u)\n", static_cast<unsigned int>(status));
        return status;
    }
    memcpy(pUidBuffer4Bytes, serial_buffer_with_bcc, 4); // 提取4字节UID

    // 3. 选卡 (SelectTag) 并获取SAK
    status = SelectTag(serial_buffer_with_bcc, &sak_value);
    printf("SelectTag card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
    {
        // printf("ReadIso14443aData: SelectTag failed (%u)\n", static_cast<unsigned int>(status));
        return status;
    }
    printf("ReadIso14443aData: SAK = 0x%02X\n", sak_value);

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
    // Note: More sophisticated SAK parsing might be needed for a wider range of cards.

    if (needs_authentication)
    {
        // printf("ReadIso14443aData: Needs Mifare Authentication for block/page %u.\n", pageAddr);
        status = Authenticate(PICC_Command::AUTHENT1A, pageAddr, DefaultKeyA, pUidBuffer4Bytes);
        printf("Authenticate card. Status code: %u\n", static_cast<unsigned int>(status));
        if (status != StatusCode::MI_OK)
        {
            // printf("ReadIso14443aData: Authenticate failed (%u)\n", static_cast<unsigned int>(status));
            return status;
        }
    }
    else
    {
        // printf("ReadIso14443aData: Skipping Mifare Authentication for SAK 0x%02X, page %u.\n", sak_value, pageAddr);
    }

    // 5. 读取数据块/页面
    // ReadBlock 方法读取16字节。对于NTAG/Ultralight, pageAddr是起始页地址，它会读取4个连续页。
    status = ReadBlock(pageAddr, pDataBuffer16Bytes);
    printf("ReadBlock card. Status code: %u\n", static_cast<unsigned int>(status));
    if (status != StatusCode::MI_OK)
    {
        // printf("ReadIso14443aData: ReadBlock failed (%u)\n", static_cast<unsigned int>(status));
        return status;
    }

    return StatusCode::MI_OK;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdValue
// 功    能：扣款和充值
// 参数说明: dd_mode[IN]：命令字
//                        0xC0 = 扣款
//                        0xC1 = 充值
//           addr[IN]：钱包地址
//           pValue[IN]：4字节增(减)值，低位在前
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
/*
char PcdValue(unsigned char dd_mode, unsigned char addr, unsigned char *pValue)
{
    char status;
    unsigned int unLen;
    unsigned char i, ucComMF522Buf[MAX_COMM_BUFFER_LEN];

    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i = 0; i < 16; i++)
        {
            ucComMF522Buf[i] = *(pValue + i);
        }
        CalulateCRC(ucComMF522Buf, 4, &ucComMF522Buf[4]);
        unLen = 0;
        status = PcdComMF522(PCD_Command::TRANSCEIVE, ucComMF522Buf, 6, ucComMF522Buf, &unLen);
        if (status != MI_ERR)
        {
            status = MI_OK;
        }
    }

    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_Command::TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

        status = PcdComMF522(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }
    return status;
}
    */
/////////////////////////////////////////////////////////////////////
// 函数名称：PcdBakValue
// 功    能：备份钱包
// 参数说明: sourceaddr[IN]：源地址
//           goaladdr[IN]：目标地址
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
/*
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAX_COMM_BUFFER_LEN];

    ucComMF522Buf[0] = PICC_Command::RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        ucComMF522Buf[0] = 0;
        ucComMF522Buf[1] = 0;
        ucComMF522Buf[2] = 0;
        ucComMF522Buf[3] = 0;
        CalulateCRC(ucComMF522Buf, 4, &ucComMF522Buf[4]);

        status = PcdComMF522(PCD_Command::TRANSCEIVE, ucComMF522Buf, 6, ucComMF522Buf, &unLen);
        if (status != MI_ERR)
        {
            status = MI_OK;
        }
    }

    if (status != MI_OK)
    {
        return MI_ERR;
    }

    ucComMF522Buf[0] = PICC_Command::TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_Command::TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    return status;
}
    */

/////////////////////////////////////////////////////////////////////
// 函数名称：RFID_Read_Data
// 功    能：读取M1卡一块数据
// 参数说明: id M1卡的块号
/////////////////////////////////////////////////////////////////////
/*
void RFID_Read_Data(unsigned char id)
{
    uint8_t i;
    unsigned char status = 0;
    PcdReset();      // 初始化射频芯片
    PcdAntennaOff(); // 关闭天线
    AntennaOn();     // 打开天线
    while (1)
    {
        // 寻卡
        status = PcdRequest(PICC_Command::REQALL, Card_Data);
        if (status != MI_OK)
        {
            break;
        }
        // 防冲撞处理
        status = PcdAnticoll(Card_Data); // 执行成功Card_Data[]得到卡的序列号
        if (status != MI_OK)
        {
            break;
        }
        // 选择卡片
        status = PcdSelect(Card_Data);
        if (status != MI_OK)
        {
            break;
        }
        // 验证卡片密码
        status = PcdAuthState(PICC_Command::AUTHENT1A, id, DefaultKey, Card_Data);
        if (status != MI_OK)
        {
            break;
        }
        // 读地址的数据
        memset(Card_Data, 0, sizeof(Card_Data));
        status = PcdRead(id, Card_Data);
        if (status != MI_OK)
        {
            break;
        }
        printf("Card_Data:\n");
        for (i = 0; i < 16; i++)
        {
            printf(" %02x ", Card_Data[i]);
        }
        printf("\n");
        // TIP_SUCCESS();
        printf("Operation read successful!\n");
        PcdHalt();
        break;
    }
}
    */

/////////////////////////////////////////////////////////////////////
// 函数名称：RFID_Write_Data
// 功    能：向M1卡的一块中写入数据
// 参数说明: id M1卡的块号
/////////////////////////////////////////////////////////////////////
/*
void RFID_Write_Data(unsigned char id)
{
    uint8_t i;
    unsigned char status = 0;
    PcdReset();      // 初始化射频芯片
    PcdAntennaOff(); // 关闭天线
    AntennaOn();     // 打开天线
    while (1)
    {
        // 寻卡
        status = PcdRequest(PICC_Command::REQALL, Card_Data);
        if (status != MI_OK)
        {
            continue;
        }
        // 防冲撞处理
        status = PcdAnticoll(Card_Data); // 执行成功Card_Data[]得到卡的序列号
        if (status != MI_OK)
        {
            continue;
        }
        // 选择卡片
        status = PcdSelect(Card_Data);
        if (status != MI_OK)
        {
            continue;
        }
        // 验证卡片密码
        status = PcdAuthState(PICC_Command::AUTHENT1A, id, DefaultKey, Card_Data);
        if (status != MI_OK)
        {
            continue;
        }
        // 向卡写数据
        status = PcdWrite(id, WriteData); // 地址满足 （地址+1）/4 ！= int
        if (status != MI_OK)
        {
            continue;
        }
        // 读地址的数据
        memset(Card_Data, 0, sizeof(Card_Data));
        status = PcdRead(id, Card_Data);
        if (status != MI_OK)
        {
            continue;
        }
        printf("Card_Data:\n");
        for (i = 0; i < 16; i++)
        {
            printf(" %02x ", Card_Data[i]);
        }
        printf("\n");
        // TIP_WRITE_SUCCESS();
        printf("Operation write successful!\n");
        PcdHalt();
        break;
    }
}
    */
