#include "MFRC522.h"
// #include "beep.h"
#include "string.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#define MAXRLEN 18
uint8_t Card_Data[30];
unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t WriteData[16] = {0};

// 引脚说明：
// STM32 PB15(MISO) <--- RC522(MOSI)
// STM32 PB14(MOSI) ---> RC522(MISO)
// STM32 PB13(CLK)  ---> RC522(SCK)
// STM32 PB12(CSS)  ---> RC522(SDA)
static spi_inst_t *mfrc522_spi = NULL;
static uint32_t mfrc522_baud_rate = DEFAULT_BAUDRATE;
static uint mfrc522_pin_mosi = DEFAULT_PIN_MOSI;
static uint mfrc522_pin_miso = DEFAULT_PIN_MISO;
static uint mfrc522_pin_sck = DEFAULT_PIN_SCK;
static uint mfrc522_pin_cs = DEFAULT_PIN_CS;
static uint mfrc522_pin_rst = DEFAULT_PIN_RST; // 新增 RST 引脚

static bool spi_initialized = false;

static void init_spi(spi_inst_t *spi, uint32_t baud_rate)
{
    // 只有在未初始化的情况下才初始化 SPI
    if (!spi_initialized || mfrc522_spi != spi)
    {
        spi_init(spi, baud_rate);
        spi_set_format(
            spi,
            8,
            SPI_CPOL_0,
            SPI_CPHA_0,
            SPI_MSB_FIRST);
    }
}

void MFRC522_DefaultInit(void)
{
    printf("MFRC522_DefaultInit called\n");
    MFRC522_Config default_config = {
        .spi = DEFAULT_SPI_PORT,
        .pin_miso = DEFAULT_PIN_MISO,
        .pin_cs = DEFAULT_PIN_CS,
        .pin_sck = DEFAULT_PIN_SCK,
        .pin_mosi = DEFAULT_PIN_MOSI,
        .pin_rst = DEFAULT_PIN_RST, // 使用默认 RST 引脚
        .baud_rate = DEFAULT_BAUDRATE};
    MFRC522_Init(&default_config);
}

void MFRC522_Init(const MFRC522_Config *config)
{
    if (config == NULL)
    {
        MFRC522_DefaultInit();
        return;
    }

    mfrc522_spi = config->spi;
    mfrc522_baud_rate = config->baud_rate;
    mfrc522_pin_mosi = config->pin_mosi;
    mfrc522_pin_miso = config->pin_miso;
    mfrc522_pin_sck = config->pin_sck;
    mfrc522_pin_cs = config->pin_cs;
    mfrc522_pin_rst = config->pin_rst;

    // 初始化 RST 引脚
    gpio_init(mfrc522_pin_rst);
    gpio_set_dir(mfrc522_pin_rst, GPIO_OUT);
    gpio_put(mfrc522_pin_rst, 1); // 默认高电平（非复位状态）

    // 设置 SPI 引脚功能
    gpio_set_function(config->pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(config->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(config->pin_mosi, GPIO_FUNC_SPI);

    // CS 引脚为普通输出
    gpio_init(config->pin_cs);
    gpio_set_dir(config->pin_cs, GPIO_OUT);
    gpio_put(config->pin_cs, 1); // 默认不选中模块

    // 初始化 SPI
    init_spi(mfrc522_spi, config->baud_rate);

    // 可选：上电后执行一次复位
    MFRC522_Reset();
}

/////////////////////////////////////////////////////////////////////
// 函数名称：MFRC522_Reset
// 功    能：通过 RST 引脚复位 MFRC522 模块
/////////////////////////////////////////////////////////////////////
void MFRC522_Reset(void)
{
    gpio_put(mfrc522_pin_rst, 0); // 拉低 RST
    sleep_ms(50);                 // 至少保持 10ms
    gpio_put(mfrc522_pin_rst, 1); // 拉高，释放复位
    sleep_ms(50);                 // 等待模块稳定
}

/////////////////////////////////////////////////////////////////////
// 函数名称：ReadRawRC
// 功    能：读RC522寄存器中的值
// 参数说明：addr 寄存器地址
/////////////////////////////////////////////////////////////////////
unsigned char ReadRawRC(unsigned char addr)
{
    unsigned char i, ucaddr;
    unsigned char ucResult = 0;

    gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET); // 将CLK拉低
    gpio_put(mfrc522_pin_cs, GPIO_PIN_RESET);  // 将CSS拉低
    /*地址处理：bit7读写位(1读/0写)，bit6~bit1地址，bit0必须为0*/
    ucaddr = ((addr << 1) & 0x7e) | 0x80;

    /*发送寄存器地址*/
    for (i = 8; i > 0; i--)
    {
        if (ucaddr & 0x80) // 通过判断地址MSB的值，将PB15置0或置1
        {
            gpio_put(mfrc522_pin_miso, GPIO_PIN_SET);
        }
        else
        {
            gpio_put(mfrc522_pin_miso, GPIO_PIN_RESET);
        }
        /*CLK跳变：完成1个bit的发送*/
        gpio_put(mfrc522_pin_sck, GPIO_PIN_SET);
        ucaddr <<= 1; // 地址左移1位后，继续判断MSB的值
        gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET);
    }

    /*读取寄存器返回的值*/
    for (i = 8; i > 0; i--)
    {
        /*CLK跳变：完成1个bit的读取*/
        gpio_put(mfrc522_pin_sck, GPIO_PIN_SET);
        ucResult <<= 1; // 左移一位，空出LSB位存放下一位读取值
        ucResult |= gpio_get(mfrc522_pin_miso);
        gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET);
    }

    gpio_put(mfrc522_pin_sck, GPIO_PIN_SET); // 将CLK拉高
    gpio_put(mfrc522_pin_cs, GPIO_PIN_SET);  // 将CSS拉高
    sleep_ms(10);
    return ucResult;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：WriteRawRC
// 功    能：向RC522寄存器中写入值
// 参数说明：addr 寄存器地址
//			val  要写入的值
/////////////////////////////////////////////////////////////////////
void WriteRawRC(unsigned char addr, unsigned char val)
{
    unsigned char i, ucaddr;
    gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET); // 将CLK拉低
    gpio_put(mfrc522_pin_cs, GPIO_PIN_RESET);  // 将CSS拉低
    /*地址处理：bit7读写位(1读/0写)，bit6~bit1地址，bit0必须为0*/
    ucaddr = ((addr << 1) & 0x7e);

    /*发送寄存器地址*/
    for (i = 8; i > 0; i--)
    {
        if (ucaddr & 0x80) // 通过判断地址MSB的值，将PB15置0或置1
        {
            gpio_put(mfrc522_pin_miso, GPIO_PIN_SET);
        }
        else
        {
            gpio_put(mfrc522_pin_miso, GPIO_PIN_RESET);
        }
        /*CLK跳变：完成1个bit的发送*/
        gpio_put(mfrc522_pin_sck, GPIO_PIN_SET);
        ucaddr <<= 1; // 地址左移1位后，继续判断MSB的值
        gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET);
    }

    /*发送要写入的数值*/
    for (i = 8; i > 0; i--)
    {
        if (val & 0x80) // 通过判断写入值MSB的值，将PB15置0或置1
        {
            gpio_put(mfrc522_pin_miso, GPIO_PIN_SET);
        }
        else
        {
            gpio_put(mfrc522_pin_miso, GPIO_PIN_RESET);
        }
        /*CLK跳变：完成1个bit的发送*/
        gpio_put(mfrc522_pin_sck, GPIO_PIN_SET);
        val <<= 1; // val左移1位后，继续判断MSB的值
        gpio_put(mfrc522_pin_sck, GPIO_PIN_RESET);
    }

    gpio_put(mfrc522_pin_sck, GPIO_PIN_SET); // 将CLK拉高
    gpio_put(mfrc522_pin_cs, GPIO_PIN_SET);  // 将CSS拉高
    sleep_ms(10);
}

/////////////////////////////////////////////////////////////////////
// 函数名称：ClearBitMask
// 功    能：清RC522寄存器位
// 参数说明：reg[IN]  寄存器地址
//           mask[IN] 清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg, unsigned char mask)
{
    char temp = 0x0;
    temp = ReadRawRC(reg);         // 读Reg寄存器的状态
    WriteRawRC(reg, temp & ~mask); // 向Reg寄存器写入tmp&~mask位
}

/////////////////////////////////////////////////////////////////////
// 函数名称：SetBitMask
// 功    能：置RC522寄存器位
// 参数说明：reg[IN]  寄存器地址
//           mask[IN] 置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg, unsigned char mask)
{
    char temp = 0x0;
    temp = ReadRawRC(reg);        // 读Reg寄存器的状态
    WriteRawRC(reg, temp | mask); // 向Reg寄存器写入 MASK|tmp位
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdAntennaOn
// 功    能：开启天线发射
// 说    明：每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn(void)
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdAntennaOff
// 功    能：开启天线发射
// 说    明：每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdReset
// 功    能：复位RC522
// 返 回 值：成功，返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdReset(void)
{
    MFRC522_Reset(); // 使用硬件复位
    WriteRawRC(CommandReg, PCD_RESETPHASE);
    sleep_ms(10);
    WriteRawRC(ModeReg, 0x3d);
    sleep_ms(10);
    WriteRawRC(TReloadRegL, 30);
    WriteRawRC(TReloadRegH, 0);
    WriteRawRC(TModeReg, 0x8d);
    WriteRawRC(TPrescalerReg, 0x3e);
    WriteRawRC(TxAskReg, 0x40);

    return MI_OK;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdComMF522
// 功    能：通过RC522和ISO14443卡通讯
// 参数说明：Command[IN]:     RC522命令字
//           pInData[IN]:     通过RC522发送到卡片的数据
//           InLenByte[IN]:   发送数据的字节长度
//           pOutData[OUT]:   接收到的卡片返回数据
//           *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
char PcdComMF522(unsigned char Command, unsigned char *pInData,
                 unsigned char InLenByte, unsigned char *pOutData, unsigned int *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
    switch (Command)
    {
    case PCD_AUTHENT:   // 0x0E  验证密钥
        irqEn = 0x12;   // 0001 0010
        waitFor = 0x10; // 0001 0000
        break;
    case PCD_TRANSCEIVE: // 0x0C  发送并接收数据
        irqEn = 0x77;    // 0111 0111
        waitFor = 0x30;  // 0011 0000
        break;
    default:
        break;
    }

    WriteRawRC(ComIEnReg, irqEn | 0x80); // 使能相应中断
    ClearBitMask(ComIrqReg, 0x80);       // 清零所有中断标志
    WriteRawRC(CommandReg, PCD_IDLE);    // 命令寄存器复零
    SetBitMask(FIFOLevelReg, 0x80);      // 复位FIFO读写指针

    /*把要发送给卡的数据写入522的FIFO数据寄存器*/
    for (i = 0; i < InLenByte; i++)
    {
        WriteRawRC(FIFODataReg, pInData[i]);
    }
    /*RC522执行发送命令*/
    WriteRawRC(CommandReg, Command);

    /*若执行的是发送数据命令*/
    if (Command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80); // 启动传输
    }

    // 退出do-while循环的IRQ条件:
    // 1.i=0
    // 2.定时器减到零中断
    // 3.接收器检测到有效的数据中断OR命令本身终止中断
    i = 500; // 根据时钟频率调整，操作M1卡最大等待时间25ms
    do
    {
        n = ReadRawRC(ComIrqReg); // 读中断标志寄存器
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitFor));

    ClearBitMask(BitFramingReg, 0x80); // 结束发送数据命令

    if (i != 0) // 射频区无卡
    {
        // 读错误标志寄存器&0001 1011不是以下错误：
        // 数据FIFO已满、碰撞错误、奇偶校验错误、SOF错误
        if (!(ReadRawRC(ErrorReg) & 0x1B))
        {
            status = MI_OK;       // 暂时无错误
            if (n & irqEn & 0x01) // 是否为定时器中断
            {
                status = MI_NOTAGERR;
            } // 错误

            if (Command == PCD_TRANSCEIVE)
            {
                n = ReadRawRC(FIFOLevelReg); // 共接收了几个字节
                // 接收的最后一个字节的有效位
                lastBits = ReadRawRC(ControlReg) & 0x07;
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

                if (n > MAXRLEN)
                {
                    n = MAXRLEN;
                } // 最多可接收18个字节
                for (i = 0; i < n; i++)
                {
                    pOutData[i] = ReadRawRC(FIFODataReg);
                } // 取出接收到的数据放到数组POUTDATA
            }
        }
        else
        {
            status = MI_ERR;
        } // 错误标志
    }

    SetBitMask(ControlReg, 0x80);     // 停止定时器
    WriteRawRC(CommandReg, PCD_IDLE); // 复位COMMANDREG
    return status;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：CalulateCRC
// 功    能：用MF522计算CRC16函数
// 参数说明:
/////////////////////////////////////////////////////////////////////
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
    unsigned char i, n;
    ClearBitMask(DivIrqReg, 0x04);    // 禁止CRC
    WriteRawRC(CommandReg, PCD_IDLE); // 复位522CommandReg寄存器
    SetBitMask(FIFOLevelReg, 0x80);   // 复位FIFO的读写指针
    for (i = 0; i < len; i++)
    {
        WriteRawRC(FIFODataReg, *(pIndata + i));
    } // 把*pIndata缓冲区的值写如FIFODataReg
    WriteRawRC(CommandReg, PCD_CALCCRC); // 执行CRC校验
    i = 0xFF;                            // 等待255us
    do
    {
        n = ReadRawRC(DivIrqReg); // 读中断请求标志寄存器
        i--;
    } while ((i != 0) && !(n & 0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL); // CRC校验的低8位
    pOutData[1] = ReadRawRC(CRCResultRegM); // CRC校验的高8位
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdRequest
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
char PcdRequest(unsigned char req_code, unsigned char *pTagType)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ClearBitMask(Status2Reg, 0x08);  // 清零Status2Reg的MFAuthent Command执行成功标志位
    WriteRawRC(BitFramingReg, 0x07); // 清零Transceive命令开始位
    SetBitMask(TxControlReg, 0x03);  // 开启天线
    ucComMF522Buf[0] = req_code;     // 取522要执行的命令
    // 向PICC发送寻天线区内全部卡命令，并接收PICC返回的数据
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x10)) // 没有错误并接接收为2个字节
    {
        *pTagType = ucComMF522Buf[0];       // 取接收缓冲区的第一个字节
        *(pTagType + 1) = ucComMF522Buf[1]; // 取接收缓冲区的第二个字节
    }
    else
    {
        status = MI_ERR;
    } // 错误

    return status;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdAnticoll
// 功    能：防冲撞
// 参数说明: pSnr[OUT]:卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i, snr_check = 0;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ClearBitMask(Status2Reg, 0x08);    // 清除标志位
    WriteRawRC(BitFramingReg, 0x00);   // 000 指示最后一个字节的所有位将被发送。
    ClearBitMask(CollReg, 0x80);       // 发生碰撞所有接收位将被清除
    ucComMF522Buf[0] = PICC_ANTICOLL1; // 0x93 防冲撞 发到卡里的命令
    ucComMF522Buf[1] = 0x20;
    // 获得卡的序列号，ucComMF522Buf[]
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);
    if (status == MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = ucComMF522Buf[i]; // 返回卡的序列号
            snr_check ^= ucComMF522Buf[i];  // 计算校验码
        }
        if (snr_check != ucComMF522Buf[i])
        {
            status = MI_ERR;
        } // 有错误
    }

    SetBitMask(CollReg, 0x80); // 置位防碰撞位
    return status;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdSelect
// 功    能：选定卡片
// 参数说明: pSnr[IN]:卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr)
{
    char status = 0XFF;
    unsigned char i;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; // MAXRLEN = 18

    ucComMF522Buf[0] = PICC_ANTICOLL1; // 防冲撞命令
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i = 0; i < 4; i++)
    {
        ucComMF522Buf[i + 2] = *(pSnr + i); // 填充卡的序列号
        ucComMF522Buf[6] ^= *(pSnr + i);    // 计算校验码
    }
    CalulateCRC(ucComMF522Buf, 7, &ucComMF522Buf[7]); // 获得CRC校验结果的16位值
                                                      // 放入ucComMF522Buf【0，1】
    ClearBitMask(Status2Reg, 0x08);                   // 清零MFAuthent Command执行成功标志位
    // 把CRC值和卡号发的卡里
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf, &unLen);

    if ((status == MI_OK) && (unLen == 0x18)) // 返回24个字节&状态为无错误
    {
        status = MI_OK;
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：PcdAuthState
// 功    能：验证卡片密码
// 参数说明: auth_mode[IN]: 密码验证模式
//                  		   0x60 = 验证A密钥
//                  		   0x61 = 验证B密钥
//           addr[IN]：块地址
//           pKey[IN]：密码
//           pSnr[IN]：卡片序列号，4字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdAuthState(unsigned char auth_mode, unsigned char addr, unsigned char *pKey, unsigned char *pSnr)
{
    char status;
    unsigned int unLen;
    unsigned char i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = auth_mode; // PICC验证A密钥指令
    ucComMF522Buf[1] = addr;      // 块地址
    for (i = 0; i < 6; i++)
    {
        ucComMF522Buf[i + 2] = *(pKey + i);
    } // 向缓冲区填充密码
    for (i = 0; i < 6; i++)
    {
        ucComMF522Buf[i + 8] = *(pSnr + i);
    } // 向缓冲区填充与密码对应的卡的序列号，有效4个字节

    status = PcdComMF522(PCD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &unLen); // 验证密码和卡号
    if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))                  // 密码验证是否成功
    {
        status = MI_ERR;
    }

    return status;
}
/////////////////////////////////////////////////////////////////////
// 函数名称：PcdRead
// 功    能：读取M1卡一块数据
// 参数说明: addr[IN]：块地址
//           pData[OUT]：读出的数据，16字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRead(unsigned char addr, unsigned char *pData)
{
    char status;
    unsigned int unLen;
    unsigned char i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    if ((status == MI_OK) && (unLen == 0x90))
    {
        for (i = 0; i < 16; i++)
        {
            *(pData + i) = ucComMF522Buf[i];
        }
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}
/////////////////////////////////////////////////////////////////////
// 函数名称：PcdWrite
// 功    能：写数据到M1卡一块
// 参数说明: addr[IN]：块地址
//           pData[IN]：写入的数据，16字节
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdWrite(unsigned char addr, unsigned char *pData)
{
    char status;
    unsigned int unLen;
    unsigned char i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_WRITE; // PICC_WRITE  0xA0   写块
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]); // 对数据进行CRC校验
                                                      // 存放于ucCoMF522Buf【0，1】
    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i = 0; i < 16; i++)
        {
            ucComMF522Buf[i] = *(pData + i);
        } // 要充值的内容
        CalulateCRC(ucComMF522Buf, 16, &ucComMF522Buf[16]);                             // 对数据进行CRC校验校验值
                                                                                        // 存放于ucCoMF522Buf【0，1】
        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf, &unLen); // 发送数据，并接收卡返回的数据
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}
/////////////////////////////////////////////////////////////////////
// 函数名称：PcdHalt
// 功    能：命令卡片进入休眠状态
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    PcdComMF522(PICC_HALT, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
    return MI_OK;
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
char PcdValue(unsigned char dd_mode, unsigned char addr, unsigned char *pValue)
{
    char status;
    unsigned int unLen;
    unsigned char i, ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = dd_mode;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

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
        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 6, ucComMF522Buf, &unLen);
        if (status != MI_ERR)
        {
            status = MI_OK;
        }
    }

    if (status == MI_OK)
    {
        ucComMF522Buf[0] = PICC_TRANSFER;
        ucComMF522Buf[1] = addr;
        CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }
    return status;
}
/////////////////////////////////////////////////////////////////////
// 函数名称：PcdBakValue
// 功    能：备份钱包
// 参数说明: sourceaddr[IN]：源地址
//           goaladdr[IN]：目标地址
// 返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
    char status;
    unsigned int unLen;
    unsigned char ucComMF522Buf[MAXRLEN];

    ucComMF522Buf[0] = PICC_RESTORE;
    ucComMF522Buf[1] = sourceaddr;
    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

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

        status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 6, ucComMF522Buf, &unLen);
        if (status != MI_ERR)
        {
            status = MI_OK;
        }
    }

    if (status != MI_OK)
    {
        return MI_ERR;
    }

    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = goaladdr;

    CalulateCRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

    status = PcdComMF522(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
// 函数名称：RFID_Read_Data
// 功    能：读取M1卡一块数据
// 参数说明: id M1卡的块号
/////////////////////////////////////////////////////////////////////
void RFID_Read_Data(unsigned char id)
{
    uint8_t i;
    unsigned char status = 0;
    PcdReset();      // 初始化射频芯片
    PcdAntennaOff(); // 关闭天线
    PcdAntennaOn();  // 打开天线
    while (1)
    {
        // 寻卡
        status = PcdRequest(PICC_REQALL, Card_Data);
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
        status = PcdAuthState(PICC_AUTHENT1A, id, DefaultKey, Card_Data);
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

/////////////////////////////////////////////////////////////////////
// 函数名称：RFID_Write_Data
// 功    能：向M1卡的一块中写入数据
// 参数说明: id M1卡的块号
/////////////////////////////////////////////////////////////////////
void RFID_Write_Data(unsigned char id)
{
    uint8_t i;
    unsigned char status = 0;
    PcdReset();      // 初始化射频芯片
    PcdAntennaOff(); // 关闭天线
    PcdAntennaOn();  // 打开天线
    while (1)
    {
        // 寻卡
        status = PcdRequest(PICC_REQALL, Card_Data);
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
        status = PcdAuthState(PICC_AUTHENT1A, id, DefaultKey, Card_Data);
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
