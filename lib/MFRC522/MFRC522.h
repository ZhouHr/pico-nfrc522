#ifndef MFRC522_H_
#define MFRC522_H_

// #include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
// #include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

struct MFRC522_Config
{
    spi_inst_t *spi;    // SPI 实例
    uint pin_miso;      // MISO 引脚
    uint pin_cs;        // CS 引脚
    uint pin_sck;       // SCK 引脚
    uint pin_mosi;      // MOSI 引脚
    uint pin_rst;       // RST 引脚（新增）
    uint32_t baud_rate; // 波特率
};

class MFRC522
{
public:
    /////////////////////////////////////////////////////////////////////
    // MF522命令字
    /////////////////////////////////////////////////////////////////////
    enum class PCD_Command : uint8_t
    {
        IDLE = 0x00,        // 取消当前命令
        AUTHENT = 0x0E,     // 验证密钥
        RECEIVE = 0x08,     // 接收数据
        TRANSMIT = 0x04,    // 发送数据
        TRANSCEIVE = 0x0C,  // 发送并接收数据
        RESET_PHASE = 0x0F, // 复位
        CALC_CRC = 0x03     // CRC计算
    };

    /////////////////////////////////////////////////////////////////////
    // Mifare_1卡片命令字
    /////////////////////////////////////////////////////////////////////
    enum class PICC_Command : uint8_t
    {
        REQIDL = 0x26,    // 寻天线区内未进入休眠状态
        REQALL = 0x52,    // 寻天线区内全部卡
        ANTICOLL1 = 0x93, // 防冲撞第1级
        ANTICOLL2 = 0x95, // 防冲撞第2级
        ANTICOLL3 = 0x97, // 防冲撞第3级
        AUTHENT1A = 0x60, // 验证A密钥
        AUTHENT1B = 0x61, // 验证B密钥
        READ = 0x30,      // 读块
        WRITE = 0xA0,     // 写块
        DECREMENT = 0xC0, // 扣款
        INCREMENT = 0xC1, // 充值
        RESTORE = 0xC2,   // 调块数据到缓冲区
        TRANSFER = 0xB0,  // 保存缓冲区中数据
        HALT = 0x50       // 休眠
    };

    enum class PCD_Register : uint8_t
    {
        /////////////////////////////////////////////////////////////////////
        // MF522 FIFO长度定义
        /////////////////////////////////////////////////////////////////////
        DEF_FIFO_LENGTH = 64, // FIFO size=64byte

        /////////////////////////////////////////////////////////////////////
        // MF522寄存器定义
        /////////////////////////////////////////////////////////////////////
        // PAGE 0
        RFU00 = 0x00,
        CommandReg = 0x01,
        ComIEnReg = 0x02,
        DivlEnReg = 0x03,
        ComIrqReg = 0x04,
        DivIrqReg = 0x05,
        ErrorReg = 0x06,
        Status1Reg = 0x07,
        Status2Reg = 0x08,
        FIFODataReg = 0x09,
        FIFOLevelReg = 0x0A,
        WaterLevelReg = 0x0B,
        ControlReg = 0x0C,
        BitFramingReg = 0x0D,
        CollReg = 0x0E,
        RFU0F = 0x0F,
        // PAGE 1
        RFU10 = 0x10,
        ModeReg = 0x11,
        TxModeReg = 0x12,
        RxModeReg = 0x13,
        TxControlReg = 0x14,
        TxAskReg = 0x15,
        TxSelReg = 0x16,
        RxSelReg = 0x17,
        RxThresholdReg = 0x18,
        DemodReg = 0x19,
        RFU1A = 0x1A,
        RFU1B = 0x1B,
        MifareReg = 0x1C,
        RFU1D = 0x1D,
        RFU1E = 0x1E,
        SerialSpeedReg = 0x1F,
        // PAGE 2
        RFU20 = 0x20,
        CRCResultRegM = 0x21,
        CRCResultRegL = 0x22,
        RFU23 = 0x23,
        ModWidthReg = 0x24,
        RFU25 = 0x25,
        RFCfgReg = 0x26,
        GsNReg = 0x27,
        CWGsCfgReg = 0x28,
        ModGsCfgReg = 0x29,
        TModeReg = 0x2A,
        TPrescalerReg = 0x2B,
        TReloadRegH = 0x2C,
        TReloadRegL = 0x2D,
        TCounterValueRegH = 0x2E,
        TCounterValueRegL = 0x2F,
        // PAGE 3
        RFU30 = 0x30,
        TestSel1Reg = 0x31,
        TestSel2Reg = 0x32,
        TestPinEnReg = 0x33,
        TestPinValueReg = 0x34,
        TestBusReg = 0x35,
        AutoTestReg = 0x36,
        VersionReg = 0x37,
        AnalogTestReg = 0x38,
        TestDAC1Reg = 0x39,
        TestDAC2Reg = 0x3A,
        TestADCReg = 0x3B,
        RFU3C = 0x3C,
        RFU3D = 0x3D,
        RFU3E = 0x3E,
        RFU3F = 0x3F,

    };

    /////////////////////////////////////////////////////////////////////
    // 和MF522通讯时返回的错误代码
    /////////////////////////////////////////////////////////////////////
    enum class StatusCode : uint8_t
    {
        MI_OK = 0,          // Success
        MI_NOTAGERR = 0xFF, // No tag error / Timeout
        MI_ERR = 0xFE,      // General error
    };

    // 默认 SPI 实例和引脚
    static constexpr uint DEFAULT_PIN_MISO = 16;
    static constexpr uint DEFAULT_PIN_CS = 17;
    static constexpr uint DEFAULT_PIN_SCK = 18;
    static constexpr uint DEFAULT_PIN_MOSI = 19;
    static constexpr uint DEFAULT_PIN_RST = 20;                   // 新增默认 RST 引脚
    static constexpr uint32_t DEFAULT_BAUDRATE = 1 * 1000 * 1000; // 1MHz

    static constexpr uint GPIO_PIN_RESET = 0;
    static constexpr uint GPIO_PIN_SET = 1;

    static constexpr uint8_t CARD_BLOCK_DATA_LEN = 16; // Length of a data block on a Mifare card
    static const uint8_t DefaultKeyA[6];               // Default Mifare Key A (all 0xFF)

    MFRC522(const MFRC522_Config &config);
    MFRC522();

    StatusCode SoftwareReset(); // Performs a software reset and initializes MFRC522 registers

    void Init();
    void Reset();
    StatusCode PcdReset();
    void WriteRegister(PCD_Register reg, uint8_t val);
    uint8_t ReadRegister(PCD_Register reg);
    void ClearRegisterBitMask(PCD_Register reg, uint8_t mask);
    void SetRegisterBitMask(PCD_Register reg, uint8_t mask);
    void AntennaOn();
    void AntennaOff();
    StatusCode CommunicateWithTag(PCD_Command Command, const uint8_t *pInData,
                                  uint8_t InLenByte, uint8_t *pOutData, uint32_t *pOutLenBit);
    void CalulateCRC(const uint8_t *pIndata, uint8_t len, uint8_t *pOutData);
    StatusCode Request(PICC_Command req_code, uint8_t *pTagType);
    StatusCode Anticollision(uint8_t *pSnr, uint8_t cascadeLevel = 1, uint8_t *pSak = nullptr);
    StatusCode SelectTag(const uint8_t *pSnr, uint8_t cascadeLevel = 1, uint8_t *pSakBuffer = nullptr);
    StatusCode Authenticate(PICC_Command auth_mode, uint8_t addr, const uint8_t *pKey, const uint8_t *pSnr);
    StatusCode ReadBlock(uint8_t addr, uint8_t *pData);
    StatusCode WriteBlock(uint8_t addr, const uint8_t *pData);
    StatusCode HaltTag();
    StatusCode ReadCardUIDAndData(uint8_t blockAddr, uint8_t *pDataBuffer16Bytes, uint8_t *pUidBuffer4Bytes);
    StatusCode ReadIso14443aData(uint8_t pageAddr, uint8_t *pDataBuffer16Bytes, uint8_t *pUidBuffer, uint8_t *pUidLength);
    StatusCode ReadCompleteUID(uint8_t *pUidBuffer, uint8_t *pUidLength, uint8_t *pSak = nullptr);

    // ===========
    // char PcdValue(unsigned char dd_mode, unsigned char addr, unsigned char *pValue);
    // char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);
    // void RFID_Read_Data(unsigned char id);
    // void RFID_Write_Data(unsigned char id);

private:
    // Member variables
    spi_inst_t *m_spi;
    uint m_pin_miso;
    uint m_pin_cs;
    uint m_pin_sck;
    uint m_pin_mosi;
    uint m_pin_rst;
    uint32_t m_baud_rate;

    bool spi_initialized = false;

    // void Init(); // Initializes GPIO pins and SPI peripheral
};

#endif // MFRC522_H_
