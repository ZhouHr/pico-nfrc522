#ifndef MFRC522_H
#define MFRC522_H

// #include "stm32f4xx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// 默认 SPI 实例和引脚
#define DEFAULT_SPI_PORT spi0
#define DEFAULT_PIN_MISO 16
#define DEFAULT_PIN_CS 17
#define DEFAULT_PIN_SCK 18
#define DEFAULT_PIN_MOSI 19
#define DEFAULT_PIN_RST 20                 // 新增默认 RST 引脚
#define DEFAULT_BAUDRATE (1 * 1000 * 1000) // 1MHz

#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

typedef struct
{
    spi_inst_t *spi;    // SPI 实例
    uint pin_miso;      // MISO 引脚
    uint pin_cs;        // CS 引脚
    uint pin_sck;       // SCK 引脚
    uint pin_mosi;      // MOSI 引脚
    uint pin_rst;       // RST 引脚（新增）
    uint32_t baud_rate; // 波特率
} MFRC522_Config;

/////////////////////////////////////////////////////////////////////
// MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE 0x00       // 取消当前命令
#define PCD_AUTHENT 0x0E    // 验证密钥
#define PCD_RECEIVE 0x08    // 接收数据
#define PCD_TRANSMIT 0x04   // 发送数据
#define PCD_TRANSCEIVE 0x0C // 发送并接收数据
#define PCD_RESETPHASE 0x0F // 复位
#define PCD_CALCCRC 0x03    // CRC计算

/////////////////////////////////////////////////////////////////////
// Mifare_1卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL 0x26    // 寻天线区内未进入休眠状态
#define PICC_REQALL 0x52    // 寻天线区内全部卡
#define PICC_ANTICOLL1 0x93 // 防冲撞
#define PICC_ANTICOLL2 0x95 // 防冲撞
#define PICC_AUTHENT1A 0x60 // 验证A密钥
#define PICC_AUTHENT1B 0x61 // 验证B密钥
#define PICC_READ 0x30      // 读块
#define PICC_WRITE 0xA0     // 写块
#define PICC_DECREMENT 0xC0 // 扣款
#define PICC_INCREMENT 0xC1 // 充值
#define PICC_RESTORE 0xC2   // 调块数据到缓冲区
#define PICC_TRANSFER 0xB0  // 保存缓冲区中数据
#define PICC_HALT 0x50      // 休眠
/////////////////////////////////////////////////////////////////////
// MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH 64 // FIFO size=64byte

/////////////////////////////////////////////////////////////////////
// MF522寄存器定义
/////////////////////////////////////////////////////////////////////
// PAGE 0
#define RFU00 0x00
#define CommandReg 0x01
#define ComIEnReg 0x02
#define DivlEnReg 0x03
#define ComIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define RFU0F 0x0F
// PAGE 1
#define RFU10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxAskReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define RFU1A 0x1A
#define RFU1B 0x1B
#define MifareReg 0x1C
#define RFU1D 0x1D
#define RFU1E 0x1E
#define SerialSpeedReg 0x1F
// PAGE 2
#define RFU20 0x20
#define CRCResultRegM 0x21
#define CRCResultRegL 0x22
#define RFU23 0x23
#define ModWidthReg 0x24
#define RFU25 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsCfgReg 0x28
#define ModGsCfgReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F
// PAGE 3
#define RFU30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define RFU3C 0x3C
#define RFU3D 0x3D
#define RFU3E 0x3E
#define RFU3F 0x3F

/////////////////////////////////////////////////////////////////////
// 和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define MI_OK 0
#define MI_NOTAGERR (0xff)
#define MI_ERR (0xfe)

// 使用 extern "C" 来确保 C++ 编译器使用 C 链接规范
#ifdef __cplusplus
extern "C"
{
#endif

    void MFRC522_Init(const MFRC522_Config *config);
    void MFRC522_DefaultInit(void);
    void MFRC522_Reset(void);
    unsigned char ReadRawRC(unsigned char addr);
    void WriteRawRC(unsigned char addr, unsigned char val);
    void ClearBitMask(unsigned char reg, unsigned char mask);
    void SetBitMask(unsigned char reg, unsigned char mask);
    void PcdAntennaOn(void);
    void PcdAntennaOff(void);
    char PcdReset(void);
    char PcdComMF522(unsigned char Command, unsigned char *pInData, unsigned char InLenByte, unsigned char *pOutData, unsigned int *pOutLenBit);
    void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData);
    char PcdRequest(unsigned char req_code, unsigned char *pTagType);
    char PcdAnticoll(unsigned char *pSnr);
    char PcdSelect(unsigned char *pSnr);
    char PcdAuthState(unsigned char auth_mode, unsigned char addr, unsigned char *pKey, unsigned char *pSnr);
    char PcdRead(unsigned char addr, unsigned char *pData);
    char PcdWrite(unsigned char addr, unsigned char *pData);
    char PcdHalt(void);
    char PcdValue(unsigned char dd_mode, unsigned char addr, unsigned char *pValue);
    char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);
    void RFID_Read_Data(unsigned char id);
    void RFID_Write_Data(unsigned char id);

#ifdef __cplusplus
}
#endif

#endif // MFRC522_H
