// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

#define ISR_PREFIX

bool initLoRaDriver(bool (*SPI_WriteRead)(void* pTransmitData, size_t txSize, void* pReceiveData, size_t rxSize);
                    bool (*SPI_Write)(void* pTransmitData, size_t txSize);
                    bool (*SPI_Read)(void* pReceiveData, size_t rxSize);
                    bool (*SPI_IsBusy)(void);
                    bool (*SPI_IsTransmitterBusy)(void));
uint32_t LoRaBegin(uint32_t frequency);
void LoRaEnd();

uint32_t LoRaBeginPacket(bool implicitHeader);
uint32_t LoRaEndPacket(bool async);

uint32_t LoRaParsePacket(uint32_t size);
uint32_t LoRaPacketRssi();
float LoRaPacketSnr();
uint32_t LoRaPacketFrequencyError();

uint32_t LoRaRssi();

  // from Print
//  virtual size_t write(uint8_t byte);
//  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
//  virtual int available();
//  virtual int read();
//  virtual int peek();
//  virtual void flush();

void LoRaOnReceive(void(*callback)(uint32_t));
void LoRaOnTxDone(void(*callback)(void ));

void LoRaReceive(uint32_t size);

void LoRaIdle();
void LoRaSleep();

void LoRaSetTxPower(uint32_t level, uint32_t outputPin/* = PA_OUTPUT_PA_BOOST_PIN*/);
void LoRaSetFrequency(uint32_t frequency);
void LoRaSetSpreadingFactor(uint32_t sf);
void LoRaSetSignalBandwidth(uint32_t sbw);
void LoRaSetCodingRate4(uint32_t denominator);
void LoRaSetPreambleLength(uint32_t length);
void LoRaSetSyncWord(uint32_t sw);
void LoRaEnableCrc();
void LoRaDisableCrc();
void LoRaEnableInvertIQ();
void LoRaDisableInvertIQ();
  
void LoRaSetOCP(uint8_t mA); // Over Current Protection control
  
void LoRaSetGain(uint8_t gain); // Set LNA gain

//void dumpRegisters(Stream& out);

void LoRaExplicitHeaderMode();
void LoRaImplicitHeaderMode();

void LoRaHandleDio0Rise();
bool LoRaIsTransmitting();

uint32_t LoRaGetSpreadingFactor();
uint32_t LoRaGetSignalBandwidth();

void LoRaSetLdoFlag();

uint8_t LoRaReadRegister(uint8_t address);
void LoRaWriteRegister(uint8_t address, uint8_t value);
uint8_t LoRaSingleTransfer(uint8_t address, uint8_t value);

static void LoRaOnDio0Rise();

//private:
//  SPISettings _spiSettings;
//  SPIClass* _spi;
//  uint32_t _ss;
//  uint32_t _reset;
//  uint32_t _dio0;
//  long _frequency;
//  uint32_t _packetIndex;
//  uint32_t _implicitHeaderMode;
//  void (*_onReceive)(uint32_t);
//  void (*_onTxDone)();

#ifdef	__cplusplus
}
#endif

#endif
