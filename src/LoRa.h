// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

// pins
#define LORA_PA_OUTPUT_RFO_PIN          0
#define LORA_PA_OUTPUT_PA_BOOST_PIN     1
// version
#define LORA_SEMTECH_VERSION          0x12
// registers
#define LORA_REG_FIFO                 0x00
#define LORA_REG_OP_MODE              0x01
#define LORA_REG_FRF_MSB              0x06
#define LORA_REG_FRF_MID              0x07
#define LORA_REG_FRF_LSB              0x08
#define LORA_REG_PA_CONFIG            0x09
#define LORA_REG_OCP                  0x0b
#define LORA_REG_LNA                  0x0c
#define LORA_REG_FIFO_ADDR_PTR        0x0d
#define LORA_REG_FIFO_TX_BASE_ADDR    0x0e
#define LORA_REG_FIFO_RX_BASE_ADDR    0x0f
#define LORA_REG_FIFO_RX_CURRENT_ADDR 0x10
#define LORA_REG_IRQ_FLAGS            0x12
#define LORA_REG_RX_NB_BYTES          0x13
#define LORA_REG_PKT_SNR_VALUE        0x19
#define LORA_REG_PKT_RSSI_VALUE       0x1a
#define LORA_REG_RSSI_VALUE           0x1b
#define LORA_REG_MODEM_CONFIG_1       0x1d
#define LORA_REG_MODEM_CONFIG_2       0x1e
#define LORA_REG_PREAMBLE_MSB         0x20
#define LORA_REG_PREAMBLE_LSB         0x21
#define LORA_REG_PAYLOAD_LENGTH       0x22
#define LORA_REG_MODEM_CONFIG_3       0x26
#define LORA_REG_FREQ_ERROR_MSB       0x28
#define LORA_REG_FREQ_ERROR_MID       0x29
#define LORA_REG_FREQ_ERROR_LSB       0x2a
#define LORA_REG_RSSI_WIDEBAND        0x2c
#define LORA_REG_DETECTION_OPTIMIZE   0x31
#define LORA_REG_INVERTIQ             0x33
#define LORA_REG_DETECTION_THRESHOLD  0x37
#define LORA_REG_SYNC_WORD            0x39
#define LORA_REG_INVERTIQ2            0x3b
#define LORA_REG_DIO_MAPPING_1        0x40
#define LORA_REG_VERSION              0x42
#define LORA_REG_PA_DAC               0x4d

// modes
#define LORA_MODE_LONG_RANGE_MODE     0x80
#define LORA_MODE_SLEEP               0x00
#define LORA_MODE_STDBY               0x01
#define LORA_MODE_TX                  0x03
#define LORA_MODE_RX_CONTINUOUS       0x05
#define LORA_MODE_RX_SINGLE           0x06

// PA config
#define LORA_PA_BOOST                 0x80

// IRQ masks
#define LORA_IRQ_TX_DONE_MASK           0x08
#define LORA_IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define LORA_IRQ_RX_DONE_MASK           0x40

#define LORA_RF_MID_BAND_THRESHOLD    525E6
#define LORA_RSSI_OFFSET_HF_PORT      157
#define LORA_RSSI_OFFSET_LF_PORT      164

#define LORA_MAX_PKT_LENGTH           255

#define LORA_ISR_PREFIX

bool initLoRaDriver(bool (*SPI_WriteRead)(void* pTransmitData, size_t txSize, void* pReceiveData, size_t rxSize);
                    bool (*SPI_Write)(void* pTransmitData, size_t txSize);
                    bool (*SPI_Read)(void* pReceiveData, size_t rxSize);
                    bool (*SPI_IsBusy)(void);
                    bool (*SPI_IsTransmitterBusy)(void),
                    void (*SPI_SetCSLow)(void),
                    void (*SPI_SetCSHigh)(void),
                    void (*setNRESETLow)(void),
                    void (*setNRESETHigh)(void),
                    void (*wait)(uint16_t ms)
                    )__attribute__((nonnull));
void LoRaReset(void);
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

void LoRaIdle(void);
void LoRaSleep(void);

void LoRaSetTxPower(uint32_t level, uint32_t outputPin/* = PA_OUTPUT_PA_BOOST_PIN*/);
void LoRaSetFrequency(uint32_t frequency);
void LoRaSetSpreadingFactor(uint32_t sf);
void LoRaSetSignalBandwidth(uint32_t sbw);
void LoRaSetCodingRate4(uint32_t denominator);
void LoRaSetPreambleLength(uint32_t length);
void LoRaSetSyncWord(uint32_t sw);
void LoRaEnableCrc(void);
void LoRaDisableCrc(void);
void LoRaEnableInvertIQ(void);
void LoRaDisableInvertIQ(void);
  
void LoRaSetOCP(uint8_t mA); // Over Current Protection control
  
void LoRaSetGain(uint8_t gain); // Set LNA gain

//void dumpRegisters(Stream& out);

void LoRaExplicitHeaderMode(void);
void LoRaImplicitHeaderMode();

void LoRaHandleDio0Rise(void);
bool LoRaIsTransmitting(void);

uint32_t LoRaGetSpreadingFactor(void);
uint32_t LoRaGetSignalBandwidth(void);

void LoRaSetLdoFlag(void);

uint8_t LoRaReadRegister(uint8_t address);
void LoRaWriteRegister(uint8_t address, uint8_t value);
uint8_t LoRaSingleTransfer(uint8_t address, uint8_t value);

static void LoRaOnDio0Rise(void);

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
