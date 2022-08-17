// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

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
