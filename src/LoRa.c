// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/**
 * @file:   LoRa.c
 * @author: apolisskyi
 *
 * @brief Semtech LoRa driver for Microchip SAM MCUs library realization.
 */
#include "LoRa.h"

struct LoRa_t {
    // TODO add here system sleep function
    bool (*SPI_WriteRead)(void *pTransmitData, size_t txSize, void *pReceiveData, size_t rxSize);

    bool (*SPI_Write)(void *pTransmitData, size_t txSize);

    bool (*SPI_Read)(void *pReceiveData, size_t rxSize);

    bool (*SPI_IsBusy)(void);

    bool (*SPI_IsTransmitterBusy)(void);

    void (*SPI_SetCSLow)(void);

    void (*SPI_SetCSHigh)(void);

    void (*setNRESETLow)(void);

    void (*setNRESETHigh)(void);

    void (*wait)(uint32_t ms);

    void (*_onTxDoneCb)(void);

    void (*_onReceiveCb)(uint32_t packetLength);

    bool _implicitHeaderMode;
    uint32_t _frequency;
    uint32_t _packetIndex;
};

struct LoRa_t LoRa;

bool LoRaInitDriver(bool (*SPI_WriteRead)(void *pTransmitData, size_t txSize, void *pReceiveData, size_t rxSize),
                    bool (*SPI_Write)(void *pTransmitData, size_t txSize),
                    bool (*SPI_Read)(void *pReceiveData, size_t rxSize),
                    bool (*SPI_IsBusy)(void),
                    bool (*SPI_IsTransmitterBusy)(void),
                    void (*SPI_SetCSLow)(void),
                    void (*SPI_SetCSHigh)(void),
                    void (*setNRESETLow)(void),
                    void (*setNRESETHigh)(void),
                    void (*wait)(uint32_t ms)
) {
    LoRa.SPI_WriteRead = SPI_WriteRead;
    LoRa.SPI_Write = SPI_Write;
    LoRa.SPI_Read = SPI_Read;
    LoRa.SPI_IsBusy = SPI_IsBusy;
    LoRa.SPI_IsTransmitterBusy = SPI_IsTransmitterBusy;
    LoRa.SPI_SetCSLow = SPI_SetCSLow;
    LoRa.SPI_SetCSHigh = SPI_SetCSHigh;
    LoRa.setNRESETLow = setNRESETLow;
    LoRa.setNRESETHigh = setNRESETHigh;
    LoRa.wait = wait;

    return true;
};

uint32_t LoRaBegin(uint32_t frequency) {
    // TODO add to the readme
    // expected that pins are set up
    // CS      -> OUTPUT, HIGH
    // NRESET  -> OUTPUT, HIGH
    // DIO     -> INPUT, Pull?
    // SPI MISO, MOSI, SCK already setup
    // SPI initializaed
    LoRaReset();

    // check version
    uint8_t version = LoRaReadRegister(LORA_REG_VERSION);
    assert(version == LORA_SEMTECH_VERSION && "unsupported SEMTECH version");

    // put in sleep mode
    LoRaSleep();

    // set frequency
    LoRaSetFrequency(frequency);

    // set base addresses
    LoRaWriteRegister(LORA_REG_FIFO_TX_BASE_ADDR, 0);
    LoRaWriteRegister(LORA_REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    uint8_t lnaVal = LoRaReadRegister(LORA_REG_LNA);
    LoRaWriteRegister(LORA_REG_LNA, lnaVal | 0x03);

    // set auto AGC
    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_3, 0x04);

    // set output power to 10 dBm
    LoRaSetTxPower(10, LORA_PA_OUTPUT_RFO_PIN);

    // put in standby mode
    LoRaIdle();

    return 0;
};

void LoRaReset(void) {
    LoRa.setNRESETLow();
    LoRa.wait(50);
    LoRa.setNRESETHigh();
    LoRa.wait(50);
};

void LoRaSleep() {
    LoRaWriteRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_SLEEP);
};

void LoRaSetFrequency(uint32_t frequency) {
    LoRa._frequency = frequency;

    uint64_t frf = ((uint64_t) frequency << 19) / 32000000;

    LoRaWriteRegister(LORA_REG_FRF_MSB, (uint8_t) (frf >> 16));
    LoRaWriteRegister(LORA_REG_FRF_MID, (uint8_t) (frf >> 8));
    LoRaWriteRegister(LORA_REG_FRF_LSB, (uint8_t) (frf >> 0));
};

void LoRaSetTxPower(uint32_t level, uint32_t outputPin) {
    if (LORA_PA_OUTPUT_RFO_PIN == outputPin) {
        // RFO
        if (level < 0) {
            level = 0;
        } else if (level > 14) {
            level = 14;
        }

        LoRaWriteRegister(LORA_REG_PA_CONFIG, 0x70 | level);
    } else {
        // PA BOOST
        if (level > 17) {
            if (level > 20) {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            LoRaWriteRegister(LORA_REG_PA_DAC, 0x87);
            LoRaSetOCP(140);
        } else {
            if (level < 2) {
                level = 2;
            }
            //Default value PA_HF/LF or +17dBm
            LoRaWriteRegister(LORA_REG_PA_DAC, 0x84);
            LoRaSetOCP(100);
        }

        LoRaWriteRegister(LORA_REG_PA_CONFIG, LORA_PA_BOOST | (level - 2));
    }
};

void LoRaSetOCP(uint8_t mA) {
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    } else if (mA <= 240) {
        ocpTrim = (mA + 30) / 10;
    }

    LoRaWriteRegister(LORA_REG_OCP, 0x20 | (0x1F & ocpTrim));
};

void LoRaIdle(void) {
    LoRaWriteRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_STDBY);
};

void LoRaWriteRegister(uint8_t address, uint8_t value) {
    LoRaSingleTransfer(address | 0x80, value);
};

uint8_t LoRaReadRegister(uint8_t address) {
    return LoRaSingleTransfer(address & 0x7F, 0x00);
};

uint8_t LoRaSingleTransfer(uint8_t address, uint8_t value) {
    uint8_t response[2] = {0, 0};

    LoRa.SPI_SetCSLow();

    LoRa.SPI_WriteRead((uint8_t[2]) {address, value}, 2, response, 2);
    while (LoRa.SPI_IsBusy());

    LoRa.SPI_SetCSHigh();

    return response[1];
};

uint32_t LoRaBeginPacket(bool implicitHeader) {
    if (LoRaIsTransmitting()) {
        return 0;
    }

    // put in standby mode
    LoRaIdle();

    if (implicitHeader) {
        LoRaImplicitHeaderMode();
    } else {
        LoRaExplicitHeaderMode();
    }

    // reset FIFO address and payload length
    LoRaWriteRegister(LORA_REG_FIFO_ADDR_PTR, 0);
    LoRaWriteRegister(LORA_REG_PAYLOAD_LENGTH, 0);

    return 1;
};

uint32_t LoRaEndPacket(bool async) {
    if ((async) && (LoRa._onTxDoneCb))
        LoRaWriteRegister(LORA_REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

    // put in TX mode
    LoRaWriteRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_TX);

    if (!async) {
        // wait for TX done
        while ((LoRaReadRegister(LORA_REG_IRQ_FLAGS) & LORA_IRQ_TX_DONE_MASK) == 0) {
            //      yield();
            // TODO add here system sleep function
        }
        // clear IRQ's
        LoRaWriteRegister(LORA_REG_IRQ_FLAGS, LORA_IRQ_TX_DONE_MASK);
    }

    return 1;
};

bool LoRaIsTransmitting(void) {
    if ((LoRaReadRegister(LORA_REG_OP_MODE) & LORA_MODE_TX) == LORA_MODE_TX)
        return true;

    if (LoRaReadRegister(LORA_REG_IRQ_FLAGS) & LORA_IRQ_TX_DONE_MASK)
        LoRaWriteRegister(LORA_REG_IRQ_FLAGS, LORA_IRQ_TX_DONE_MASK); // clear IRQ's

    return false;
};

uint32_t LoRaParsePacket(uint8_t size) {
    int packetLength = 0;
    int irqFlags = LoRaReadRegister(LORA_REG_IRQ_FLAGS);

    if (size > 0) {
        LoRaImplicitHeaderMode();

        LoRaWriteRegister(LORA_REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        LoRaExplicitHeaderMode();
    };

    // clear IRQ's
    LoRaWriteRegister(LORA_REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & LORA_IRQ_RX_DONE_MASK) && (irqFlags & LORA_IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        LoRa._packetIndex = 0;

        // read packet length
        if (LoRa._implicitHeaderMode) {
            packetLength = LoRaReadRegister(LORA_REG_PAYLOAD_LENGTH);
        } else {
            packetLength = LoRaReadRegister(LORA_REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        LoRaWriteRegister(LORA_REG_FIFO_ADDR_PTR, LoRaReadRegister(LORA_REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        LoRaIdle();
    } else if (LoRaReadRegister(LORA_REG_OP_MODE) != (LORA_MODE_LONG_RANGE_MODE | LORA_MODE_RX_SINGLE)) {
        // not currently in RX mode

        // reset FIFO address
        LoRaWriteRegister(LORA_REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        LoRaWriteRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_RX_SINGLE);
    }

    return packetLength;
}

uint32_t LoRaPacketRssi() {
    return (LoRaReadRegister(LORA_REG_PKT_RSSI_VALUE) -
            (LoRa._frequency < LORA_RF_MID_BAND_THRESHOLD ? LORA_RSSI_OFFSET_LF_PORT : LORA_RSSI_OFFSET_HF_PORT));
};

float LoRaPacketSnr() {
    return ((int8_t) LoRaReadRegister(LORA_REG_PKT_SNR_VALUE)) * 0.25;
};

uint32_t LoRaPacketFrequencyError() {
    int32_t freqError = 0;
    freqError = (int32_t) (LoRaReadRegister(LORA_REG_FREQ_ERROR_MSB) & 0b111);
    freqError <<= 8L;
    freqError += (int32_t) (LoRaReadRegister(LORA_REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += (int32_t) (LoRaReadRegister(LORA_REG_FREQ_ERROR_LSB));

    if (LoRaReadRegister(LORA_REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
        freqError -= 524288; // 0b1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError =
            (((float) (freqError) * (1L << 24)) / fXtal) * (LoRaGetSignalBandwidth() / 500000.0f); // p. 37

    return (uint32_t) (fError);
};

uint32_t LoRaRssi() {
    return (LoRaReadRegister(LORA_REG_RSSI_VALUE) -
            (LoRa._frequency < LORA_RF_MID_BAND_THRESHOLD ? LORA_RSSI_OFFSET_LF_PORT : LORA_RSSI_OFFSET_HF_PORT));
};

size_t LoRaWrite(const uint8_t *buffer, size_t size) {
    int currentLength = LoRaReadRegister(LORA_REG_PAYLOAD_LENGTH);

    // check size
    if ((currentLength + size) > LORA_MAX_PKT_LENGTH) {
        size = LORA_MAX_PKT_LENGTH - currentLength;
    }

    // write data
    for (size_t i = 0; i < size; i++) {
        LoRaWriteRegister(LORA_REG_FIFO, buffer[i]);
    }

    // update length
    LoRaWriteRegister(LORA_REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
};

uint32_t LoRaAvailable(void) {
    return (LoRaReadRegister(LORA_REG_RX_NB_BYTES) - LoRa._packetIndex);
};

uint8_t LoRaRead(void) {
    if (!LoRaAvailable()) {
        return -1;
    };

    LoRa._packetIndex++;

    return LoRaReadRegister(LORA_REG_FIFO);
};

uint8_t LoRaPeek(void) {
    if (!LoRaAvailable()) {
        return -1;
    }

    // store current FIFO address
    uint8_t currentAddress = LoRaReadRegister(LORA_REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = LoRaReadRegister(LORA_REG_FIFO);

    // restore FIFO address
    LoRaWriteRegister(LORA_REG_FIFO_ADDR_PTR, currentAddress);

    return b;
};

void LoRaSetOnReceive(void(*_onReceiveCb)(uint32_t packetLength)) {
    LoRa._onReceiveCb = _onReceiveCb;
};

void LoRaSetOnTxDone(void(*_onTxDoneCb)(void)) {
    LoRa._onTxDoneCb = _onTxDoneCb;
};

void LoRaReceive(uint8_t size) {
    LoRaWriteRegister(LORA_REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if (size > 0) {
        LoRaImplicitHeaderMode();

        LoRaWriteRegister(LORA_REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        LoRaExplicitHeaderMode();
    }

    LoRaWriteRegister(LORA_REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_RX_CONTINUOUS);
};

uint32_t LoRaGetSpreadingFactor() {
    return LoRaReadRegister(LORA_REG_MODEM_CONFIG_2) >> 4;
};

void LoRaSetSpreadingFactor(uint32_t sf) {
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }

    if (sf == 6) {
        LoRaWriteRegister(LORA_REG_DETECTION_OPTIMIZE, 0xc5);
        LoRaWriteRegister(LORA_REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        LoRaWriteRegister(LORA_REG_DETECTION_OPTIMIZE, 0xc3);
        LoRaWriteRegister(LORA_REG_DETECTION_THRESHOLD, 0x0a);
    }

    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_2, (LoRaReadRegister(LORA_REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    LoRaSetLdoFlag();
};

uint32_t LoRaGetSignalBandwidth() {
    uint8_t bw = (LoRaReadRegister(LORA_REG_MODEM_CONFIG_1) >> 4);

    switch (bw) {
        case 0:
            return 7.8E3;
        case 1:
            return 10.4E3;
        case 2:
            return 15.6E3;
        case 3:
            return 20.8E3;
        case 4:
            return 31.25E3;
        case 5:
            return 41.7E3;
        case 6:
            return 62.5E3;
        case 7:
            return 125E3;
        case 8:
            return 250E3;
        case 9:
            return 500E3;
    }

    return -1;
};

void LoRaSetSignalBandwidth(uint32_t sbw) {
    uint32_t bw = 0;

    if (sbw <= 7.8E3) {
        bw = 0;
    } else if (sbw <= 10.4E3) {
        bw = 1;
    } else if (sbw <= 15.6E3) {
        bw = 2;
    } else if (sbw <= 20.8E3) {
        bw = 3;
    } else if (sbw <= 31.25E3) {
        bw = 4;
    } else if (sbw <= 41.7E3) {
        bw = 5;
    } else if (sbw <= 62.5E3) {
        bw = 6;
    } else if (sbw <= 125E3) {
        bw = 7;
    } else if (sbw <= 250E3) {
        bw = 8;
    } else /*if (sbw <= 250E3)*/ {
        bw = 9;
    }

    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_1, (LoRaReadRegister(LORA_REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    LoRaSetLdoFlag();
};

void LoRaSetLdoFlag() {
    // Section 4.1.1.5
    uint32_t symbolDuration = 1000 / (LoRaGetSignalBandwidth() / (1L << LoRaGetSpreadingFactor()));

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = LoRaReadRegister(LORA_REG_MODEM_CONFIG_3);
    // TODO double check
    ldoOn
    ? config3 | 0b00001000
    : config3 & 0b11110111;

    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_3, config3);
};

void LoRaSetCodingRate4(uint32_t denominator) {
    if (denominator < 5) {
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }

    int cr = denominator - 4;

    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_1, (LoRaReadRegister(LORA_REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
};

void LoRaSetPreambleLength(uint32_t length) {
    LoRaWriteRegister(LORA_REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
    LoRaWriteRegister(LORA_REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
};

void LoRaSetSyncWord(uint32_t sw) {
    LoRaWriteRegister(LORA_REG_SYNC_WORD, sw);
};

void LoRaEnableCrc() {
    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_2, LoRaReadRegister(LORA_REG_MODEM_CONFIG_2) | 0x04);
};

void LoRaDisableCrc() {
    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_2, LoRaReadRegister(LORA_REG_MODEM_CONFIG_2) & 0xfb);
};

void LoRaEnableInvertIQ(void) {
    LoRaWriteRegister(LORA_REG_INVERTIQ, 0x66);
    LoRaWriteRegister(LORA_REG_INVERTIQ2, 0x19);
};

void LoRaDisableInvertIQ(void) {
    LoRaWriteRegister(LORA_REG_INVERTIQ, 0x27);
    LoRaWriteRegister(LORA_REG_INVERTIQ2, 0x1d);
};

void LoRaSetGain(uint8_t gain) {
    // check allowed range
    if (gain > 6) {
        gain = 6;
    }

    // set to standby
    LoRaIdle();

    // set gain
    if (gain == 0) {
        // if gain = 0, enable AGC
        LoRaWriteRegister(LORA_REG_MODEM_CONFIG_3, 0x04);
    } else {
        // disable AGC
        LoRaWriteRegister(LORA_REG_MODEM_CONFIG_3, 0x00);

        // clear Gain and set LNA boost
        LoRaWriteRegister(LORA_REG_LNA, 0x03);

        // set gain
        LoRaWriteRegister(LORA_REG_LNA, LoRaReadRegister(LORA_REG_LNA) | (gain << 5));
    }
};

void LoRaExplicitHeaderMode(void) {
    LoRa._implicitHeaderMode = false;
    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_1, LoRaReadRegister(LORA_REG_MODEM_CONFIG_1) & 0xfe);
};

void LoRaImplicitHeaderMode(void) {
    LoRa._implicitHeaderMode = true;
    LoRaWriteRegister(LORA_REG_MODEM_CONFIG_1, LoRaReadRegister(LORA_REG_MODEM_CONFIG_1) | 0x01);
};

// Interrupt handler
void LoRaHandleDio0Rise() {
    uint32_t irqFlags = LoRaReadRegister(LORA_REG_IRQ_FLAGS);

    // clear IRQ's
    LoRaWriteRegister(LORA_REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & LORA_IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

        if ((irqFlags & LORA_IRQ_RX_DONE_MASK) != 0) {
            // received a packet
            LoRa._packetIndex = 0;

            // read packet length
            uint32_t packetLength = LoRa._implicitHeaderMode ? LoRaReadRegister(LORA_REG_PAYLOAD_LENGTH) : LoRaReadRegister(
                    LORA_REG_RX_NB_BYTES);

            // set FIFO address to current RX address
            LoRaWriteRegister(LORA_REG_FIFO_ADDR_PTR, LoRaReadRegister(LORA_REG_FIFO_RX_CURRENT_ADDR));

            if (LoRa._onReceiveCb) {
                LoRa._onReceiveCb(packetLength);
            }
        } else if ((irqFlags & LORA_IRQ_TX_DONE_MASK) != 0) {
            // TODO
            if (LoRa._onTxDoneCb) {
                LoRa._onTxDoneCb();
            }
        }
    }
};
