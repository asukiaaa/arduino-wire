#include "wire_asukiaaa.h"

namespace wire_asukiaaa {
  int readBytes(TwoWire *wire, uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t dataLen) {
    wire->beginTransmission(deviceAddress);
    wire->write(registerAddress);
    uint8_t result = wire->endTransmission();
    if (result != 0) {
      return result;
    }

    wire->requestFrom(deviceAddress, dataLen);
    uint8_t index = 0;
    while (wire->available()) {
      uint8_t d = wire->read();
      if (index < dataLen) {
        data[index++] = d;
      }
    }
    return 0;
  }

  int writeBytes(TwoWire *wire, uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data, uint8_t dataLen) {
    wire->beginTransmission(deviceAddress);
    wire->write(registerAddress);
    wire->write(data, dataLen);
    return wire->endTransmission();
  }

  PeripheralHandler::PeripheralHandler(TwoWire* wire, int buffLen, bool (*prohibitWriting)(int index)): buffLen { buffLen } {
    this->wire = wire;
    this->prohibitWriting = prohibitWriting;
    buffs = new uint8_t[buffLen];
    for (int i = 0; i < buffLen; ++i) {
      buffs[0] = 0;
    }
    receivedLen = 0;
    receivedAt = 0;
  }

  PeripheralHandler::~PeripheralHandler() {
    delete[] buffs;
  }

  void PeripheralHandler::onReceive(int) {
    receivedLen = 0;
    while (0 < wire->available()) {
      uint8_t v = wire->read();
      if (receivedLen == 0) {
        buffIndex = v;
      } else {
        if (buffIndex < buffLen && (prohibitWriting == NULL || !prohibitWriting(buffIndex))) {
          buffs[buffIndex] = v;
        }
        ++buffIndex;
      }
      ++receivedLen;
    }
    receivedAt = millis();
  }

  void PeripheralHandler::onRequest() {
    if (buffIndex < buffLen) {
      wire->write(&buffs[buffIndex], buffLen - buffIndex);
    } else {
      wire->write(0);
    }
  }
}
