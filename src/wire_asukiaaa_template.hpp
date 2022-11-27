#pragma once

#include <Arduino.h>

namespace wire_asukiaaa {

struct ReadConfig {
  bool checkPresence = true;
  bool stopBitAfterWritingAddress = false;
};

static const ReadConfig defaultReadConfig;

template <class TemplateWire>
int checkPresence(TemplateWire* wire, uint8_t deviceAddress) {
  wire->beginTransmission(deviceAddress);
  return wire->endTransmission();
}

template <class TemplateWire>
int readBytes(TemplateWire* wire, uint8_t deviceAddress,
              uint8_t registerAddress, uint8_t* data, uint8_t dataLen,
              ReadConfig readConfig = defaultReadConfig) {
  uint8_t result;
  if (readConfig.checkPresence) {
    result = checkPresence(wire, deviceAddress);
    if (result != 0) {
      return result;
    }
    delay(15);
  }
  wire->beginTransmission(deviceAddress);
  wire->write(registerAddress);
  result = wire->endTransmission(readConfig.stopBitAfterWritingAddress);
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

template <class TemplateWire>
int writeBytes(TemplateWire* wire, uint8_t deviceAddress,
               uint8_t registerAddress, const uint8_t* data, uint8_t dataLen) {
  wire->beginTransmission(deviceAddress);
  wire->write(registerAddress);
  wire->write(data, dataLen);
  return wire->endTransmission();
}

template <class TemplateWire>
class PeripheralHandlerTemplate {
 public:
  uint8_t* buffs;
  const int buffLen;
  unsigned long receivedAt;
  int receivedLen;

  PeripheralHandlerTemplate(TemplateWire* wire, int buffLen = 0xff,
                            bool (*prohibitWriting)(int index) = NULL)
      : buffLen{buffLen} {
    this->wire = wire;
    this->prohibitWriting = prohibitWriting;
    buffs = new uint8_t[buffLen];
    for (int i = 0; i < buffLen; ++i) {
      buffs[0] = 0;
    }
    receivedLen = 0;
    receivedAt = 0;
  }

  ~PeripheralHandlerTemplate() { delete[] buffs; }

  void onReceive(int) {
    receivedLen = 0;
    while (0 < wire->available()) {
      uint8_t v = wire->read();
      if (receivedLen == 0) {
        buffIndex = v;
      } else {
        if (buffIndex < buffLen &&
            (prohibitWriting == NULL || !prohibitWriting(buffIndex))) {
          buffs[buffIndex] = v;
        }
        ++buffIndex;
      }
      ++receivedLen;
    }
    if (receivedLen > 0) {
      receivedAt = millis();
    }
  }

  void onRequest() {
    if (buffIndex < buffLen) {
#ifdef ARDUINO_ARCH_STM32
      wire->write(buffs[buffIndex++]);
#else
      wire->write(&buffs[buffIndex], buffLen - buffIndex);
#endif
    } else {
      wire->write(0);
    }
  }

 private:
  TemplateWire* wire;
  int buffIndex;
  bool (*prohibitWriting)(int index);
};

}  // namespace wire_asukiaaa
